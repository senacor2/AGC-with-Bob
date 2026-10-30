//! Zentrales Aggregat-Struct für den AGC-Systemzustand.
//!
//! Dieses Modul stellt [`AgcContext`] bereit — das einzige, zentral übergebene
//! Zustandsobjekt für den Verb-Dispatcher und alle Guidance-Programme.
//!
//! Vorher wurden `DisplayState`, `Executive` und `PhaseTable` separat übergeben.
//! Der Dispatcher (Sub-Tasks 2–5) benötigt Zugriff auf alle Zustände in einem
//! einzigen `&mut AgcContext`, damit Handler-Funktionszeiger typisiert sind als
//! `fn(&mut AgcContext)`.

use crate::alarm::AlarmState;
use crate::dap::DapContext;
use crate::display::input::InputLoadState;
use crate::display::verb_noun::DisplayState;
use crate::executive::waitlist::Waitlist;
use crate::executive::Executive;
use crate::guidance::p30_37::P30Output;
use crate::guidance::p37_70::P37EmState;
use crate::guidance::p40_47::{P40State, P41State, P47State};
use crate::guidance::p51_53::P51State;
use crate::guidance::p61_67::{P61State, P64State, P67State};
use crate::navigation::state_vector::StateVector;
use crate::restart::phase_table::PhaseTable;

// ---------------------------------------------------------------------------
// AlignmentCommand
// ---------------------------------------------------------------------------

/// IMU-Alignment-Befehl — gesetzt von V41 (Coarse Align) oder V42 (Fine Align).
///
/// Entspricht dem AGC-VBCOARK (V41) / IMUFINEK (V42)-Mechanismus. Die
/// P51/P52-Programme prüfen dieses Feld und führen den jeweiligen
/// Alignment-Schritt durch.
///
/// AGC source: `Comanche055/EXTENDED_VERBS.agc`, VBCOARK (V41) / IMUFINEK (V42).
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum AlignmentCommand {
    /// V41 — Coarse Align IMU. Setzt die IMU-Gimbals grob auf THETAD-Winkel.
    ///
    /// AGC source: `Comanche055/EXTENDED_VERBS.agc`, VBCOARK / IMUCOARK.
    CoarseAlign,

    /// V42 — Fine Align IMU. Erzeugt Gyro-Torquing-Pulse aus THETAD-Differenz.
    ///
    /// AGC source: `Comanche055/EXTENDED_VERBS.agc`, IMUFINEK / IMUFINE.
    FineAlign,
}

// ---------------------------------------------------------------------------
// GuidanceContext
// ---------------------------------------------------------------------------

/// Aggregat aller mission-kritischen Guidance-Programmzustände.
///
/// Jedes Feld ist `Option<T>` — `None` bedeutet, das Programm ist inaktiv.
/// Nur das jeweils aktive Programm hat `Some(...)`.
///
/// Die Signalisierungs-Flags (`proceed_flag`, `terminate_flag`, `recycle_flag`)
/// entsprechen den Crew-Tasten PRO (V33), TERM (V34), RSET (V32) und werden
/// vom Dispatcher gesetzt und von den P-Programm-Jobs gelesen.
///
/// AGC-Quelle: `Comanche055/FRESH_START_AND_RESTART.agc`, Abschnitt MODREG/MODRST.
#[derive(Debug)]
pub struct GuidanceContext {
    /// Aktive Programm-Nummer (MODREG), `None` wenn kein Programm läuft.
    pub active_program: Option<u8>,

    // -----------------------------------------------------------------------
    // Crew-Signalisierungs-Flags
    // -----------------------------------------------------------------------
    /// PROCEED-Flag: wird gesetzt wenn die Crew PRO drückt (V33).
    /// AGC-Quelle: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, PROCKEY.
    pub proceed_flag: bool,

    /// TERMINATE-Flag: wird gesetzt wenn die Crew TERM drückt (V34).
    /// AGC-Quelle: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, TERMKEY.
    pub terminate_flag: bool,

    /// RECYCLE-Flag: wird gesetzt wenn die Crew RSET drückt (V32).
    /// AGC-Quelle: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, RSETKEY.
    pub recycle_flag: bool,

    // -----------------------------------------------------------------------
    // Reentry-Ziel-Parameter (für P61–P67)
    // -----------------------------------------------------------------------
    /// Splashdown-Ziel-Breite [rad] — LAT(SPL), Crew-Eingabe über V25/N61.
    /// AGC-Quelle: `Comanche055/ASSEMBLY_AND_OPERATION_INFORMATION.agc`, N61.
    pub splashdown_lat: f64,

    /// Splashdown-Ziel-Länge [rad] — LNG(SPL), Crew-Eingabe über V25/N61.
    /// AGC-Quelle: `Comanche055/ASSEMBLY_AND_OPERATION_INFORMATION.agc`, N61.
    pub splashdown_lon: f64,

    /// HEADSUP-Flag — `true` = Lift-Down (Hitzeschild nach oben), Crew-Eingabe über V25/N61.
    /// AGC-Quelle: `Comanche055/ASSEMBLY_AND_OPERATION_INFORMATION.agc`, N61.
    pub heads_up: bool,

    // -----------------------------------------------------------------------
    // Fahrzeugparameter + Burn-Planung
    // -----------------------------------------------------------------------
    /// Fahrzeugmasse vor dem nächsten Burn [kg].
    ///
    /// Wird aus dem Szenario geladen (`mass_kg`-Feld) und von P40 als
    /// Startmasse für die Tsiolkovsky-Gleichung (Brennzeit-Berechnung) verwendet.
    /// Standardwert: 28 800 kg (Apollo CSM-Nominalmasse vor TEI).
    ///
    /// AGC-Quelle: `Comanche055/P40-P47.agc`, FENG / EMDOT (pad-geladene Masse).
    pub vehicle_mass_kg: f64,

    /// Vorausberechneter Zündzeitpunkt TIG [s] — Mission Elapsed Time.
    ///
    /// Wird von der Crew per `V25 N33` eingegeben oder aus dem Szenario geladen
    /// (`tig_s`-Feld). P30 propagiert den aktuellen Bahnzustandsvektor von der
    /// aktuellen MET bis zum TIG (Kepler-Schritt), bevor der Burn-Plan berechnet
    /// wird. P40 zündet automatisch wenn `ctx.current_sv.met >= tig_s`.
    ///
    /// `0.0` bedeutet: kein TIG gesetzt — P30 benutzt den aktuellen SV direkt
    /// (Verhalten wie bisher).
    ///
    /// AGC-Quelle: `Comanche055/PINBALL_NOUN_TABLES.agc`, N33 (TIG).
    pub tig_s: f64,

    // -----------------------------------------------------------------------
    // P30-Eingabedaten (Crew via V25/N81)
    // -----------------------------------------------------------------------
    /// Δv-Manövervektor im lokalen Vertikalsystem [radial, along-track, normal] [m/s].
    ///
    /// Wird von der Crew per V25/N81 eingegeben. P30 liest diesen Wert als
    /// Manöverimpuls. Ist `[0.0; 3]` solange noch keine Eingabe erfolgt ist.
    ///
    /// AGC-Quelle: `Comanche055/P30,P37.agc`, DELVSLV (DELVLVC-Eingabe).
    pub delta_v_lv: [f64; 3],

    // -----------------------------------------------------------------------
    // Programmzustände — TEI / LOI (P30, P37-EM)
    // -----------------------------------------------------------------------
    /// Letzter berechneter P30-External-ΔV-Targeting-Output, `None` wenn P30 nie lief.
    /// AGC-Quelle: `Comanche055/P30,P37.agc`, P30CALL.
    pub p30: Option<P30Output>,

    /// Laufzustand P37-Emergency-Return-to-Earth, `None` wenn P37-EM inaktiv.
    /// AGC-Quelle: `Comanche055/P37,P70.agc`, P37EMRG.
    pub p37_em: Option<P37EmState>,

    // -----------------------------------------------------------------------
    // Programmzustände — SPS-Burn (P40, P41, P47)
    // -----------------------------------------------------------------------
    /// Laufzustand P40 — SPS-Haupttriebwerksbrand, `None` wenn P40 inaktiv.
    /// AGC-Quelle: `Comanche055/P40-P47.agc`, P40CALL.
    pub p40: Option<P40State>,

    /// Laufzustand P41 — Gimbal-Motor-Test, `None` wenn P41 inaktiv.
    /// AGC-Quelle: `Comanche055/P40-P47.agc`, P41CALL.
    pub p41: Option<P41State>,

    /// Laufzustand P47 — Trim-Manöver, `None` wenn P47 inaktiv.
    /// AGC-Quelle: `Comanche055/P40-P47.agc`, P47CALL.
    pub p47: Option<P47State>,

    // -----------------------------------------------------------------------
    // Programmzustände — IMU-Alignment (P51)
    // -----------------------------------------------------------------------
    /// Laufzustand P51 — IMU-Alignment (Sternmessungen), `None` wenn P51 inaktiv.
    /// AGC-Quelle: `Comanche055/P51-P53.agc`, P51CALL.
    pub p51: Option<P51State>,

    /// Ausstehender IMU-Alignment-Befehl von V41 (Coarse Align) oder V42 (Fine Align).
    ///
    /// Wird vom Dispatcher gesetzt und von P51/P52 konsumiert (auf `None` zurückgesetzt
    /// wenn verarbeitet).
    ///
    /// AGC source: `Comanche055/EXTENDED_VERBS.agc`, VBCOARK / IMUFINEK.
    pub alignment_cmd: Option<AlignmentCommand>,

    /// Ziel-Gimbal-Winkel für Coarse/Fine Align [rad] — THETAD (OGA, MGA, IGA).
    ///
    /// Werden von V41/V42 aus dem `DisplayState` (R1/R2/R3 nach V25/N22-Eingabe) übernommen
    /// oder direkt vom P52-Programm gesetzt.
    ///
    /// AGC source: `Comanche055/EXTENDED_VERBS.agc`, VBCOARK (THETAD reference).
    pub thetad: [f64; 3],

    // -----------------------------------------------------------------------
    // Programmzustände — Reentry (P61, P64, P67)
    // -----------------------------------------------------------------------
    /// Laufzustand P61 — Entry Preparation, `None` wenn P61 inaktiv.
    /// AGC-Quelle: `Comanche055/P61-P67.agc`, P61CALL.
    pub p61: Option<P61State>,

    /// Laufzustand P64 — Constant-Drag Entry Phase, `None` wenn P64 inaktiv.
    /// AGC-Quelle: `Comanche055/P61-P67.agc`, P64CALL.
    pub p64: Option<P64State>,

    /// Laufzustand P67 — Terminal Phase, `None` wenn P67 inaktiv.
    /// AGC-Quelle: `Comanche055/P61-P67.agc`, P67CALL.
    pub p67: Option<P67State>,
}

impl Default for GuidanceContext {
    fn default() -> Self {
        Self {
            active_program: None,
            proceed_flag: false,
            terminate_flag: false,
            recycle_flag: false,
            splashdown_lat: 0.0,
            splashdown_lon: 0.0,
            heads_up: false,
            vehicle_mass_kg: 28_800.0,
            tig_s: 0.0,
            delta_v_lv: [0.0; 3],
            p30: None,
            p37_em: None,
            p40: None,
            p41: None,
            p47: None,
            p51: None,
            alignment_cmd: None,
            thetad: [0.0; 3],
            p61: None,
            p64: None,
            p67: None,
        }
    }
}

// ---------------------------------------------------------------------------
// AgcContext
// ---------------------------------------------------------------------------

/// Zentrales Aggregat des gesamten AGC-Systemzustands.
///
/// Dieser Typ ist der einzige Parameter von Handler-Funktionen im
/// `VerbDispatcher` — alle Handler haben die Signatur `fn(&mut AgcContext)`.
///
/// Enthält alle bisher separat übergebenen Zustände:
/// - [`DisplayState`][]: DSKY-Anzeigestatus (PINBALL-Erasable)
/// - [`Executive`][]: kooperativer Job-Scheduler
/// - [`PhaseTable`][]: restart-sichere Fortschrittsverfolgung
/// - [`GuidanceContext`][]: Aggregat aller Guidance-Programmzustände
/// - [`StateVector`][]: aktueller Bahnzustandsvektor (wird vor jedem Tick aktualisiert)
/// - [`AlarmState`][]: Program-Alarm-Zustand
#[derive(Debug, Default)]
pub struct AgcContext {
    /// DSKY-Anzeigestatus (VERBREG, NOUNREG, MODREG, R1–R3, Statuslichter).
    pub display: DisplayState,

    /// Digitaler Autopilot — erasable-memory-Brücke (CDU-Eingabe, RCS-Ausgabe).
    ///
    /// Der Sim-Loop schreibt `dap.cdu_angles` vor dem Waitlist-Tick und liest
    /// `dap.jets_ab` / `dap.jets_cd` danach.  Der DAP-Task läuft als sich
    /// selbst wiederholender Waitlist-Task (10 cs = 100 ms).
    ///
    /// AGC source: `Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc`, DAPDATR1/DAPDATR2.
    pub dap: DapContext,

    /// Kooperativer Job-Scheduler (Executive).
    pub exec: Executive,

    /// Restart-sichere Phase-Table aller aktiven Gruppen.
    pub phase_table: PhaseTable,

    /// Aggregat aller Guidance-Programmzustände.
    pub guidance: GuidanceContext,

    /// Aktueller Bahn-Zustandsvektor (ECI J2000, m / m/s).
    ///
    /// Wird von der Sim-Loop vor jedem Dispatch-Tick aktualisiert.
    /// Guidance-Programme (z. B. P30) lesen diesen Wert als TIG-Zustandsvektor.
    ///
    /// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc`, RATT/VATT.
    pub current_sv: StateVector,

    /// Program-Alarm-Zustand (FAILREG-Warteschlange).
    ///
    /// Guidance-Berechnungen schreiben Alarm-Codes hierein; die Sim-Loop
    /// liest den Zustand für die TUI-Alarmanzeige aus.
    ///
    /// AGC source: `Comanche055/ALARM_AND_ABORT.agc`.
    pub alarms: AlarmState,

    /// Zustand des V25-Dateneingabe-Automaten (PINBALL ENTPAS/STOREDAT).
    ///
    /// Akkumuliert Ziffern, Vorzeichen und Register-Puffer für V25-Eingaben.
    /// Wird von [`crate::display::input::process_v25_key`] verwaltet.
    ///
    /// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,
    /// ENTPAS / STOREDAT.
    pub load_state: InputLoadState,

    /// Zeitgesteuerte Warteschlange (Waitlist) für verzögerte Tasks.
    ///
    /// Analog zu LST1/LST2 im AGC — verwaltet bis zu
    /// [`crate::executive::waitlist::MAX_TASKS`] Tasks, die nach einer
    /// Verzögerung in Zentisekunden ausgeführt werden. Task-Funktionen erhalten
    /// `&mut AgcContext` und können sich selbst via `ctx.waitlist.call_after(...)`
    /// erneut einplanen (selbst-wiederholendes Pattern für Monitor-Tasks).
    ///
    /// AGC source: `Comanche055/WAITLIST.agc`, LST1/LST2 memory layout.
    pub waitlist: Waitlist,
}
