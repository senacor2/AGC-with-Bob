//! Zentrales Aggregat-Struct für den AGC-Systemzustand.
//!
//! Dieses Modul stellt [`AgcContext`] bereit — das einzige, zentral übergebene
//! Zustandsobjekt für den Verb-Dispatcher und alle Guidance-Programme.
//!
//! Vorher wurden `DisplayState`, `Executive` und `PhaseTable` separat übergeben.
//! Der Dispatcher (Sub-Tasks 2–5) benötigt Zugriff auf alle Zustände in einem
//! einzigen `&mut AgcContext`, damit Handler-Funktionszeiger typisiert sind als
//! `fn(&mut AgcContext)`.

use crate::display::verb_noun::DisplayState;
use crate::executive::Executive;
use crate::guidance::p30_37::P30Output;
use crate::guidance::p37_70::P37EmState;
use crate::guidance::p40_47::{P40State, P41State, P47State};
use crate::guidance::p51_53::P51State;
use crate::guidance::p61_67::{P61State, P64State, P67State};
use crate::restart::phase_table::PhaseTable;

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
            p30: None,
            p37_em: None,
            p40: None,
            p41: None,
            p47: None,
            p51: None,
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
#[derive(Debug, Default)]
pub struct AgcContext {
    /// DSKY-Anzeigestatus (VERBREG, NOUNREG, MODREG, R1–R3, Statuslichter).
    pub display: DisplayState,

    /// Kooperativer Job-Scheduler (Executive).
    pub exec: Executive,

    /// Restart-sichere Phase-Table aller aktiven Gruppen.
    pub phase_table: PhaseTable,

    /// Aggregat aller Guidance-Programmzustände.
    pub guidance: GuidanceContext,
}
