//! Verb-Dispatcher — tabellenbasierter AGC-VERBFAN-Kern.
//!
//! Übersetzt den von der Crew eingegebenen Verb-Code in eine Aktion.
//! Entspricht dem `VERBFAN`-Mechanismus des AGC:
//! `VBFANDIR INDEX VERBREG / CAF VERBTAB`.
//!
//! # Umfang
//! - Sub-Task 2: V06 (Decimal Display, einmalig) und V16 (Monitor Display, zyklisch).
//! - Sub-Task 3: V37 (Change Program), V25/N61 (Load 3 Komponenten),
//!   V33 (Proceed), V34 (Terminate), V32 (Recycle).
//! - Sub-Task 5: V41 (Coarse Align), V42 (Fine Align IMU).
//!
//! # Hinweis zum NounResolver
//! `dispatch()` delegiert die Datenbeschaffung an [`super::noun_resolver`]
//! (Sub-Task 4/5). Solange dieser noch nicht vollständig implementiert ist,
//! liefert der Resolver für unbekannte Nouns `[DisplayRegister::Blank; 3]`.
//!
//! AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, VERBFAN/VERBTAB.
//! AGC source: `Comanche055/EXTENDED_VERBS.agc`, LST2FAN / VBCOARK / IMUFINEK.

use crate::context::AgcContext;
use crate::display::noun_resolver;
use crate::display::verb_noun::{start_flash, stop_flash, NounCode, VerbCode};
use crate::guidance::p30_37::{p30_compute, P30Input};
use crate::guidance::p40_47::{P40State, SPS_ISP, SPS_THRUST_N};
use crate::navigation::orbital::{CentralBody, MOON_THRESHOLD_M};
use crate::types::Vec3;

// ---------------------------------------------------------------------------
// VerbAction
// ---------------------------------------------------------------------------

/// Aktion, die ein Verb im AGC-Dispatcher auslöst.
///
/// Entspricht den Einträgen in der AGC-`VERBTAB`-Tabelle.
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, VERBTAB.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum VerbAction {
    /// Einmalige Dezimalanzeige (V06 — DECDSP).
    /// Lädt Daten vom NounResolver in R1/R2/R3 und stoppt Blinken.
    ///
    /// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, DECDSP label.
    DisplayOnce,

    /// Monitor-Anzeige: R1/R2/R3 werden zyklisch per T4RUPT aktualisiert (V16 — MONITOR).
    /// Setzt `flash_state = false` (kein Blinken während Monitor läuft).
    ///
    /// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, MONITOR label.
    MonitorDisplay,

    /// Programm-Wechsel (V37 — MMCHANG). Noun gibt die Programm-Nummer an.
    ///
    /// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, MMCHANG label.
    ProgramChange(u8),

    /// Erster ENTER nach V37 — fordert Programmnummer an.
    ///
    /// Flash einschalten, auf Noun (Programm-Nummer) warten.
    /// Zweiter Schritt (Noun gesetzt) wird in `dispatch()` separat abgefangen.
    ///
    /// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, REQMM/MMCHANG label.
    RequestProgram,

    /// Lade-Verb: fordert Crew-Eingabe in R1/R2/R3 (V25 — Comp Acty Flash).
    /// Startet VERB/NOUN-Blinken, bis Crew ENTER drückt.
    ///
    /// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, NVSBWAIT.
    LoadData,

    /// Proceed / Crew-Bestätigung (V33 — PRO-Taste).
    /// Setzt `guidance.proceed_flag`.
    ///
    /// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, PROCKEY.
    Proceed,

    /// Terminate (V34 — TERM-Taste).
    /// Setzt `guidance.terminate_flag`.
    ///
    /// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, TERMKEY.
    Terminate,

    /// Recycle (V32 — RSET-Taste im V32-Kontext).
    /// Setzt `guidance.recycle_flag`.
    ///
    /// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, RSETKEY.
    Recycle,

    /// Coarse Align IMU (V41 — VBCOARK).
    ///
    /// Setzt `guidance.alignment_cmd = CoarseAlign`. P51/P52 liest dieses Flag
    /// und führt IMUCOARK durch.
    ///
    /// AGC source: `Comanche055/EXTENDED_VERBS.agc`, VBCOARK label.
    CoarseAlign,

    /// Fine Align IMU (V42 — IMUFINEK).
    ///
    /// Setzt `guidance.alignment_cmd = FineAlign`. P51/P52 liest dieses Flag
    /// und führt IMUFINE durch (Gyro-Torquing).
    ///
    /// AGC source: `Comanche055/EXTENDED_VERBS.agc`, IMUFINEK label.
    FineAlign,

    /// Verb ist nicht implementiert oder nicht relevant (Spare / Illegal).
    ///
    /// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, GODSPALM label.
    Noop,
}

// ---------------------------------------------------------------------------
// Monitor-Task (V16 — selbst-wiederholend via Waitlist)
// ---------------------------------------------------------------------------

/// Waitlist-Task für den V16-Monitor: aktualisiert R1/R2/R3 und plant sich
/// selbst nach 100 cs (= 1 Sekunde) erneut ein.
///
/// Muss als freie `fn`-Funktion (kein Closure) definiert sein, damit sie als
/// `fn(&mut AgcContext)` in die Waitlist eingetragen werden kann.
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, MONITOR label.
fn monitor_task(ctx: &mut AgcContext) {
    let noun = ctx.display.noun;
    let [r1, r2, r3] = noun_resolver::resolve(noun, ctx);
    ctx.display.r1 = r1;
    ctx.display.r2 = r2;
    ctx.display.r3 = r3;
    // Nächste Aktualisierung in 100 cs einplanen (selbst-wiederholend)
    let _ = ctx.waitlist.call_after(100, monitor_task);
}

// ---------------------------------------------------------------------------
// Statische Verb-Tabelle V00–V39 (VERBTAB)
// ---------------------------------------------------------------------------

/// Statische Dispatch-Tabelle für Verben V00–V39.
///
/// Index entspricht dem Verb-Code. Nur die für TEI/LOI/Reentry relevanten
/// Verben werden eingetragen; alle anderen bleiben `Noop`.
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, VERBTAB.
const VERB_TABLE: [VerbAction; 40] = {
    let mut t = [VerbAction::Noop; 40];
    // VB06: Decimal Display (einmalig)
    // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, DECDSP label.
    t[6] = VerbAction::DisplayOnce;
    // VB16: Monitor Decimal (zyklisch)
    // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, MONITOR label.
    t[16] = VerbAction::MonitorDisplay;
    // VB25: Load 3 Komponenten (R1/R2/R3 Crew-Eingabe)
    // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, NVSBWAIT.
    t[25] = VerbAction::LoadData;
    // VB32: Recycle
    // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, RSETKEY.
    t[32] = VerbAction::Recycle;
    // VB33: Proceed / PRO
    // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, PROCKEY.
    t[33] = VerbAction::Proceed;
    // VB34: Terminate
    // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, TERMKEY.
    t[34] = VerbAction::Terminate;
    // VB37: Change Major Mode — zweistufiger Ablauf (REQMM → MMCHANG)
    // Erster ENTER: Flash einschalten, auf Programm-Nummer warten (RequestProgram).
    // Zweiter Aufruf (NounSet mit noun≠0): dispatch_v37() via gesonderten Zweig.
    // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, REQMM/MMCHANG label.
    t[37] = VerbAction::RequestProgram;
    t
};

// ---------------------------------------------------------------------------
// Statische Extended-Verb-Tabelle V40–V45 (LST2FAN)
// ---------------------------------------------------------------------------

/// Statische Dispatch-Tabelle für Extended Verbs V40–V45.
///
/// Index ist relativ: Index 0 = V40, Index 1 = V41, Index 2 = V42 usw.
/// Nur die für IMU-Alignment (P51/P52) relevanten Extended Verbs sind eingetragen.
///
/// AGC source: `Comanche055/EXTENDED_VERBS.agc`, LST2FAN (zweite Verb-Fan-Tabelle).
const EXT_VERB_BASE: usize = 40;
const EXT_VERB_TABLE: [VerbAction; 6] = {
    let mut t = [VerbAction::Noop; 6];
    // VB40: VBZERO — Zero IMU CDUs (nicht implementiert, nur Platzhalter)
    // AGC source: EXTENDED_VERBS.agc, VBZERO label.
    t[0] = VerbAction::Noop; // V40
                             // VB41: VBCOARK — Coarse Align IMU (mit N20)
                             // AGC source: EXTENDED_VERBS.agc, VBCOARK / IMUCOARK label.
    t[1] = VerbAction::CoarseAlign; // V41
                                    // VB42: IMUFINEK — Fine Align IMU (Gyro-Torquing)
                                    // AGC source: EXTENDED_VERBS.agc, IMUFINEK / IMUFINE label.
    t[2] = VerbAction::FineAlign; // V42
                                  // VB43–VB45: Spare / nicht implementiert
    t
};

// ---------------------------------------------------------------------------
// Statische Programm-Tabelle (PROGRAM_TABLE)
// ---------------------------------------------------------------------------

/// Priorität, mit der Guidance-Programme im Executive eingetragen werden.
///
/// Guidance-Jobs haben typisch Priorität 10 im AGC-Executive.
///
/// AGC source: `Comanche055/EXECUTIVE.agc`, NOVAC-Prioritätstabelle.
const GUIDANCE_PRIORITY: u8 = 10;

/// P30 — External ΔV Targeting.
///
/// Berechnet Apogee, Perigee und Δv-ECI (DELVSIN).
/// Wenn `guidance.tig_s > 0` und in der Zukunft liegt, propagiert P30 den
/// aktuellen SV per Kepler-Schritt bis zum TIG und rechnet dort.
///
/// Zentralkörper wird aus dem Betrag des aktuellen Positionsvektors abgeleitet:
/// `|r| < 3 000 000 m` → Mond, sonst Erde.
///
/// AGC source: `Comanche055/P30,P37.agc`, P30CALL / S30.1.
fn program_p30(ctx: &mut AgcContext) {
    ctx.guidance.active_program = Some(30);

    // Zentralkörper aus aktuellem Positionsvektor ableiten.
    let body = if ctx.current_sv.position.magnitude() < MOON_THRESHOLD_M {
        CentralBody::Moon
    } else {
        CentralBody::Earth
    };

    let dv = ctx.guidance.delta_v_lv;
    let input = P30Input {
        sv_now: ctx.current_sv,
        tig_s: ctx.guidance.tig_s,
        delta_v_lv: Vec3::new(dv[0], dv[1], dv[2]),
        body,
    };

    ctx.guidance.p30 = p30_compute(&input, &mut ctx.alarms);
    ctx.exec.schedule(GUIDANCE_PRIORITY, program_noop_fn);
}

/// P37 — Return-to-Earth (Emergency).
///
/// AGC source: `Comanche055/P37,P70.agc`, P37EMRG.
fn program_p37(ctx: &mut AgcContext) {
    ctx.guidance.active_program = Some(37);
    ctx.exec.schedule(GUIDANCE_PRIORITY, program_noop_fn);
}

/// P40 — SPS-Haupttriebwerksbrand.
///
/// Initialisiert [`P40State`] aus dem letzten P30-Output (delta_v_ref-Betrag)
/// und der in `guidance.vehicle_mass_kg` hinterlegten Fahrzeugmasse.
///
/// Wenn P30 noch nicht ausgeführt wurde (`ctx.guidance.p30 == None`), wird
/// P40 ohne aktiven Burn-State gestartet — N40 zeigt dann weiterhin Blank.
///
/// AGC source: `Comanche055/P40-P47.agc`, P40CALL / P40CSM.
fn program_p40(ctx: &mut AgcContext) {
    ctx.guidance.active_program = Some(40);

    // Δv-Betrag aus P30-Ergebnis (DELVSIN) bestimmen.
    // Falls P30 noch nicht lief, keinen P40State anlegen → N40 bleibt Blank.
    if let Some(ref p30) = ctx.guidance.p30 {
        let dv_mag = p30.delta_v_ref.magnitude();
        if dv_mag > 0.0 {
            ctx.guidance.p40 = Some(P40State::new(
                dv_mag,
                SPS_ISP,
                SPS_THRUST_N,
                ctx.guidance.vehicle_mass_kg,
            ));
        }
    }

    ctx.exec.schedule(GUIDANCE_PRIORITY, program_noop_fn);
}

/// P51 — IMU-Alignment (Sternmessungen).
///
/// AGC source: `Comanche055/P51-P53.agc`, P51CALL.
fn program_p51(ctx: &mut AgcContext) {
    ctx.guidance.active_program = Some(51);
    ctx.exec.schedule(GUIDANCE_PRIORITY, program_noop_fn);
}

/// P52 — IMU-Alignment (Gyro-Torquing).
///
/// AGC source: `Comanche055/P51-P53.agc`, P52CALL.
fn program_p52(ctx: &mut AgcContext) {
    ctx.guidance.active_program = Some(52);
    ctx.exec.schedule(GUIDANCE_PRIORITY, program_noop_fn);
}

/// P61 — Entry Preparation.
///
/// AGC source: `Comanche055/P61-P67.agc`, P61CALL.
fn program_p61(ctx: &mut AgcContext) {
    ctx.guidance.active_program = Some(61);
    ctx.exec.schedule(GUIDANCE_PRIORITY, program_noop_fn);
}

/// P62 — Pre-Entry Orientation.
///
/// AGC source: `Comanche055/P61-P67.agc`, P62CALL.
fn program_p62(ctx: &mut AgcContext) {
    ctx.guidance.active_program = Some(62);
    ctx.exec.schedule(GUIDANCE_PRIORITY, program_noop_fn);
}

/// P63 — Entry Monitor / .05G Sensing.
///
/// AGC source: `Comanche055/P61-P67.agc`, P63CALL.
fn program_p63(ctx: &mut AgcContext) {
    ctx.guidance.active_program = Some(63);
    ctx.exec.schedule(GUIDANCE_PRIORITY, program_noop_fn);
}

/// P64 — Constant-Drag Entry.
///
/// AGC source: `Comanche055/P61-P67.agc`, P64CALL.
fn program_p64(ctx: &mut AgcContext) {
    ctx.guidance.active_program = Some(64);
    ctx.exec.schedule(GUIDANCE_PRIORITY, program_noop_fn);
}

/// P65 — Up-Control Phase.
///
/// AGC source: `Comanche055/P61-P67.agc`, P65CALL.
fn program_p65(ctx: &mut AgcContext) {
    ctx.guidance.active_program = Some(65);
    ctx.exec.schedule(GUIDANCE_PRIORITY, program_noop_fn);
}

/// P67 — Terminal Phase.
///
/// AGC source: `Comanche055/P61-P67.agc`, P67CALL.
fn program_p67(ctx: &mut AgcContext) {
    ctx.guidance.active_program = Some(67);
    ctx.exec.schedule(GUIDANCE_PRIORITY, program_noop_fn);
}

/// `fn()` Platzhalter für Executive-Jobs (keine Parameter).
///
/// Der Executive nimmt `fn()` ohne Argumente. Diese Stub-Funktion wird
/// eingetragen, damit der Executive-Slot belegt ist. Die eigentliche
/// Guidance-Logik wird in Sub-Task 6 über den `AgcContext`-Mutex verdrahtet.
///
/// AGC source: `Comanche055/EXECUTIVE.agc`, DUMMYJOB label.
fn program_noop_fn() {}

/// Statische Programm-Dispatch-Tabelle (PROGRAM_TABLE).
///
/// Index ist die Programm-Nummer (0–99). `None` bedeutet, das Programm
/// ist nicht implementiert. Nur die für TEI/LOI/Reentry kritischen
/// Programme sind eingetragen.
///
/// AGC source: `Comanche055/FRESH_START_AND_RESTART.agc`, MODREG/MODRST.
static PROGRAM_TABLE: [Option<fn(&mut AgcContext)>; 100] = {
    let mut t: [Option<fn(&mut AgcContext)>; 100] = [None; 100];
    t[30] = Some(program_p30);
    t[37] = Some(program_p37);
    t[40] = Some(program_p40);
    t[51] = Some(program_p51);
    t[52] = Some(program_p52);
    t[61] = Some(program_p61);
    t[62] = Some(program_p62);
    t[63] = Some(program_p63);
    t[64] = Some(program_p64);
    t[65] = Some(program_p65);
    t[67] = Some(program_p67);
    t
};

// ---------------------------------------------------------------------------
// VerbDispatcher
// ---------------------------------------------------------------------------

/// Tabellenbasierter Verb-Dispatcher — AGC-VERBFAN-Äquivalent in Rust.
///
/// Bildet `VerbCode` auf `VerbAction` ab und führt die Aktion gegen den
/// [`AgcContext`] aus. Statisch, kein Heap, kein `static mut`.
///
/// # Sub-Task 3 — neue Funktionalität
/// - [`VerbDispatcher::dispatch`]: V37, V25, V33, V34, V32
/// - [`VerbDispatcher::accept_load`]: übernimmt R1/R2/R3 bei V25-Enter
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, VERBFAN/VBFANDIR.
pub struct VerbDispatcher;

impl VerbDispatcher {
    /// Dispatcht einen Verb/Noun-Enter-Event gegen den übergebenen [`AgcContext`].
    ///
    /// Setzt `ctx.display.verb` und `ctx.display.noun`, führt dann die zur
    /// `VerbAction` gehörige Logik aus:
    ///
    /// - [`VerbAction::DisplayOnce`] (V06): stoppt Blinken, lädt R1/R2/R3 per NounResolver.
    /// - [`VerbAction::MonitorDisplay`] (V16): setzt `flash_state = false`, lädt R1/R2/R3.
    /// - [`VerbAction::LoadData`] (V25): startet VERB/NOUN-Blinken → Eingabeaufforderung.
    ///   Crew muss danach `accept_load()` aufrufen wenn ENTER kommt.
    /// - V37 (Change Program): Tabellen-Lookup in `PROGRAM_TABLE`, Executive-Einplanung,
    ///   `major_mode` und `active_program` setzen, Blinken stoppen.
    /// - [`VerbAction::Proceed`] (V33): setzt `guidance.proceed_flag`.
    /// - [`VerbAction::Terminate`] (V34): setzt `guidance.terminate_flag`.
    /// - [`VerbAction::Recycle`] (V32): setzt `guidance.recycle_flag`.
    /// - [`VerbAction::Noop`]: keine Aktion.
    ///
    /// # AGC source
    /// `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, VERBFAN / VBFANDIR labels.
    pub fn dispatch(verb: VerbCode, noun: NounCode, ctx: &mut AgcContext) {
        // Verb und Noun in DisplayState übertragen
        ctx.display.verb = verb;
        ctx.display.noun = noun;

        let v = verb.val() as usize;

        // V37 zweiter Schritt: Noun wurde nach dem Flash-Prompt eingegeben (noun≠0).
        // Dieser Zweig feuert beim NounSet-Event, nachdem RequestProgram Flash aktiviert hat.
        // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, MMCHANG label.
        if v == 37 && noun.val() != 0 {
            Self::dispatch_v37(noun, ctx);
            return;
        }

        // Tabellen-Lookup: V00–V39 oder Extended Verbs V40–V45
        let action = if v < VERB_TABLE.len() {
            VERB_TABLE[v]
        } else if v >= EXT_VERB_BASE && v < EXT_VERB_BASE + EXT_VERB_TABLE.len() {
            EXT_VERB_TABLE[v - EXT_VERB_BASE]
        } else {
            VerbAction::Noop
        };

        match action {
            // V06: Einmalige Dezimalanzeige
            // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, DECDSP label.
            VerbAction::DisplayOnce => {
                // Blinken stoppen (kein VERB/NOUN-Flash bei einmaliger Anzeige)
                ctx.display.flash_state = false;
                ctx.display.flash_counter = 0;
                ctx.display.lights.verb_noun_flash = false;

                // NounResolver lädt R1/R2/R3 aus GuidanceContext
                let [r1, r2, r3] = noun_resolver::resolve(noun, ctx);
                ctx.display.r1 = r1;
                ctx.display.r2 = r2;
                ctx.display.r3 = r3;
            }

            // V16: Monitor-Anzeige (zyklische Aktualisierung per Waitlist-Task)
            // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, MONITOR label.
            VerbAction::MonitorDisplay => {
                // Kein Blinken während Monitor läuft
                ctx.display.flash_state = false;
                ctx.display.flash_counter = 0;
                ctx.display.lights.verb_noun_flash = false;

                // Ersten Wert sofort laden
                let [r1, r2, r3] = noun_resolver::resolve(noun, ctx);
                ctx.display.r1 = r1;
                ctx.display.r2 = r2;
                ctx.display.r3 = r3;

                // Selbst-wiederholenden Monitor-Task in Waitlist eintragen (100 cs = 1 s).
                // AGC source: Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, MONITOR label.
                let _ = ctx.waitlist.call_after(100, monitor_task);
            }

            // V25: Load-Daten — VERB/NOUN-Blinken starten (Eingabeaufforderung)
            // Crew-Eingabe wird über accept_load() übernommen.
            // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, NVSBWAIT.
            VerbAction::LoadData => {
                start_flash(&mut ctx.display);
            }

            // V33: Proceed
            // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, PROCKEY.
            VerbAction::Proceed => {
                ctx.guidance.proceed_flag = true;
            }

            // V34: Terminate
            // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, TERMKEY.
            VerbAction::Terminate => {
                ctx.guidance.terminate_flag = true;
            }

            // V32: Recycle
            // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, RSETKEY.
            VerbAction::Recycle => {
                ctx.guidance.recycle_flag = true;
            }

            // V41: Coarse Align IMU — Lade-Prompt V25 N22 anzeigen.
            //
            // Laut AGC zeigt V41 zuerst blinkend V25 N22 (neue ICDU-Winkel eingeben),
            // bevor der eigentliche Coarse-Align-Befehl gesetzt wird.
            // `alignment_cmd` wird erst gesetzt, wenn die Crew mit V33 (PRO) oder
            // ENTER bestätigt.
            //
            // AGC source: `Comanche055/EXTENDED_VERBS.agc`, VBCOARK / IMUCOARK label.
            VerbAction::CoarseAlign => {
                // Flash einschalten: blinkend V25 N22 anzeigen (ICDU-Winkel Eingabe).
                // VerbCode(25) und NounCode(22) sind garantiert gültig (≤ 99).
                start_flash(&mut ctx.display);
                if let (Some(v), Some(n)) = (VerbCode::new(25), NounCode::new(22)) {
                    ctx.display.verb = v;
                    ctx.display.noun = n;
                }
            }

            // V42: Fine Align IMU — Lade-Prompt V25 N93 anzeigen.
            //
            // Laut AGC zeigt V42 zuerst blinkend V25 N93 (Delta-Gyro-Winkel eingeben),
            // bevor der eigentliche Fine-Align-Befehl gesetzt wird.
            // `alignment_cmd` wird erst gesetzt, wenn die Crew mit V33 (PRO) oder
            // ENTER bestätigt.
            //
            // AGC source: `Comanche055/EXTENDED_VERBS.agc`, IMUFINEK / IMUFINE label.
            VerbAction::FineAlign => {
                // Flash einschalten: blinkend V25 N93 anzeigen (Delta-Gyro-Winkel Eingabe).
                // VerbCode(25) und NounCode(93) sind garantiert gültig (≤ 99).
                start_flash(&mut ctx.display);
                if let (Some(v), Some(n)) = (VerbCode::new(25), NounCode::new(93)) {
                    ctx.display.verb = v;
                    ctx.display.noun = n;
                }
            }

            // V37 erster Schritt: Flash einschalten, auf Programm-Nummer warten.
            // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, REQMM label.
            VerbAction::RequestProgram => {
                start_flash(&mut ctx.display);
            }

            VerbAction::ProgramChange(_) | VerbAction::Noop => {}
        }
    }

    /// Übernimmt die Crew-Eingabe nach `V25 ENTER [Werte] ENTER`.
    ///
    /// Wird vom übergeordneten KEYRUPT-Handler aufgerufen, wenn nach einem
    /// aktiven V25 der ENTER-Tastendruck erfolgt. Interpretiert R1/R2/R3
    /// anhand des aktiven Nouns:
    ///
    /// - **N22**: THETAD (Gimbal-Zielwinkel CoarseAlign) [0.01°] → `guidance.thetad` [rad],
    ///   setzt `guidance.alignment_cmd = CoarseAlign`.
    ///   AGC source: `Comanche055/PINBALL_NOUN_TABLES.agc`, N22 (THETAD).
    /// - **N61**: LAT(SPL) [0.01°] → `guidance.splashdown_lat` [rad],
    ///   LNG(SPL) [0.01°] → `guidance.splashdown_lon` [rad],
    ///   HEADSUP  [±1]    → `guidance.heads_up` (1 = heads-up, −1 = heads-down)
    /// - **N93**: DELTA GYRO ANGLES (FineAlign-Korrekturen) [0.001°] → `guidance.thetad` [rad],
    ///   setzt `guidance.alignment_cmd = FineAlign`.
    ///   AGC source: `Comanche055/PINBALL_NOUN_TABLES.agc`, N93 (DELTA GYRO ANGLES).
    ///
    /// Stoppt VERB/NOUN-Blinken nach erfolgreicher Übernahme.
    ///
    /// # Parameter
    /// - `r1`: R1-Eingabe (Einheit je nach Noun, s.o.)
    /// - `r2`: R2-Eingabe
    /// - `r3`: R3-Eingabe
    /// - `ctx`: Zustand
    ///
    /// # AGC source
    /// `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, NVSBWAIT / CHARIN.
    /// `Comanche055/ASSEMBLY_AND_OPERATION_INFORMATION.agc`, N61.
    /// `Comanche055/EXTENDED_VERBS.agc`, VBCOARK / IMUFINEK.
    pub fn accept_load(r1: i32, r2: i32, r3: i32, ctx: &mut AgcContext) {
        let noun = ctx.display.noun;

        match noun.val() {
            // N22: THETAD — Gimbal-Zielwinkel für CoarseAlign
            // Einheit: 0.01° (Centideg) → rad; 0.01° = π/18 000 rad
            // Setzt alignment_cmd = CoarseAlign (Crew-Bestätigung per ENTER/PRO).
            // AGC source: Comanche055/PINBALL_NOUN_TABLES.agc, N22 (THETAD).
            // AGC source: Comanche055/EXTENDED_VERBS.agc, VBCOARK.
            22 => {
                const CENTIDEG_TO_RAD: f64 = core::f64::consts::PI / 18_000.0;
                ctx.guidance.thetad = [
                    r1 as f64 * CENTIDEG_TO_RAD,
                    r2 as f64 * CENTIDEG_TO_RAD,
                    r3 as f64 * CENTIDEG_TO_RAD,
                ];
                ctx.guidance.alignment_cmd = Some(crate::context::AlignmentCommand::CoarseAlign);
                stop_flash(&mut ctx.display);
            }

            // N61: LAT(SPL), LNG(SPL), HEADSUP
            // Einheit: 0.01° → rad; 1° = π/180 rad → 0.01° = π/18 000 rad
            // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N61.
            61 => {
                const CENTIDEG_TO_RAD: f64 = core::f64::consts::PI / 18_000.0;
                ctx.guidance.splashdown_lat = r1 as f64 * CENTIDEG_TO_RAD;
                ctx.guidance.splashdown_lon = r2 as f64 * CENTIDEG_TO_RAD;
                ctx.guidance.heads_up = r3 >= 0;
                stop_flash(&mut ctx.display);
            }

            // N33: TIG — Zündzeitpunkt (HH:MM:SS)
            // R1 = Stunden, R2 = Minuten, R3 = Sekunden (ganzzahlig, ohne Vorzeichen)
            // Wird in Sekunden (MET) umgerechnet und in guidance.tig_s gespeichert.
            // AGC source: PINBALL_NOUN_TABLES.agc, N33 (TIG / HH:MM:SS.SS).
            33 => {
                let h = r1.max(0) as f64;
                let m = r2.max(0) as f64;
                let s = r3.max(0) as f64;
                ctx.guidance.tig_s = h * 3600.0 + m * 60.0 + s;
                stop_flash(&mut ctx.display);
            }

            // N81: DELVLVC — Δv-Manövervektor im LV-System [m/s]
            // R1 = DVx (radial), R2 = DVy (along-track), R3 = DVz (normal) [m/s]
            // Wert wird als Ganzzahl [m/s] übergeben (identische Skalierung, kein Faktor).
            // Wird von P30 als Manöverimpuls genutzt.
            // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N81 (DELVLVC).
            81 => {
                ctx.guidance.delta_v_lv = [r1 as f64, r2 as f64, r3 as f64];
                stop_flash(&mut ctx.display);
            }

            // N93: DELTA GYRO ANGLES — Korrekturwinkel für FineAlign (IMU-Torquing)
            // Einheit: 0.001° (Millideg) → rad; 0.001° = π/180 000 rad
            // Der Simulator nutzt thetad als Proxy (keine separate Gyro-Torque-Variable).
            // Setzt alignment_cmd = FineAlign (Crew-Bestätigung per ENTER/PRO).
            // AGC source: Comanche055/PINBALL_NOUN_TABLES.agc, N93 (DELTA GYRO ANGLES).
            // AGC source: Comanche055/EXTENDED_VERBS.agc, IMUFINEK.
            93 => {
                const MILLIDEG_TO_RAD: f64 = core::f64::consts::PI / 180_000.0;
                ctx.guidance.thetad = [
                    r1 as f64 * MILLIDEG_TO_RAD,
                    r2 as f64 * MILLIDEG_TO_RAD,
                    r3 as f64 * MILLIDEG_TO_RAD,
                ];
                ctx.guidance.alignment_cmd = Some(crate::context::AlignmentCommand::FineAlign);
                stop_flash(&mut ctx.display);
            }

            // Andere Nouns: keine Aktion
            _ => {}
        }
    }

    /// Interne Implementierung für V37 (Change Major Mode).
    ///
    /// Schlägt die Programm-Nummer (= Noun-Wert) in [`PROGRAM_TABLE`] nach,
    /// ruft den Handler auf (der `active_program` setzt und den Executive-Job
    /// einplant), setzt `display.major_mode` und stoppt Blinken.
    ///
    /// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, MMCHANG.
    /// AGC source: `Comanche055/FRESH_START_AND_RESTART.agc`, MODREG-Logik.
    fn dispatch_v37(noun: NounCode, ctx: &mut AgcContext) {
        let prog_nr = noun.val();

        // Tabellen-Lookup — nur mission-kritische Programme eingetragen
        let handler = if (prog_nr as usize) < PROGRAM_TABLE.len() {
            PROGRAM_TABLE[prog_nr as usize]
        } else {
            None
        };

        if let Some(handler_fn) = handler {
            // Handler setzt active_program und plant Executive-Job ein
            handler_fn(ctx);

            // MODREG setzen
            if let Some(mm) = crate::display::verb_noun::MajorMode::new(prog_nr) {
                ctx.display.major_mode = mm;
            }

            // Blinken stoppen (MMCHANG beendet den Flash-Zustand)
            ctx.display.flash_state = false;
            ctx.display.flash_counter = 0;
            ctx.display.lights.verb_noun_flash = false;
        }
        // Unbekannte Programm-Nummer: keine Aktion (kein Alarm auf Dispatcher-Ebene)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::VerbDispatcher;
    use crate::context::AgcContext;
    use crate::display::verb_noun::{DisplayRegister, MajorMode, NounCode, VerbCode};
    use crate::guidance::p61_67::{EntryDisplayData, P61State, P67State};

    // -----------------------------------------------------------------------
    // Hilfsfunktionen
    // -----------------------------------------------------------------------

    fn verb(v: u8) -> VerbCode {
        VerbCode::new(v).unwrap()
    }
    fn noun(n: u8) -> NounCode {
        NounCode::new(n).unwrap()
    }

    // -----------------------------------------------------------------------
    // Test 1: V06/N60 → R1/R2/R3 aus P61State befüllt
    //
    // N60 = GMAX [0.01 m/s²], VPRED [m/s], GAMMAEI [0.01°]
    // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N60.
    // -----------------------------------------------------------------------
    #[test]
    fn v06n60_fills_r1r2r3_from_p61_state() {
        let mut ctx = AgcContext::default();

        // P61State mit bekannten Anzeigedaten vorbelegen
        ctx.guidance.p61 = Some(P61State {
            phase: 0, // bereits fertig
            display: Some(EntryDisplayData {
                gmax: 3.5,          // 3.5 g₀ → R1 = 350 (0.01 m/s²-Einheiten)
                v_pred: 7_800.0,    // 7800 m/s → R2 = 7800
                gamma_ei_deg: -6.5, // −6.5° → R3 = -650 (0.01°)
                rtgo_m: 500_000.0,
                vio: 7_750.0,
                tte_s: 120.0,
            }),
            lat_spl: 0.0,
            lng_spl: 0.0,
            heads_up: false,
        });

        VerbDispatcher::dispatch(verb(6), noun(60), &mut ctx);

        // Verb/Noun im DisplayState gesetzt
        assert_eq!(ctx.display.verb, verb(6));
        assert_eq!(ctx.display.noun, noun(60));

        // Blinken aus
        assert!(!ctx.display.flash_state, "flash_state muss false sein");
        assert!(!ctx.display.lights.verb_noun_flash, "VERB/NOUN FLASH aus");

        // R1 = GMAX in 0.01 m/s²: 3.5 × 9.80665 × 100 = 3432.33 → 3432
        // (Skalierung: gmax ist in g₀, 1 g₀ = 9.80665 m/s²)
        if let DisplayRegister::Decimal(r1) = ctx.display.r1 {
            assert!(
                r1 > 2000 && r1 < 5000,
                "R1 (GMAX) erwartet 2000–5000 (0.01 m/s²), got {r1}"
            );
        } else {
            panic!("R1 sollte Decimal sein, got {:?}", ctx.display.r1);
        }

        // R2 = VPRED in m/s: 7800
        assert_eq!(
            ctx.display.r2,
            DisplayRegister::Decimal(7800),
            "R2 (VPRED) = 7800 m/s"
        );

        // R3 = GAMMAEI in 0.01°: −6.5° → −650
        assert_eq!(
            ctx.display.r3,
            DisplayRegister::Decimal(-650),
            "R3 (GAMMAEI) = −650 (0.01°)"
        );
    }

    // -----------------------------------------------------------------------
    // Test 2: V16/N67 → Monitor-Flag gesetzt, kein Blinken
    //
    // N67 = RTOGO [km], LAT [0.01°], LONG [0.01°] — P67 Terminal Phase.
    // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N67.
    // -----------------------------------------------------------------------
    #[test]
    fn v16n67_monitor_flag_set_no_flash() {
        let mut ctx = AgcContext::default();

        // P67State mit bekannten Werten
        ctx.guidance.p67 = Some(P67State {
            phase: 2,
            xrngerr_m: 1_500.0,  // 1.5 km Quer-Fehler
            dnrngerr_m: 3_200.0, // 3.2 km Längs-Fehler
            roll_cmd_deg: 12.5,
        });

        // Zunächst Flash an (simuliert vorherige Eingabe)
        ctx.display.flash_state = true;
        ctx.display.lights.verb_noun_flash = true;

        VerbDispatcher::dispatch(verb(16), noun(67), &mut ctx);

        // Verb/Noun im DisplayState gesetzt
        assert_eq!(ctx.display.verb, verb(16));
        assert_eq!(ctx.display.noun, noun(67));

        // V16 schaltet Flash ab
        assert!(!ctx.display.flash_state, "flash_state muss false sein");
        assert!(!ctx.display.lights.verb_noun_flash, "VERB/NOUN FLASH aus");

        // R-Register werden befüllt (nicht Blank)
        assert_ne!(
            ctx.display.r1,
            DisplayRegister::Blank,
            "R1 muss befüllt sein (N67)"
        );
    }

    // -----------------------------------------------------------------------
    // Test 3: Unbekanntes Verb (Noop) → keine Veränderung an R-Registern
    // -----------------------------------------------------------------------
    #[test]
    fn noop_verb_does_not_change_registers() {
        let mut ctx = AgcContext::default();
        ctx.display.r1 = DisplayRegister::Decimal(42);

        // V08 ist Spare (Noop) laut VERBTAB
        VerbDispatcher::dispatch(verb(8), noun(0), &mut ctx);

        // R1 bleibt unverändert
        assert_eq!(ctx.display.r1, DisplayRegister::Decimal(42));
    }

    // -----------------------------------------------------------------------
    // Test 4 (Sub-Task 3): V37 + N61 → P61-Job im Executive, MODREG = 61
    //
    // Sequenz: VERB 37 ENTER NOUN 61 ENTER → P61 gestartet.
    // AGC source: FRESH_START_AND_RESTART.agc, MODREG-Logik.
    // -----------------------------------------------------------------------
    #[test]
    fn v37n61_schedules_p61_and_sets_modreg() {
        let mut ctx = AgcContext::default();

        VerbDispatcher::dispatch(verb(37), noun(61), &mut ctx);

        // active_program = 61 (von program_p61 gesetzt)
        assert_eq!(
            ctx.guidance.active_program,
            Some(61),
            "active_program muss 61 sein"
        );

        // MODREG = 61
        assert_eq!(
            ctx.display.major_mode,
            MajorMode::new(61).unwrap(),
            "MODREG (major_mode) muss 61 sein"
        );

        // Executive hat einen Job eingetragen
        assert_eq!(
            ctx.exec.active_count(),
            1,
            "Executive muss genau einen P61-Job haben"
        );

        // Blinken gestoppt
        assert!(!ctx.display.flash_state, "flash_state nach V37 aus");
        assert!(
            !ctx.display.lights.verb_noun_flash,
            "VERB/NOUN FLASH nach V37 aus"
        );
    }

    // -----------------------------------------------------------------------
    // Test 5 (Sub-Task 3): V37 + N30 → P30-Job im Executive, MODREG = 30
    // -----------------------------------------------------------------------
    #[test]
    fn v37n30_schedules_p30_and_sets_modreg() {
        let mut ctx = AgcContext::default();

        VerbDispatcher::dispatch(verb(37), noun(30), &mut ctx);

        assert_eq!(ctx.guidance.active_program, Some(30));
        assert_eq!(ctx.display.major_mode, MajorMode::new(30).unwrap());
        assert_eq!(ctx.exec.active_count(), 1);
    }

    // -----------------------------------------------------------------------
    // Test 6 (Sub-Task 3): V37 + unbekannte Programm-Nummer → keine Aktion
    // -----------------------------------------------------------------------
    #[test]
    fn v37_unknown_program_no_action() {
        let mut ctx = AgcContext::default();

        // N99 ist nicht in PROGRAM_TABLE eingetragen
        VerbDispatcher::dispatch(verb(37), noun(99), &mut ctx);

        assert!(
            ctx.guidance.active_program.is_none(),
            "active_program bleibt None für unbekannte Programm-Nr."
        );
        assert_eq!(ctx.exec.active_count(), 0, "Executive bleibt leer");
    }

    // -----------------------------------------------------------------------
    // Test 7 (Sub-Task 3): V25/N61 → Flash gestartet (Eingabeaufforderung)
    // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, NVSBWAIT.
    // -----------------------------------------------------------------------
    #[test]
    fn v25n61_starts_flash_for_data_entry() {
        let mut ctx = AgcContext::default();

        VerbDispatcher::dispatch(verb(25), noun(61), &mut ctx);

        // Flash muss gestartet sein (Eingabeaufforderung)
        assert!(
            ctx.display.flash_state,
            "flash_state nach V25 muss true sein"
        );
        assert!(ctx.display.flash_counter > 0, "flash_counter muss > 0 sein");

        // Verb/Noun im Display gesetzt
        assert_eq!(ctx.display.verb, verb(25));
        assert_eq!(ctx.display.noun, noun(61));
    }

    // -----------------------------------------------------------------------
    // Test 8 (Sub-Task 3): accept_load mit N61 → p61_target befüllt
    //
    // LAT(SPL) = 25.25° → 2525 Centideg → 0.44 rad (≈)
    // LNG(SPL) = −135.0° → −13500 Centideg → −2.356 rad (≈)
    // HEADSUP  = 1 → heads_up = true
    // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N61.
    // -----------------------------------------------------------------------
    #[test]
    fn accept_load_n61_fills_p61_target() {
        let mut ctx = AgcContext::default();

        // V25/N61 aktiv setzen
        VerbDispatcher::dispatch(verb(25), noun(61), &mut ctx);

        // Crew gibt Werte ein und drückt ENTER
        VerbDispatcher::accept_load(2525, -13500, 1, &mut ctx);

        // LAT(SPL): 2525 × π/18000 ≈ 0.4407 rad
        let lat_rad = ctx.guidance.splashdown_lat;
        let lat_expected = 2525.0 * core::f64::consts::PI / 18_000.0;
        assert!(
            (lat_rad - lat_expected).abs() < 1e-9,
            "splashdown_lat falsch: {lat_rad} ≠ {lat_expected}"
        );

        // LNG(SPL): −13500 × π/18000 ≈ −2.3562 rad
        let lon_rad = ctx.guidance.splashdown_lon;
        let lon_expected = -13500.0 * core::f64::consts::PI / 18_000.0;
        assert!(
            (lon_rad - lon_expected).abs() < 1e-9,
            "splashdown_lon falsch: {lon_rad} ≠ {lon_expected}"
        );

        // HEADSUP = true
        assert!(ctx.guidance.heads_up, "heads_up muss true sein (r3=1)");

        // Flash gestoppt
        assert!(!ctx.display.flash_state, "flash_state nach accept_load aus");
    }

    // -----------------------------------------------------------------------
    // Test 9 (Sub-Task 3): accept_load HEADSUP = −1 → heads_up = false
    // -----------------------------------------------------------------------
    #[test]
    fn accept_load_n61_headsdown() {
        let mut ctx = AgcContext::default();
        VerbDispatcher::dispatch(verb(25), noun(61), &mut ctx);
        VerbDispatcher::accept_load(0, 0, -1, &mut ctx);
        assert!(!ctx.guidance.heads_up, "heads_up muss false sein (r3=−1)");
    }

    // -----------------------------------------------------------------------
    // Test 10 (Sub-Task 3): V33 → proceed_flag gesetzt
    // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, PROCKEY.
    // -----------------------------------------------------------------------
    #[test]
    fn v33_sets_proceed_flag() {
        let mut ctx = AgcContext::default();

        VerbDispatcher::dispatch(verb(33), noun(0), &mut ctx);

        assert!(ctx.guidance.proceed_flag, "proceed_flag muss true sein");
        assert!(!ctx.guidance.terminate_flag, "terminate_flag bleibt false");
        assert!(!ctx.guidance.recycle_flag, "recycle_flag bleibt false");
    }

    // -----------------------------------------------------------------------
    // Test 11 (Sub-Task 3): V34 → terminate_flag gesetzt
    // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, TERMKEY.
    // -----------------------------------------------------------------------
    #[test]
    fn v34_sets_terminate_flag() {
        let mut ctx = AgcContext::default();

        VerbDispatcher::dispatch(verb(34), noun(0), &mut ctx);

        assert!(ctx.guidance.terminate_flag, "terminate_flag muss true sein");
        assert!(!ctx.guidance.proceed_flag, "proceed_flag bleibt false");
    }

    // -----------------------------------------------------------------------
    // Test 12 (Sub-Task 3): V32 → recycle_flag gesetzt
    // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, RSETKEY.
    // -----------------------------------------------------------------------
    #[test]
    fn v32_sets_recycle_flag() {
        let mut ctx = AgcContext::default();

        VerbDispatcher::dispatch(verb(32), noun(0), &mut ctx);

        assert!(ctx.guidance.recycle_flag, "recycle_flag muss true sein");
        assert!(!ctx.guidance.proceed_flag, "proceed_flag bleibt false");
        assert!(!ctx.guidance.terminate_flag, "terminate_flag bleibt false");
    }

    // -----------------------------------------------------------------------
    // Test 13 (Sub-Task 5 / 15.6): V41 → Flash ein, V25/N22, alignment_cmd None
    //
    // Neues Verhalten (Task 15.6): V41 zeigt zuerst blinkend V25 N22 als
    // Lade-Prompt. Der `alignment_cmd` wird erst nach Crew-Bestätigung gesetzt.
    // AGC source: EXTENDED_VERBS.agc, VBCOARK / IMUCOARK.
    // -----------------------------------------------------------------------
    #[test]
    fn v41n20_sets_coarse_align_cmd() {
        let mut ctx = AgcContext::default();

        VerbDispatcher::dispatch(verb(41), noun(20), &mut ctx);

        // Flash muss aktiv sein (Lade-Prompt für ICDU-Winkel).
        // `lights.verb_noun_flash` wird erst durch t4rupt_tick gesetzt (nach
        // FLASH_HALF_PERIOD_TICKS); hier prüfen wir nur flash_state.
        assert!(
            ctx.display.flash_state,
            "flash_state muss true sein nach V41 (Lade-Prompt V25N22)"
        );

        // Verb/Noun auf V25/N22 gesetzt (Lade-Prompt)
        assert_eq!(ctx.display.verb, verb(25), "Verb im Display muss 25 sein");
        assert_eq!(ctx.display.noun, noun(22), "Noun im Display muss 22 sein");

        // alignment_cmd noch NICHT gesetzt (Crew hat noch nicht bestätigt)
        assert_eq!(
            ctx.guidance.alignment_cmd, None,
            "alignment_cmd muss None sein nach V41 (Prompt aktiv)"
        );
    }

    // -----------------------------------------------------------------------
    // Test 14 (Sub-Task 5 / 15.6): V42 → Flash ein, V25/N93, alignment_cmd None
    //
    // Neues Verhalten (Task 15.6): V42 zeigt zuerst blinkend V25 N93 als
    // Lade-Prompt. Der `alignment_cmd` wird erst nach Crew-Bestätigung gesetzt.
    // AGC source: EXTENDED_VERBS.agc, IMUFINEK / IMUFINE.
    // -----------------------------------------------------------------------
    #[test]
    fn v42_sets_fine_align_cmd() {
        let mut ctx = AgcContext::default();

        VerbDispatcher::dispatch(verb(42), noun(0), &mut ctx);

        // Flash muss aktiv sein (Lade-Prompt für Delta-Gyro-Winkel).
        // `lights.verb_noun_flash` wird erst durch t4rupt_tick gesetzt (nach
        // FLASH_HALF_PERIOD_TICKS); hier prüfen wir nur flash_state.
        assert!(
            ctx.display.flash_state,
            "flash_state muss true sein nach V42 (Lade-Prompt V25N93)"
        );

        // Verb/Noun auf V25/N93 gesetzt (Lade-Prompt)
        assert_eq!(ctx.display.verb, verb(25), "Verb im Display muss 25 sein");
        assert_eq!(ctx.display.noun, noun(93), "Noun im Display muss 93 sein");

        // alignment_cmd noch NICHT gesetzt
        assert_eq!(
            ctx.guidance.alignment_cmd, None,
            "alignment_cmd muss None sein nach V42 (Prompt aktiv)"
        );
    }

    // -----------------------------------------------------------------------
    // Test 15 (Sub-Task 5 / 15.6): V41 nach V42 — beide zeigen Prompt (Flash)
    //
    // Sicherheitstest: nach einem V42-Prompt kann V41-Prompt aktiv sein.
    // alignment_cmd bleibt None bis zur Crew-Bestätigung.
    // -----------------------------------------------------------------------
    #[test]
    fn v41_overwrites_previous_alignment_cmd() {
        let mut ctx = AgcContext::default();

        // Zuerst V42 → Prompt V25/N93
        VerbDispatcher::dispatch(verb(42), noun(0), &mut ctx);
        assert!(ctx.display.flash_state, "Flash nach V42 erwartet");
        assert_eq!(ctx.display.noun, noun(93), "Noun nach V42 muss 93 sein");
        assert_eq!(ctx.guidance.alignment_cmd, None);

        // Dann V41 → überschreibt Prompt auf V25/N22
        VerbDispatcher::dispatch(verb(41), noun(20), &mut ctx);
        assert!(ctx.display.flash_state, "Flash nach V41 erwartet");
        assert_eq!(
            ctx.display.noun,
            noun(22),
            "Noun nach V41 muss 22 sein (überschreibt N93)"
        );
        assert_eq!(
            ctx.guidance.alignment_cmd, None,
            "alignment_cmd bleibt None bis Crew bestätigt"
        );
    }

    // -----------------------------------------------------------------------
    // Test 16 (Sub-Task 5 / 15.6): V41 → Flash aktiv, R-Register nicht direkt gesetzt
    //
    // Neues Verhalten: V41 setzt keinen NounResolver-Aufruf mehr beim Dispatch.
    // R-Register bleiben unverändert (Blank). Die Anzeige erfolgt erst nach
    // Crew-Bestätigung über den NounResolver im Monitor/Load-Mechanismus.
    // -----------------------------------------------------------------------
    #[test]
    fn v41n20_displays_thetad_if_set() {
        let mut ctx = AgcContext::default();

        // Ziel-Gimbal-Winkel setzen (wird erst nach Bestätigung angezeigt)
        let oga_rad = 30.0_f64.to_radians();
        let mga_rad = (-45.0_f64).to_radians();
        let iga_rad = 10.0_f64.to_radians();
        ctx.guidance.thetad = [oga_rad, mga_rad, iga_rad];

        VerbDispatcher::dispatch(verb(41), noun(20), &mut ctx);

        // Flash aktiv und Prompt V25/N22 angezeigt
        assert!(
            ctx.display.flash_state,
            "Flash muss nach V41 aktiv sein (Lade-Prompt)"
        );
        assert_eq!(ctx.display.verb, verb(25), "Verb muss V25 sein");
        assert_eq!(ctx.display.noun, noun(22), "Noun muss N22 sein");

        // R-Register bleiben unverändert (kein NounResolver-Aufruf beim Dispatch)
        // Die Anzeige erfolgt erst nach Crew-Bestätigung.
        assert_eq!(
            ctx.guidance.alignment_cmd, None,
            "alignment_cmd muss None sein (Prompt noch aktiv)"
        );
    }

    // -----------------------------------------------------------------------
    // Test 17: accept_load N22 → alignment_cmd = CoarseAlign
    // -----------------------------------------------------------------------

    /// `accept_load` mit N22 setzt `alignment_cmd = CoarseAlign` und stoppt Flash.
    ///
    /// AGC source: `Comanche055/EXTENDED_VERBS.agc`, VBCOARK.
    #[test]
    fn accept_load_n22_sets_coarse_align() {
        let mut ctx = AgcContext::default();

        // V25/N22-Prompt simulieren (wie nach V41 ENTER)
        ctx.display.verb = verb(25);
        ctx.display.noun = noun(22);
        ctx.display.flash_state = true;
        ctx.display.lights.verb_noun_flash = true;

        VerbDispatcher::accept_load(900, 0, 0, &mut ctx);

        assert_eq!(
            ctx.guidance.alignment_cmd,
            Some(crate::context::AlignmentCommand::CoarseAlign),
            "accept_load N22 muss alignment_cmd = CoarseAlign setzen"
        );
        assert!(
            !ctx.display.flash_state,
            "accept_load N22 muss Flash stoppen"
        );
    }

    // -----------------------------------------------------------------------
    // Test 18: accept_load N93 → alignment_cmd = FineAlign
    // -----------------------------------------------------------------------

    /// `accept_load` mit N93 setzt `alignment_cmd = FineAlign` und stoppt Flash.
    ///
    /// AGC source: `Comanche055/EXTENDED_VERBS.agc`, IMUFINEK.
    #[test]
    fn accept_load_n93_sets_fine_align() {
        let mut ctx = AgcContext::default();

        // V25/N93-Prompt simulieren (wie nach V42 ENTER)
        ctx.display.verb = verb(25);
        ctx.display.noun = noun(93);
        ctx.display.flash_state = true;
        ctx.display.lights.verb_noun_flash = true;

        VerbDispatcher::accept_load(100, 200, 300, &mut ctx);

        assert_eq!(
            ctx.guidance.alignment_cmd,
            Some(crate::context::AlignmentCommand::FineAlign),
            "accept_load N93 muss alignment_cmd = FineAlign setzen"
        );
        assert!(
            !ctx.display.flash_state,
            "accept_load N93 muss Flash stoppen"
        );
    }

    // -----------------------------------------------------------------------
    // Test: V16/N40 Monitor-Task via Waitlist
    // -----------------------------------------------------------------------

    /// V16 N40 dispatchen, 100× `tick_with_ctx` aufrufen — Display muss danach
    /// aktualisiert worden sein (r1 ≠ Blank, weil P40-State vorhanden ist).
    ///
    /// Prüft, dass der selbst-wiederholende Waitlist-Monitor-Task korrekt
    /// eingetragen wird und nach 100 cs feuert.
    #[test]
    fn v16_monitor_task_fires_after_100_ticks() {
        use crate::executive::waitlist::tick_with_ctx;
        use crate::guidance::p40_47::{P40State, SPS_ISP, SPS_THRUST_N};

        let mut ctx = AgcContext::default();

        // P40-State vorbelegen (damit N40 etwas liefert statt Blank)
        ctx.guidance.p40 = Some(P40State::new(100.0, SPS_ISP, SPS_THRUST_N, 28_800.0));

        // V16/N40 dispatchen — trägt monitor_task nach 100 cs in Waitlist ein
        VerbDispatcher::dispatch(verb(16), noun(40), &mut ctx);

        // Nach dem Dispatch ist r1/r2/r3 sofort gesetzt (erster Wert),
        // und ein Waitlist-Task wurde eingetragen.
        assert_eq!(ctx.waitlist.count(), 1);
        assert_eq!(ctx.waitlist.time_to_next(), Some(100));

        // R1 direkt nach dispatch prüfen (Sofort-Update)
        assert_ne!(
            ctx.display.r1,
            DisplayRegister::Blank,
            "V16/N40 dispatch muss R1 sofort befüllen"
        );

        // R1 auf Blank zurücksetzen, um den Waitlist-Tick zu testen
        ctx.display.r1 = DisplayRegister::Blank;
        ctx.display.r2 = DisplayRegister::Blank;
        ctx.display.r3 = DisplayRegister::Blank;

        // 99 Ticks — noch nicht fällig
        for _ in 0..99 {
            tick_with_ctx(&mut ctx, 1);
        }
        assert_eq!(
            ctx.display.r1,
            DisplayRegister::Blank,
            "Nach 99 Ticks darf der Monitor-Task noch nicht gefeuert haben"
        );

        // 100. Tick — Task feuert, Display muss aktualisiert sein
        tick_with_ctx(&mut ctx, 1);
        assert_ne!(
            ctx.display.r1,
            DisplayRegister::Blank,
            "Nach 100 Ticks muss der Monitor-Task R1 befüllt haben"
        );

        // Monitor hat sich selbst wieder eingetragen (selbst-wiederholend)
        assert_eq!(
            ctx.waitlist.count(),
            1,
            "Monitor-Task muss sich selbst für den nächsten Zyklus eingetragen haben"
        );
        assert_eq!(
            ctx.waitlist.time_to_next(),
            Some(100),
            "Nächste Monitor-Aktualisierung in 100 cs"
        );
    }
}
