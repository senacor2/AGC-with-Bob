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
//!
//! # Hinweis zum NounResolver
//! `dispatch()` delegiert die Datenbeschaffung an [`super::noun_resolver`]
//! (Sub-Task 4). Solange dieser noch nicht vollständig implementiert ist,
//! liefert der Resolver für unbekannte Nouns `[DisplayRegister::Blank; 3]`.
//!
//! AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, VERBFAN/VERBTAB.

use crate::context::AgcContext;
use crate::display::noun_resolver;
use crate::display::verb_noun::{start_flash, NounCode, VerbCode};

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

    /// Verb ist nicht implementiert oder nicht relevant (Spare / Illegal).
    ///
    /// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, GODSPALM label.
    Noop,
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
    // VB37: Change Major Mode — gesonderte Logik in dispatch()
    // (Programm-Nummer wird zur Dispatch-Zeit aus dem Noun gelesen)
    // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, MMCHANG label.
    t[37] = VerbAction::Noop; // Platzhalter; dispatch() erkennt V37 gesondert
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
/// Trägt P30 in den Executive ein. Der tatsächliche P30-Step wird im
/// job-callback über den `AgcContext` via globalen Mutex aufgerufen
/// (vollständige Executive-Integration in Sub-Task 6).
///
/// AGC source: `Comanche055/P30,P37.agc`, P30CALL.
fn program_p30(ctx: &mut AgcContext) {
    ctx.guidance.active_program = Some(30);
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
/// AGC source: `Comanche055/P40-P47.agc`, P40CALL.
fn program_p40(ctx: &mut AgcContext) {
    ctx.guidance.active_program = Some(40);
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

        // V37: Change Program — gesonderte Logik (Noun = Programm-Nummer)
        // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, MMCHANG label.
        if v == 37 {
            Self::dispatch_v37(noun, ctx);
            return;
        }

        // Tabellen-Lookup: Verb-Codes ≥ 40 → Noop (Extended Verbs folgen in Sub-Task 5)
        let action = if v < VERB_TABLE.len() {
            VERB_TABLE[v]
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

            // V16: Monitor-Anzeige (zyklische Aktualisierung per T4RUPT)
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

            VerbAction::ProgramChange(_) | VerbAction::Noop => {}
        }
    }

    /// Übernimmt die Crew-Eingabe nach `V25 ENTER [Werte] ENTER`.
    ///
    /// Wird vom übergeordneten KEYRUPT-Handler aufgerufen, wenn nach einem
    /// aktiven V25 der ENTER-Tastendruck erfolgt. Interpretiert R1/R2/R3
    /// anhand des aktiven Nouns:
    ///
    /// - **N61**: LAT(SPL) [0.01°] → `guidance.splashdown_lat` [rad],
    ///   LNG(SPL) [0.01°] → `guidance.splashdown_lon` [rad],
    ///   HEADSUP  [±1]    → `guidance.heads_up` (1 = heads-up, −1 = heads-down)
    ///
    /// Für andere Nouns wird keine Aktion ausgeführt (unvollständig — Sub-Task 5+).
    ///
    /// Stoppt VERB/NOUN-Blinken nach erfolgreicher Übernahme.
    ///
    /// # Parameter
    /// - `r1_centideg`: R1-Eingabe in 0.01° (z. B. 2525 für 25.25°)
    /// - `r2_centideg`: R2-Eingabe in 0.01°
    /// - `r3_headsup`:  R3-Eingabe: 1 = heads-up, −1 = heads-down (für N61)
    /// - `ctx`: Zustand
    ///
    /// # AGC source
    /// `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, NVSBWAIT / CHARIN.
    /// `Comanche055/ASSEMBLY_AND_OPERATION_INFORMATION.agc`, N61.
    pub fn accept_load(r1_centideg: i32, r2_centideg: i32, r3_headsup: i32, ctx: &mut AgcContext) {
        let noun = ctx.display.noun;

        // N61: LAT(SPL), LNG(SPL), HEADSUP
        // Einheit: 0.01° → rad; 1° = π/180 rad → 0.01° = π/18000 rad
        // AGC source: ASSEMBLY_AND_OPERATION_INFORMATION.agc, N61.
        // Andere Nouns: keine Aktion (Sub-Task 5+ ergänzt weitere)
        if noun.val() == 61 {
            const CENTIDEG_TO_RAD: f64 = core::f64::consts::PI / 18_000.0;
            ctx.guidance.splashdown_lat = r1_centideg as f64 * CENTIDEG_TO_RAD;
            ctx.guidance.splashdown_lon = r2_centideg as f64 * CENTIDEG_TO_RAD;
            ctx.guidance.heads_up = r3_headsup >= 0;

            // Blinken stoppen — Eingabe abgeschlossen
            ctx.display.flash_state = false;
            ctx.display.flash_counter = 0;
            ctx.display.lights.verb_noun_flash = false;
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
}
