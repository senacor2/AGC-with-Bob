//! Verb-Dispatcher — tabellenbasierter AGC-VERBFAN-Kern.
//!
//! Übersetzt den von der Crew eingegebenen Verb-Code in eine Aktion.
//! Entspricht dem `VERBFAN`-Mechanismus des AGC:
//! `VBFANDIR INDEX VERBREG / CAF VERBTAB`.
//!
//! # Umfang (Sub-Task 2)
//! V06 (Decimal Display, einmalig) und V16 (Monitor Display, zyklisch)
//! für die Reentry-relevanten Nouns N60–N67, N74.
//! Weitere Verben (V37, V33, V34, V25) folgen in Sub-Task 3.
//!
//! # Hinweis zum NounResolver
//! `dispatch()` delegiert die Datenbeschaffung an [`super::noun_resolver`]
//! (Sub-Task 4). Solange dieser noch nicht vollständig implementiert ist,
//! liefert der Resolver für unbekannte Nouns `[DisplayRegister::Blank; 3]`.
//!
//! AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, VERBFAN/VERBTAB.

use crate::context::AgcContext;
use crate::display::noun_resolver;
use crate::display::verb_noun::{NounCode, VerbCode};

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
/// Index entspricht dem Verb-Code. Einträge außerhalb des implementierten
/// Bereichs (Sub-Task 2: nur V06 und V16) werden als `Noop` eingetragen.
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
    // VB37: Change Major Mode
    // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, MMCHANG label.
    // Programm-Nummer wird zur Dispatch-Zeit aus dem Noun gelesen → Noop als Platzhalter;
    // dispatch() behandelt V37 mit gesonderter Logik.
    // (V37 wird in Sub-Task 3 vollständig implementiert.)
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
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, VERBFAN/VBFANDIR.
pub struct VerbDispatcher;

impl VerbDispatcher {
    /// Dispatcht einen Verb/Noun-Enter-Event gegen den übergebenen [`AgcContext`].
    ///
    /// Setzt `ctx.display.verb` und `ctx.display.noun`, führt dann die zur
    /// `VerbAction` gehörige Logik aus:
    ///
    /// - [`VerbAction::DisplayOnce`] (V06): stoppt Blinken, lädt R1/R2/R3 per NounResolver.
    /// - [`VerbAction::MonitorDisplay`] (V16): setzt `flash_state = false` (kein Blinken
    ///   während Monitor), lädt R1/R2/R3 per NounResolver.
    /// - Alle anderen Verben: keine Aktion (Noop, noch nicht implementiert).
    ///
    /// # AGC source
    /// `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, VERBFAN / VBFANDIR labels.
    pub fn dispatch(verb: VerbCode, noun: NounCode, ctx: &mut AgcContext) {
        // Verb und Noun in DisplayState übertragen
        ctx.display.verb = verb;
        ctx.display.noun = noun;

        // Tabellen-Lookup: Verb-Codes ≥ 40 → Noop (Extended Verbs folgen in Sub-Task 5)
        let action = if (verb.val() as usize) < VERB_TABLE.len() {
            VERB_TABLE[verb.val() as usize]
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

            // Alle anderen Verben noch nicht implementiert
            VerbAction::ProgramChange(_) | VerbAction::Noop => {}
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::VerbDispatcher;
    use crate::context::AgcContext;
    use crate::display::verb_noun::{DisplayRegister, NounCode, VerbCode};
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
}
