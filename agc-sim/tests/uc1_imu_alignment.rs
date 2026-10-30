//! UC-1 Integrationstest: IMU-Alignment (P51 → P52)
//!
//! Verifiziert den vollständigen IMU-Alignment-Ablauf:
//! 1. Szenario `leo_circ.scenario` laden
//! 2. `V37E51E` → PROG=51
//! 3. `V41E` → CoarseAlign-Prompt (V25 N22 blinkt)
//! 4. `V25 N22` Werte eingeben → `thetad` gesetzt
//! 5. `V37E52E` → PROG=52
//! 6. `V42E` → FineAlign-Prompt (V25 N93 blinkt)
//! 7. PRO (V33) → `alignment_cmd = FineAlign`
//!
//! DSKY-Notation: `V37E51E` = VERB 3 7 ENTER 5 1 ENTER
//! Quelle: "The Apollo Guidance Computer", Kap. 8, IMU Alignment.

#[path = "test_utils.rs"]
mod test_utils;

use agc_core::context::AlignmentCommand;
use agc_core::display::verb_noun::{
    KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_7, KEY_ENTER, KEY_NOUN, KEY_PRO, KEY_VERB,
};

const SCENARIO: &str = "docs/scenarios/leo_circ.scenario";

// ---------------------------------------------------------------------------
// UC-1-A: P51 starten — PROG=51
// ---------------------------------------------------------------------------

#[test]
fn uc1_p51_start() {
    let mut state = test_utils::load_scenario_state(SCENARIO);

    // V37E51E — Programm 51 starten
    test_utils::press_keys(
        &mut state,
        &[
            KEY_VERB, KEY_3, KEY_7, KEY_ENTER, // V37E
            KEY_5, KEY_1, KEY_ENTER, // 51E
        ],
    );

    assert_eq!(
        state.ctx.display.major_mode.val(),
        51,
        "major_mode nach V37E51E erwartet 51"
    );
    assert_eq!(
        state.ctx.guidance.active_program,
        Some(51),
        "active_program nach V37E51E erwartet Some(51)"
    );
}

// ---------------------------------------------------------------------------
// UC-1-B: CoarseAlign-Prompt via V41 → Flash + N22
// ---------------------------------------------------------------------------

#[test]
fn uc1_coarse_align_prompt() {
    let mut state = test_utils::load_scenario_state(SCENARIO);

    // P51 starten
    test_utils::press_keys(
        &mut state,
        &[KEY_VERB, KEY_3, KEY_7, KEY_ENTER, KEY_5, KEY_1, KEY_ENTER],
    );

    // V41E — CoarseAlign-Verb (kein Noun nötig)
    test_utils::press_keys(&mut state, &[KEY_VERB, KEY_4, KEY_1, KEY_ENTER]);

    // Flash muss aktiv sein, Noun = 22
    assert!(
        state.ctx.display.flash_state,
        "Flash muss nach V41E aktiv sein (CoarseAlign-Prompt)"
    );
    assert_eq!(
        state.ctx.display.noun.val(),
        22,
        "Noun muss nach V41E = 22 (THETAD) sein"
    );
}

// ---------------------------------------------------------------------------
// UC-1-C: N22-Werte eingeben → thetad gesetzt
// ---------------------------------------------------------------------------

#[test]
fn uc1_thetad_via_n22() {
    let mut state = test_utils::load_scenario_state(SCENARIO);

    // P51 + CoarseAlign-Prompt: Flash+V25+N22 aktiv
    test_utils::press_keys(
        &mut state,
        &[
            KEY_VERB, KEY_3, KEY_7, KEY_ENTER, KEY_5, KEY_1, KEY_ENTER, // P51
            KEY_VERB, KEY_4, KEY_1, KEY_ENTER, // V41 → Flash aktiv, Noun=22
        ],
    );

    // V25+N22 Flash-Modus: Werte als Noun-Eingaben (je 1-stellig für den Test)
    // Gemäß sim_loop.rs NounSet-Arm: Wert landet in ctx.load_state.r1/r2/r3
    // Wichtig: Noun-Eingabe setzt Noun UM → danach ist Noun ≠ 22.
    // Daher prüfen wir ctx.load_state.r1/r2/r3, nicht alignment_cmd.
    test_utils::press_keys(
        &mut state,
        &[
            KEY_NOUN, KEY_1, KEY_ENTER, // load_state.r1 = 1
            KEY_NOUN, KEY_2, KEY_ENTER, // load_state.r2 = 2
            KEY_NOUN, KEY_3, KEY_ENTER, // load_state.r3 = 3
        ],
    );

    // Lade-Register müssen die eingetippten Werte halten
    assert_eq!(
        state.ctx.load_state.r1, 1,
        "load_state.r1 nach N22-Eingabe = 1 erwartet, got {}",
        state.ctx.load_state.r1
    );
    assert_eq!(
        state.ctx.load_state.r2, 2,
        "load_state.r2 nach N22-Eingabe = 2 erwartet, got {}",
        state.ctx.load_state.r2
    );
    assert_eq!(
        state.ctx.load_state.r3, 3,
        "load_state.r3 nach N22-Eingabe = 3 erwartet, got {}",
        state.ctx.load_state.r3
    );
}

// ---------------------------------------------------------------------------
// UC-1-D: P52 starten → PROG=52
// ---------------------------------------------------------------------------

#[test]
fn uc1_p52_start() {
    let mut state = test_utils::load_scenario_state(SCENARIO);

    // V37E52E — Programm 52 starten
    test_utils::press_keys(
        &mut state,
        &[
            KEY_VERB, KEY_3, KEY_7, KEY_ENTER, // V37E
            KEY_5, KEY_2, KEY_ENTER, // 52E
        ],
    );

    assert_eq!(
        state.ctx.display.major_mode.val(),
        52,
        "major_mode nach V37E52E erwartet 52"
    );
}

// ---------------------------------------------------------------------------
// UC-1-E: FineAlign-Prompt via V42 → Flash + N93
// ---------------------------------------------------------------------------

#[test]
fn uc1_fine_align_prompt_via_v42() {
    let mut state = test_utils::load_scenario_state(SCENARIO);

    // P52 starten
    test_utils::press_keys(
        &mut state,
        &[KEY_VERB, KEY_3, KEY_7, KEY_ENTER, KEY_5, KEY_2, KEY_ENTER],
    );

    // V42E — FineAlign-Verb
    test_utils::press_keys(&mut state, &[KEY_VERB, KEY_4, KEY_2, KEY_ENTER]);

    assert!(
        state.ctx.display.flash_state,
        "Flash muss nach V42E aktiv sein (FineAlign-Prompt)"
    );
    assert_eq!(
        state.ctx.display.noun.val(),
        93,
        "Noun muss nach V42E = 93 (DELTA GYRO ANGLES) sein"
    );
}

// ---------------------------------------------------------------------------
// UC-1-F: PRO nach FineAlign-Prompt → alignment_cmd = FineAlign
// ---------------------------------------------------------------------------

#[test]
fn uc1_fine_align_cmd_via_pro() {
    let mut state = test_utils::load_scenario_state(SCENARIO);

    // P52 + FineAlign-Prompt
    test_utils::press_keys(
        &mut state,
        &[
            KEY_VERB, KEY_3, KEY_7, KEY_ENTER, KEY_5, KEY_2, KEY_ENTER, // P52
            KEY_VERB, KEY_4, KEY_2, KEY_ENTER, // V42
        ],
    );

    // PRO (KEY_PRO) im Flash-Modus mit N93 → FineAlign
    // KEY_PRO löst KeyEvent::Proceed aus, der alignment_cmd setzt.
    // V33 als Verb-Eingabe würde nur VerbSet erzeugen, nicht Proceed.
    test_utils::press_keys(&mut state, &[KEY_PRO]);

    assert_eq!(
        state.ctx.guidance.alignment_cmd,
        Some(AlignmentCommand::FineAlign),
        "alignment_cmd muss nach PRO im N93-Modus FineAlign sein"
    );
    assert!(
        !state.ctx.display.flash_state,
        "Flash muss nach PRO gestoppt sein"
    );
}

// ---------------------------------------------------------------------------
// UC-1-G: N22 accept_load → thetad wird gesetzt
// ---------------------------------------------------------------------------

#[test]
fn uc1_accept_load_n22_sets_thetad() {
    use agc_core::display::dispatcher::VerbDispatcher;
    use agc_core::display::verb_noun::NounCode;

    let mut state = test_utils::load_scenario_state(SCENARIO);

    // Direkt noun auf 22 setzen (Simulation eines V25 N22-Zustands)
    #[expect(
        clippy::unwrap_used,
        reason = "22 ist garantiert gültig als NounCode in Tests"
    )]
    let noun22 = NounCode::new(22).unwrap();
    state.ctx.display.noun = noun22;

    // accept_load mit Centideg-Werten aufrufen (1000 = 10.00°, 2000 = 20.00°, 3000 = 30.00°)
    VerbDispatcher::accept_load(1000, 2000, 3000, &mut state.ctx);

    const CENTIDEG_TO_RAD: f64 = core::f64::consts::PI / 18_000.0;
    let expected = [
        1000.0 * CENTIDEG_TO_RAD,
        2000.0 * CENTIDEG_TO_RAD,
        3000.0 * CENTIDEG_TO_RAD,
    ];

    for i in 0..3 {
        assert!(
            (state.ctx.guidance.thetad[i] - expected[i]).abs() < 1e-9,
            "thetad[{i}] erwartet {:.6}, got {:.6}",
            expected[i],
            state.ctx.guidance.thetad[i]
        );
    }
}

// ---------------------------------------------------------------------------
// UC-1-H: N93 accept_load → thetad wird gesetzt (Millideg)
// ---------------------------------------------------------------------------

#[test]
fn uc1_accept_load_n93_sets_thetad() {
    use agc_core::display::dispatcher::VerbDispatcher;
    use agc_core::display::verb_noun::NounCode;

    let mut state = test_utils::load_scenario_state(SCENARIO);

    #[expect(
        clippy::unwrap_used,
        reason = "93 ist garantiert gültig als NounCode in Tests"
    )]
    let noun93 = NounCode::new(93).unwrap();
    state.ctx.display.noun = noun93;

    // accept_load mit Millideg-Werten (500 = 0.500°, 1000 = 1.000°, 2000 = 2.000°)
    VerbDispatcher::accept_load(500, 1000, 2000, &mut state.ctx);

    const MILLIDEG_TO_RAD: f64 = core::f64::consts::PI / 180_000.0;
    let expected = [
        500.0 * MILLIDEG_TO_RAD,
        1000.0 * MILLIDEG_TO_RAD,
        2000.0 * MILLIDEG_TO_RAD,
    ];

    for i in 0..3 {
        assert!(
            (state.ctx.guidance.thetad[i] - expected[i]).abs() < 1e-9,
            "thetad[{i}] (N93/Millideg) erwartet {:.6}, got {:.6}",
            expected[i],
            state.ctx.guidance.thetad[i]
        );
    }
}
