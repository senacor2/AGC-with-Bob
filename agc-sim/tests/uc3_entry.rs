//! UC-3 Integrationstest: Entry (P61 → P64 → P67)
//!
//! Verifiziert den Wiedereintritts-Ablauf:
//! 1. Szenario `entry_pre.scenario` laden (EI 122 km)
//! 2. `V37E61E` → PROG=61
//! 3. `V25 N61 ENTR` → Splashdown-Koordinaten eingeben → gesetzt
//! 4. `V06 N61 ENTR` → zeigt Splashdown-Koordinaten (nicht Blank)
//! 5. `V37E64E` → PROG=64
//! 6. `V37E67E` → PROG=67
//! 7. `V16 N66 ENTR` → Monitor-Modus aktiv
//!
//! DSKY-Notation: `V37E61E` = VERB 3 7 ENTER 6 1 ENTER
//! Quelle: "The Apollo Guidance Computer", Kap. 4 „Command Module entry".

#[path = "test_utils.rs"]
mod test_utils;

use agc_core::display::verb_noun::{
    DisplayRegister, KEY_0, KEY_1, KEY_3, KEY_4, KEY_6, KEY_7, KEY_ENTER, KEY_NOUN, KEY_VERB,
};

const SCENARIO: &str = "docs/scenarios/entry_pre.scenario";

// ---------------------------------------------------------------------------
// UC-3-A: P61 starten — PROG=61
// ---------------------------------------------------------------------------

#[test]
fn uc3_p61_start() {
    let mut state = test_utils::load_scenario_state(SCENARIO);

    test_utils::press_keys(
        &mut state,
        &[
            KEY_VERB, KEY_3, KEY_7, KEY_ENTER, // V37E
            KEY_6, KEY_1, KEY_ENTER, // 61E
        ],
    );

    assert_eq!(
        state.ctx.display.major_mode.val(),
        61,
        "major_mode nach V37E61E erwartet 61"
    );
    assert_eq!(
        state.ctx.guidance.active_program,
        Some(61),
        "active_program nach V37E61E erwartet Some(61)"
    );
}

// ---------------------------------------------------------------------------
// UC-3-B: Splashdown-Koordinaten via V25 N61 setzen
// ---------------------------------------------------------------------------

#[test]
fn uc3_splashdown_coords_set() {
    use agc_core::display::dispatcher::VerbDispatcher;
    use agc_core::display::verb_noun::NounCode;

    let mut state = test_utils::load_scenario_state(SCENARIO);

    // P61 starten
    test_utils::press_keys(
        &mut state,
        &[KEY_VERB, KEY_3, KEY_7, KEY_ENTER, KEY_6, KEY_1, KEY_ENTER],
    );

    // Direkt accept_load für N61 aufrufen (Crew-Simulation V25 N61)
    // lat = 1354 centideg = 13.54°, lon = -17234 centideg = -172.34°, heads_up = 1
    #[expect(
        clippy::unwrap_used,
        reason = "61 ist garantiert gültig als NounCode in Tests"
    )]
    let noun61 = NounCode::new(61).unwrap();
    state.ctx.display.noun = noun61;
    VerbDispatcher::accept_load(1354, -17234, 1, &mut state.ctx);

    const CENTIDEG_TO_RAD: f64 = core::f64::consts::PI / 18_000.0;
    let expected_lat = 1354.0 * CENTIDEG_TO_RAD;
    let expected_lon = -17234.0 * CENTIDEG_TO_RAD;

    assert!(
        (state.ctx.guidance.splashdown_lat - expected_lat).abs() < 1e-9,
        "splashdown_lat stimmt nicht: erwartet {:.6}, got {:.6}",
        expected_lat,
        state.ctx.guidance.splashdown_lat
    );
    assert!(
        (state.ctx.guidance.splashdown_lon - expected_lon).abs() < 1e-9,
        "splashdown_lon stimmt nicht"
    );
    assert!(
        state.ctx.guidance.heads_up,
        "heads_up muss bei r3=1 gesetzt sein"
    );
}

// ---------------------------------------------------------------------------
// UC-3-C: V06 N61 zeigt Splashdown-Koordinaten (nicht Blank)
// ---------------------------------------------------------------------------

#[test]
fn uc3_n61_display_not_blank_after_set() {
    use agc_core::display::dispatcher::VerbDispatcher;
    use agc_core::display::verb_noun::NounCode;

    let mut state = test_utils::load_scenario_state(SCENARIO);

    // P61 + Splashdown setzen
    test_utils::press_keys(
        &mut state,
        &[KEY_VERB, KEY_3, KEY_7, KEY_ENTER, KEY_6, KEY_1, KEY_ENTER],
    );
    #[expect(
        clippy::unwrap_used,
        reason = "61 ist garantiert gültig als NounCode in Tests"
    )]
    let noun61 = NounCode::new(61).unwrap();
    state.ctx.display.noun = noun61;
    VerbDispatcher::accept_load(1354, -17234, 1, &mut state.ctx);

    // V06 N61 ENTER — einmalige Anzeige
    test_utils::press_keys(
        &mut state,
        &[
            KEY_VERB, KEY_0, KEY_6, KEY_ENTER, KEY_NOUN, KEY_6, KEY_1, KEY_ENTER,
        ],
    );

    let r1_blank = matches!(state.ctx.display.r1, DisplayRegister::Blank);
    assert!(
        !r1_blank,
        "R1 (LAT/SPL) darf nach V06 N61 nicht Blank sein wenn Splashdown gesetzt"
    );
}

// ---------------------------------------------------------------------------
// UC-3-D: P64 starten — PROG=64
// ---------------------------------------------------------------------------

#[test]
fn uc3_p64_start() {
    let mut state = test_utils::load_scenario_state(SCENARIO);

    test_utils::press_keys(
        &mut state,
        &[
            KEY_VERB, KEY_3, KEY_7, KEY_ENTER, // V37E
            KEY_6, KEY_4, KEY_ENTER, // 64E
        ],
    );

    assert_eq!(
        state.ctx.display.major_mode.val(),
        64,
        "major_mode nach V37E64E erwartet 64"
    );
    assert_eq!(
        state.ctx.guidance.active_program,
        Some(64),
        "active_program nach V37E64E erwartet Some(64)"
    );
}

// ---------------------------------------------------------------------------
// UC-3-E: P67 starten — PROG=67
// ---------------------------------------------------------------------------

#[test]
fn uc3_p67_start() {
    let mut state = test_utils::load_scenario_state(SCENARIO);

    test_utils::press_keys(
        &mut state,
        &[
            KEY_VERB, KEY_3, KEY_7, KEY_ENTER, // V37E
            KEY_6, KEY_7, KEY_ENTER, // 67E
        ],
    );

    assert_eq!(
        state.ctx.display.major_mode.val(),
        67,
        "major_mode nach V37E67E erwartet 67"
    );
    assert_eq!(
        state.ctx.guidance.active_program,
        Some(67),
        "active_program nach V37E67E erwartet Some(67)"
    );
}

// ---------------------------------------------------------------------------
// UC-3-F: V16 N66 — Monitor-Modus für Guidance-Fehler
// ---------------------------------------------------------------------------

#[test]
fn uc3_v16_n66_monitor_mode() {
    let mut state = test_utils::load_scenario_state(SCENARIO);

    // P67 starten (Guidance-Monitor-Modus)
    test_utils::press_keys(
        &mut state,
        &[KEY_VERB, KEY_3, KEY_7, KEY_ENTER, KEY_6, KEY_7, KEY_ENTER],
    );

    // V16 N66 ENTER
    test_utils::press_keys(
        &mut state,
        &[
            KEY_VERB, KEY_1, KEY_6, KEY_ENTER, KEY_NOUN, KEY_6, KEY_6, KEY_ENTER,
        ],
    );

    assert_eq!(
        state.ctx.display.verb.val(),
        16,
        "Verb muss nach V16 N66 = 16 sein"
    );
    assert_eq!(
        state.ctx.display.noun.val(),
        66,
        "Noun muss nach V16 N66 = 66 sein"
    );
}
