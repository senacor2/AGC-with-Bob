//! Integrationstests: VerbDispatcher + NounResolver + SimState (Sub-Task 6).
//!
//! Prüft die vollständige Verdrahtung:
//! Tastaturfolge → process_key() → VerbDispatcher::dispatch() → GuidanceContext → Display.
//!
//! # P61-Startsequenz
//! Crew-Sequenz zum Starten von P61 (Entry Preparation):
//! `VERB 3 7 ENTER NOUN 6 1 ENTER`
//! → PROG = 61, active_program = 61, display.verb = 37, display.noun = 61.
//!
//! # TEI/LOI-Sequenz (P30 → P40)
//! `VERB 3 7 ENTER NOUN 3 0 ENTER` → P30 aktiv
//! `VERB 3 7 ENTER NOUN 4 0 ENTER` → P40 aktiv
//!
//! # Monitor-Modus (V16)
//! Nach `VERB 1 6 ENTER NOUN 6 7 ENTER` wird V16N67 aktiv:
//! R1/R2/R3 werden pro Tick aktualisiert (NounResolver → GuidanceContext).
//!
//! # V33/V34 (Proceed/Terminate)
//! `VERB 3 3 ENTER` → proceed_flag gesetzt
//! `VERB 3 4 ENTER` → terminate_flag gesetzt

use agc_core::display::verb_noun::{
    KEY_0, KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_ENTER, KEY_NOUN, KEY_VERB,
};
use agc_sim::sim_loop::{sim_tick, SimState};

// ---------------------------------------------------------------------------
// Hilfsfunktion: Tastenfolge einspielen
// ---------------------------------------------------------------------------

/// Schiebt eine Folge von AGC-Rohkeycodes in den SimDsky-FIFO und
/// ruft pro Taste einen `sim_tick()` auf.
fn press_keys(state: &mut SimState, keys: &[u8]) {
    for &k in keys {
        state.dsky.push_key(k);
        sim_tick(state);
    }
}

// ---------------------------------------------------------------------------
// Test 1: P61-Startsequenz — PROG=61 nach V37 N61 ENTER
// ---------------------------------------------------------------------------

/// Crew drückt `VERB 3 7 ENTER NOUN 6 1 ENTER`.
/// Erwartet: display.major_mode = 61, active_program = 61, verb = 37, noun = 61.
#[test]
fn p61_start_sequence_sets_program_61() {
    let mut state = SimState::new();

    // VERB 3 7 ENTER — setzt Verb = 37
    press_keys(&mut state, &[KEY_VERB, KEY_3, KEY_7, KEY_ENTER]);

    // NOUN 6 1 ENTER — setzt Noun = 61, löst Dispatcher V37 aus
    press_keys(&mut state, &[KEY_NOUN, KEY_6, KEY_1, KEY_ENTER]);

    // PROG-Anzeige (major_mode) muss 61 sein
    assert_eq!(
        state.ctx.display.major_mode.val(),
        61,
        "major_mode nach V37N61 erwartet 61, got {}",
        state.ctx.display.major_mode.val()
    );

    // active_program im GuidanceContext muss gesetzt sein
    assert_eq!(
        state.ctx.guidance.active_program,
        Some(61),
        "active_program nach V37N61 erwartet Some(61), got {:?}",
        state.ctx.guidance.active_program
    );

    // Verb und Noun im Display
    assert_eq!(state.ctx.display.verb.val(), 37);
    assert_eq!(state.ctx.display.noun.val(), 61);

    // Flash muss nach Programm-Wechsel gestoppt sein
    assert!(
        !state.ctx.display.flash_state,
        "Flash läuft nach V37N61 unerwartet"
    );
}

// ---------------------------------------------------------------------------
// Test 2: P30-Startsequenz — PROG=30 nach V37 N30 ENTER
// ---------------------------------------------------------------------------

/// Crew drückt `VERB 3 7 ENTER NOUN 3 0 ENTER`.
/// Erwartet: PROG = 30, active_program = 30.
#[test]
fn p30_start_sequence_sets_program_30() {
    let mut state = SimState::new();

    press_keys(&mut state, &[KEY_VERB, KEY_3, KEY_7, KEY_ENTER]);
    press_keys(&mut state, &[KEY_NOUN, KEY_3, KEY_0, KEY_ENTER]);

    assert_eq!(
        state.ctx.display.major_mode.val(),
        30,
        "major_mode nach V37N30 erwartet 30"
    );
    assert_eq!(state.ctx.guidance.active_program, Some(30));
}

// ---------------------------------------------------------------------------
// Test 3: P40-Startsequenz — PROG=40 nach V37 N40 ENTER
// ---------------------------------------------------------------------------

/// Crew drückt `VERB 3 7 ENTER NOUN 4 0 ENTER`.
/// Erwartet: PROG = 40, active_program = 40.
#[test]
fn p40_start_sequence_sets_program_40() {
    let mut state = SimState::new();

    press_keys(&mut state, &[KEY_VERB, KEY_3, KEY_7, KEY_ENTER]);
    press_keys(&mut state, &[KEY_NOUN, KEY_4, KEY_0, KEY_ENTER]);

    assert_eq!(state.ctx.display.major_mode.val(), 40);
    assert_eq!(state.ctx.guidance.active_program, Some(40));
}

// ---------------------------------------------------------------------------
// Test 4: V06 N60 — DisplayOnce setzt Verb+Noun, kein Flash
// ---------------------------------------------------------------------------

/// `VERB 0 6 ENTER NOUN 6 0 ENTER` → V06 N60 aktiv, kein Blinken.
#[test]
fn v06n60_display_once_no_flash() {
    let mut state = SimState::new();

    press_keys(&mut state, &[KEY_VERB, KEY_0, KEY_6, KEY_ENTER]);
    press_keys(&mut state, &[KEY_NOUN, KEY_6, KEY_0, KEY_ENTER]);

    assert_eq!(state.ctx.display.verb.val(), 6, "Verb sollte 6 sein");
    assert_eq!(state.ctx.display.noun.val(), 60, "Noun sollte 60 sein");
    // DisplayOnce stoppt Flash
    assert!(
        !state.ctx.display.flash_state,
        "Flash sollte bei V06 gestoppt sein"
    );
}

// ---------------------------------------------------------------------------
// Test 5: V16 N67 — Monitor-Modus aktiv
// ---------------------------------------------------------------------------

/// `VERB 1 6 ENTER NOUN 6 7 ENTER` → V16 N67 aktiv.
/// Pro Tick wird NounResolver aufgerufen (kein Crash, kein Flash).
#[test]
fn v16n67_monitor_mode_no_crash() {
    let mut state = SimState::new();

    press_keys(&mut state, &[KEY_VERB, KEY_1, KEY_6, KEY_ENTER]);
    press_keys(&mut state, &[KEY_NOUN, KEY_6, KEY_7, KEY_ENTER]);

    assert_eq!(state.ctx.display.verb.val(), 16);
    assert_eq!(state.ctx.display.noun.val(), 67);

    // Mehrere Ticks: Monitor-Modus darf keinen Crash erzeugen
    for _ in 0..20 {
        sim_tick(&mut state);
    }

    // Flash ist bei V16 deaktiviert
    assert!(
        !state.ctx.display.flash_state,
        "Flash sollte bei V16 deaktiviert sein"
    );
}

// ---------------------------------------------------------------------------
// Test 6: V33 (Proceed) — proceed_flag gesetzt
// ---------------------------------------------------------------------------

/// `VERB 3 3 ENTER` → guidance.proceed_flag = true.
#[test]
fn v33_proceed_sets_flag() {
    let mut state = SimState::new();
    assert!(!state.ctx.guidance.proceed_flag);

    press_keys(&mut state, &[KEY_VERB, KEY_3, KEY_3, KEY_ENTER]);

    assert!(
        state.ctx.guidance.proceed_flag,
        "proceed_flag nach V33 nicht gesetzt"
    );
}

// ---------------------------------------------------------------------------
// Test 7: V34 (Terminate) — terminate_flag gesetzt
// ---------------------------------------------------------------------------

/// `VERB 3 4 ENTER` → guidance.terminate_flag = true.
#[test]
fn v34_terminate_sets_flag() {
    let mut state = SimState::new();
    assert!(!state.ctx.guidance.terminate_flag);

    press_keys(&mut state, &[KEY_VERB, KEY_3, KEY_4, KEY_ENTER]);

    assert!(
        state.ctx.guidance.terminate_flag,
        "terminate_flag nach V34 nicht gesetzt"
    );
}

// ---------------------------------------------------------------------------
// Test 8: TEI-Sequenz P30 → P40 (Verb-Dispatcher Verkettung)
// ---------------------------------------------------------------------------

/// Simuliert einen vereinfachten TEI-Ablauf:
/// 1. `V37 N30 ENTER` → P30 starten
/// 2. `V06 N81 ENTER` → ΔV-Anzeige (nur Verb+Noun prüfen, kein P30-Zustand)
/// 3. `V33 ENTER` → Proceed
/// 4. `V37 N40 ENTER` → P40 starten (SPS-Burn)
#[test]
fn tei_sequence_p30_then_p40() {
    let mut state = SimState::new();

    // 1. P30 starten
    press_keys(&mut state, &[KEY_VERB, KEY_3, KEY_7, KEY_ENTER]);
    press_keys(&mut state, &[KEY_NOUN, KEY_3, KEY_0, KEY_ENTER]);
    assert_eq!(state.ctx.guidance.active_program, Some(30));
    assert_eq!(state.ctx.display.major_mode.val(), 30);

    // 2. V06 N81 — ΔV-Anzeige (Verb=6, Noun=81)
    press_keys(&mut state, &[KEY_VERB, KEY_0, KEY_6, KEY_ENTER]);
    press_keys(&mut state, &[KEY_NOUN, KEY_8, KEY_1, KEY_ENTER]);
    assert_eq!(state.ctx.display.verb.val(), 6);
    assert_eq!(state.ctx.display.noun.val(), 81);

    // 3. Proceed (V33)
    press_keys(&mut state, &[KEY_VERB, KEY_3, KEY_3, KEY_ENTER]);
    assert!(state.ctx.guidance.proceed_flag);

    // 4. P40 starten (SPS-Burn)
    press_keys(&mut state, &[KEY_VERB, KEY_3, KEY_7, KEY_ENTER]);
    press_keys(&mut state, &[KEY_NOUN, KEY_4, KEY_0, KEY_ENTER]);
    assert_eq!(state.ctx.guidance.active_program, Some(40));
    assert_eq!(state.ctx.display.major_mode.val(), 40);
}

// ---------------------------------------------------------------------------
// Test 9: P52-Startsequenz (IMU Alignment)
// ---------------------------------------------------------------------------

/// `VERB 3 7 ENTER NOUN 5 2 ENTER` → PROG=52, active_program=52.
#[test]
fn p52_start_sequence_sets_program_52() {
    let mut state = SimState::new();

    press_keys(&mut state, &[KEY_VERB, KEY_3, KEY_7, KEY_ENTER]);
    press_keys(&mut state, &[KEY_NOUN, KEY_5, KEY_2, KEY_ENTER]);

    assert_eq!(state.ctx.display.major_mode.val(), 52);
    assert_eq!(state.ctx.guidance.active_program, Some(52));
    assert!(!state.ctx.display.flash_state);
}

// ---------------------------------------------------------------------------
// Test 10: VerbSet-Dispatch-Regression — V16 ENTER darf Noun nicht überschreiben
// ---------------------------------------------------------------------------

/// Stellt sicher dass `V16 ENTER` allein keinen Dispatch auslöst der den
/// zuvor gesetzten Noun auf N00 zurücksetzt.
///
/// Vorher (Bug): VerbSet dispatcht sofort → `dispatch(V16, N00)` → setzt
/// `ctx.display.noun = N00`, überschreibt vorherigen Noun.
/// Nachher (Fix): VerbSet dispatcht nur bei V32/V33/V34; Noun bleibt erhalten.
#[test]
fn verbset_does_not_overwrite_existing_noun() {
    let mut state = SimState::new();

    // Erst einen Noun setzen
    press_keys(&mut state, &[KEY_VERB, KEY_3, KEY_7, KEY_ENTER]);
    press_keys(&mut state, &[KEY_NOUN, KEY_6, KEY_1, KEY_ENTER]);
    assert_eq!(state.ctx.display.noun.val(), 61, "Setup: noun=61 erwartet");

    // Jetzt V16 ENTER — darf noun NICHT zurücksetzen
    press_keys(&mut state, &[KEY_VERB, KEY_1, KEY_6, KEY_ENTER]);

    // Noun muss noch 61 sein (nicht auf 00 zurückgesetzt)
    assert_eq!(
        state.ctx.display.noun.val(),
        61,
        "Noun darf nach VerbSet(16) nicht überschrieben werden (war 61)"
    );
    // Verb muss auf 16 gesetzt sein
    assert_eq!(
        state.ctx.display.verb.val(),
        16,
        "Verb nach V16-ENTER muss 16 sein"
    );
}

// ---------------------------------------------------------------------------
// Test 11: V16N67 — Verb und Noun korrekt gesetzt, kein Flash
// ---------------------------------------------------------------------------

/// Vollständige V16N67-Sequenz: Verb=16, Noun=67, flash=false.
/// Regression für den Bug "Verb wird nicht angezeigt".
#[test]
fn v16n67_verb_and_noun_set_no_flash() {
    let mut state = SimState::new();

    press_keys(&mut state, &[KEY_VERB, KEY_1, KEY_6, KEY_ENTER]);
    // Zwischen VerbSet und NounSet: Noun darf nicht auf 0 fallen
    assert_ne!(
        state.ctx.display.verb.val(),
        0,
        "Verb darf nach V16-ENTER nicht 0 sein"
    );

    press_keys(&mut state, &[KEY_NOUN, KEY_6, KEY_7, KEY_ENTER]);

    // Verb = 16, Noun = 67
    assert_eq!(state.ctx.display.verb.val(), 16, "Verb muss 16 sein");
    assert_eq!(state.ctx.display.noun.val(), 67, "Noun muss 67 sein");

    // Kein Blinken (V16 = Monitor, kein Flash)
    assert!(
        !state.ctx.display.flash_state,
        "flash_state muss false sein bei V16"
    );
    assert!(
        !state.ctx.display.lights.verb_noun_flash,
        "verb_noun_flash muss false sein bei V16"
    );
}

// ---------------------------------------------------------------------------
// Test 12: V33 — Proceed-Flag gesetzt auch ohne vorherigen Noun (VerbSet-Dispatch)
// ---------------------------------------------------------------------------

/// V33 (Proceed) muss auch ohne vorherigen Noun-ENTER durch VerbSet ausgelöst
/// werden, da es ein Sofort-Verb ist (kein Noun benötigt).
#[test]
fn v33_verbset_immediate_dispatch() {
    let mut state = SimState::new();
    assert!(
        !state.ctx.guidance.proceed_flag,
        "proceed_flag initial false"
    );

    // V33 ENTER — kein Noun nötig, Dispatch sofort
    press_keys(&mut state, &[KEY_VERB, KEY_3, KEY_3, KEY_ENTER]);

    assert!(
        state.ctx.guidance.proceed_flag,
        "proceed_flag nach V33 ENTER muss true sein"
    );
}

// ---------------------------------------------------------------------------
// Test 13: V34 — Terminate-Flag gesetzt durch VerbSet-Dispatch
// ---------------------------------------------------------------------------

/// V34 (Terminate) muss durch VerbSet sofort dispatcht werden.
#[test]
fn v34_verbset_immediate_dispatch() {
    let mut state = SimState::new();

    press_keys(&mut state, &[KEY_VERB, KEY_3, KEY_4, KEY_ENTER]);

    assert!(
        state.ctx.guidance.terminate_flag,
        "terminate_flag nach V34 ENTER muss true sein"
    );
}

// ---------------------------------------------------------------------------
// Test 14: V37 erster ENTER → Flash aktiv, kein Programm gestartet
// ---------------------------------------------------------------------------

/// Crew drückt `VERB 3 7 ENTER` (ohne Noun).
///
/// Erwartet:
/// - `display.flash_state == true` (Flash aktiv — wartet auf Programmnummer)
/// - `active_program == None` (kein Programm gestartet)
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, REQMM label.
#[test]
fn v37_first_enter_activates_flash() {
    let mut state = SimState::new();

    // V37 ENTER — erster Schritt: RequestProgram → Flash einschalten
    press_keys(&mut state, &[KEY_VERB, KEY_3, KEY_7, KEY_ENTER]);

    // Flash muss aktiv sein (wartet auf Programm-Nummer)
    assert!(
        state.ctx.display.flash_state,
        "flash_state nach V37 ENTER muss true sein (warte auf Programmnummer)"
    );

    // Kein Programm gestartet
    assert_eq!(
        state.ctx.guidance.active_program, None,
        "active_program nach V37 ENTER allein muss None sein"
    );

    // Verb muss auf 37 gesetzt sein
    assert_eq!(
        state.ctx.display.verb.val(),
        37,
        "Verb nach V37 ENTER muss 37 sein"
    );
}

/// Vollständiger zweistufiger V37-Ablauf:
/// 1. `VERB 3 7 ENTER` → Flash aktiv
/// 2. `NOUN 6 1 ENTER` → P61 gestartet, Flash gestoppt
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, REQMM/MMCHANG.
#[test]
fn v37_two_step_starts_program_and_stops_flash() {
    let mut state = SimState::new();

    // Schritt 1: V37 ENTER — Flash einschalten
    press_keys(&mut state, &[KEY_VERB, KEY_3, KEY_7, KEY_ENTER]);
    assert!(
        state.ctx.display.flash_state,
        "Nach V37 ENTER muss Flash aktiv sein"
    );

    // Schritt 2: NOUN 6 1 ENTER — Programm starten
    press_keys(&mut state, &[KEY_NOUN, KEY_6, KEY_1, KEY_ENTER]);

    assert_eq!(
        state.ctx.display.major_mode.val(),
        61,
        "major_mode nach V37→N61 muss 61 sein"
    );
    assert_eq!(
        state.ctx.guidance.active_program,
        Some(61),
        "active_program nach V37→N61 muss Some(61) sein"
    );
    assert!(
        !state.ctx.display.flash_state,
        "Flash muss nach Programm-Start gestoppt sein"
    );
}

// ---------------------------------------------------------------------------
// Test 15: V25 N61 ENTER — accept_load liest aus load_r1/r2/r3, nicht Display
// ---------------------------------------------------------------------------

/// Stellt sicher dass `accept_load` beim ENTER-Tastendruck die Werte aus
/// `SimState::load_r1/r2/r3` verwendet, nicht aus den Display-Registern.
///
/// Aufbau:
/// 1. V25 N61 ENTER — Lade-Verb aktiv, wartet auf Eingabe.
/// 2. `state.load_r1 = 2525` (25.25°), `load_r2 = -800` (-8.00°), `load_r3 = 1`.
/// 3. Display-Register werden auf einen abweichenden Wert gesetzt, um zu
///    beweisen dass `accept_load` den Eingabepuffer, nicht das Display, liest.
/// 4. ENTER drücken → `accept_load` mit load_r1/r2/r3-Werten aufgerufen.
/// 5. `guidance.splashdown_lat/lon` und `heads_up` prüfen.
#[test]
fn v25n61_enter_uses_load_buffer_not_display() {
    use agc_core::display::verb_noun::DisplayRegister;
    use std::f64::consts::PI;

    let mut state = SimState::new();

    // V25 N61 ENTER — Lade-Sequenz starten
    press_keys(&mut state, &[KEY_VERB, KEY_2, KEY_5, KEY_ENTER]);
    press_keys(&mut state, &[KEY_NOUN, KEY_6, KEY_1, KEY_ENTER]);

    // Eingabepuffer mit Crew-Werten füllen
    state.load_r1 = 2525; // 25.25°
    state.load_r2 = -800; // -8.00°
    state.load_r3 = 1; // heads-up

    // Display-Register absichtlich auf anderen Wert setzen —
    // wenn accept_load fälschlicherweise hieraus liest, schlägt der Test fehl.
    state.ctx.display.r1 = DisplayRegister::Decimal(9999);
    state.ctx.display.r2 = DisplayRegister::Decimal(-9999);
    state.ctx.display.r3 = DisplayRegister::Decimal(0);

    // ENTER drücken — löst accept_load(load_r1, load_r2, load_r3) aus
    press_keys(&mut state, &[KEY_ENTER]);

    // Erwartete Werte in rad (1 centideg = π/18000 rad)
    let centideg_to_rad = PI / 18_000.0_f64;
    let expected_lat = 2525.0 * centideg_to_rad;
    let expected_lon = -800.0 * centideg_to_rad;

    assert!(
        (state.ctx.guidance.splashdown_lat - expected_lat).abs() < 1e-12,
        "splashdown_lat: erwartet {expected_lat:.10}, got {:.10}",
        state.ctx.guidance.splashdown_lat
    );
    assert!(
        (state.ctx.guidance.splashdown_lon - expected_lon).abs() < 1e-12,
        "splashdown_lon: erwartet {expected_lon:.10}, got {:.10}",
        state.ctx.guidance.splashdown_lon
    );
    assert!(
        state.ctx.guidance.heads_up,
        "heads_up muss true sein (load_r3 = 1)"
    );
}

/// Drei sequenzielle Noun-Eingaben im V25-Flash-Modus befüllen load_r1/r2/r3.
///
/// Ablauf:
/// 1. `VERB 2 5 ENTER NOUN 6 1 ENTER` → V25 N61 aktiv, Flash an.
/// 2. Erste Noun-Eingabe (`NOUN 1 1 ENTER`) → load_r1 = 11, load_reg_idx = 1.
/// 3. Zweite Noun-Eingabe (`NOUN 2 2 ENTER`) → load_r2 = 22, load_reg_idx = 2.
/// 4. Dritte Noun-Eingabe (`NOUN 3 3 ENTER`) → load_r3 = 33, load_reg_idx bleibt 2.
#[test]
fn v25n61_sequential_noun_entries_fill_load_registers() {
    let mut state = SimState::new();

    // V25 N61 ENTER — Lade-Sequenz starten (NounSet löst Dispatcher aus → flash=true)
    press_keys(&mut state, &[KEY_VERB, KEY_2, KEY_5, KEY_ENTER]);
    press_keys(&mut state, &[KEY_NOUN, KEY_6, KEY_1, KEY_ENTER]);

    assert!(
        state.ctx.display.flash_state,
        "Flash muss nach V25 N61 aktiv sein"
    );
    assert_eq!(state.load_reg_idx, 0, "Index startet bei 0");

    // Erste Dateneingabe: Noun 11
    press_keys(&mut state, &[KEY_NOUN, KEY_1, KEY_1, KEY_ENTER]);
    assert_eq!(state.load_r1, 11, "load_r1 muss 11 sein");
    assert_eq!(state.load_reg_idx, 1, "Index nach erster Eingabe = 1");

    // Zweite Dateneingabe: Noun 22
    press_keys(&mut state, &[KEY_NOUN, KEY_2, KEY_2, KEY_ENTER]);
    assert_eq!(state.load_r2, 22, "load_r2 muss 22 sein");
    assert_eq!(state.load_reg_idx, 2, "Index nach zweiter Eingabe = 2");

    // Dritte Dateneingabe: Noun 33
    press_keys(&mut state, &[KEY_NOUN, KEY_3, KEY_3, KEY_ENTER]);
    assert_eq!(state.load_r3, 33, "load_r3 muss 33 sein");
    assert_eq!(state.load_reg_idx, 2, "Index bleibt bei 2 (kein Überlauf)");
}

// ---------------------------------------------------------------------------
// Test 16: V41 — Flash aktiv nach Enter, alignment_cmd noch None
// ---------------------------------------------------------------------------

/// `VERB 4 1 ENTER` → Flash aktiv, V25 N22 blinkend, alignment_cmd == None.
///
/// AGC source: `Comanche055/EXTENDED_VERBS.agc`, VBCOARK label.
#[test]
fn v41_first_enter_shows_flash_v25n22() {
    use agc_core::display::verb_noun::KEY_PRO;

    let mut state = SimState::new();

    // V41 ENTER — erster Schritt: Flash einschalten mit V25/N22
    press_keys(&mut state, &[KEY_VERB, KEY_4, KEY_1, KEY_ENTER]);

    // Flash muss aktiv sein (Lade-Prompt für ICDU-Winkel)
    assert!(
        state.ctx.display.flash_state,
        "flash_state nach V41 ENTER muss true sein (Lade-Prompt V25N22)"
    );
    assert_eq!(
        state.ctx.display.verb.val(),
        25,
        "Verb nach V41 muss 25 sein (V25 Lade-Prompt)"
    );
    assert_eq!(
        state.ctx.display.noun.val(),
        22,
        "Noun nach V41 muss 22 sein (N22 ICDU-Winkel)"
    );

    // alignment_cmd darf NOCH NICHT gesetzt sein (Crew hat noch nicht bestätigt)
    assert_eq!(
        state.ctx.guidance.alignment_cmd, None,
        "alignment_cmd nach V41 ENTER muss None sein (Crew noch nicht bestätigt)"
    );

    // --- Bestätigung mit PRO (V33 / Proceed) ---
    press_keys(&mut state, &[KEY_PRO]);

    // alignment_cmd muss jetzt gesetzt sein
    assert_eq!(
        state.ctx.guidance.alignment_cmd,
        Some(agc_core::context::AlignmentCommand::CoarseAlign),
        "alignment_cmd nach V41→PRO muss CoarseAlign sein"
    );
    // Flash muss gestoppt sein
    assert!(
        !state.ctx.display.flash_state,
        "Flash muss nach PRO-Bestätigung gestoppt sein"
    );
}

// ---------------------------------------------------------------------------
// Test 17: V41 — alignment_cmd per ENTER (statt PRO) gesetzt
// ---------------------------------------------------------------------------

/// `VERB 4 1 ENTER` → Flash aktiv; danach ENTER → alignment_cmd = CoarseAlign.
///
/// AGC source: `Comanche055/EXTENDED_VERBS.agc`, VBCOARK label.
#[test]
fn v41_enter_confirm_sets_coarse_align() {
    let mut state = SimState::new();

    // V41 ENTER — Flash einschalten
    press_keys(&mut state, &[KEY_VERB, KEY_4, KEY_1, KEY_ENTER]);
    assert!(state.ctx.display.flash_state, "Flash nach V41 erwartet");
    assert_eq!(state.ctx.guidance.alignment_cmd, None);

    // ENTER — Bestätigung (zweiter Weg neben PRO)
    press_keys(&mut state, &[KEY_ENTER]);

    assert_eq!(
        state.ctx.guidance.alignment_cmd,
        Some(agc_core::context::AlignmentCommand::CoarseAlign),
        "alignment_cmd nach V41→ENTER muss CoarseAlign sein"
    );
    assert!(
        !state.ctx.display.flash_state,
        "Flash muss nach ENTER gestoppt sein"
    );
}

// ---------------------------------------------------------------------------
// Test 18: V41 — Terminate bricht Prompt ab (kein alignment_cmd)
// ---------------------------------------------------------------------------

/// `VERB 4 1 ENTER` → Flash aktiv; danach V34 ENTER → Flash gestoppt, alignment_cmd None.
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, TERMKEY.
#[test]
fn v41_terminate_cancels_prompt_no_alignment_cmd() {
    use agc_core::display::verb_noun::KEY_PRO;
    let _ = KEY_PRO; // suppress unused import warning

    let mut state = SimState::new();

    // V41 ENTER — Flash einschalten
    press_keys(&mut state, &[KEY_VERB, KEY_4, KEY_1, KEY_ENTER]);
    assert!(state.ctx.display.flash_state, "Flash nach V41 erwartet");

    // V34 ENTER — Terminate: Prompt abbrechen
    press_keys(&mut state, &[KEY_VERB, KEY_3, KEY_4, KEY_ENTER]);

    assert_eq!(
        state.ctx.guidance.alignment_cmd, None,
        "alignment_cmd nach Terminate muss None bleiben"
    );
    assert!(
        !state.ctx.display.flash_state,
        "Flash muss nach Terminate gestoppt sein"
    );
}

// ---------------------------------------------------------------------------
// Test 19: V42 — Flash aktiv nach Enter, alignment_cmd noch None
// ---------------------------------------------------------------------------

/// `VERB 4 2 ENTER` → Flash aktiv, V25 N93 blinkend, alignment_cmd == None.
///
/// AGC source: `Comanche055/EXTENDED_VERBS.agc`, IMUFINEK label.
#[test]
fn v42_first_enter_shows_flash_v25n93() {
    use agc_core::display::verb_noun::KEY_PRO;

    let mut state = SimState::new();

    // V42 ENTER — erster Schritt: Flash einschalten mit V25/N93
    press_keys(&mut state, &[KEY_VERB, KEY_4, KEY_2, KEY_ENTER]);

    // Flash muss aktiv sein
    assert!(
        state.ctx.display.flash_state,
        "flash_state nach V42 ENTER muss true sein (Lade-Prompt V25N93)"
    );
    assert_eq!(
        state.ctx.display.verb.val(),
        25,
        "Verb nach V42 muss 25 sein"
    );
    assert_eq!(
        state.ctx.display.noun.val(),
        93,
        "Noun nach V42 muss 93 sein (N93 Delta-Gyro)"
    );

    // alignment_cmd darf NOCH NICHT gesetzt sein
    assert_eq!(
        state.ctx.guidance.alignment_cmd, None,
        "alignment_cmd nach V42 ENTER muss None sein"
    );

    // --- Bestätigung mit PRO ---
    press_keys(&mut state, &[KEY_PRO]);

    assert_eq!(
        state.ctx.guidance.alignment_cmd,
        Some(agc_core::context::AlignmentCommand::FineAlign),
        "alignment_cmd nach V42→PRO muss FineAlign sein"
    );
    assert!(
        !state.ctx.display.flash_state,
        "Flash muss nach PRO-Bestätigung gestoppt sein"
    );
}

// ---------------------------------------------------------------------------
// Test 20: V42 — alignment_cmd per ENTER gesetzt
// ---------------------------------------------------------------------------

/// `VERB 4 2 ENTER` → Flash aktiv; danach ENTER → alignment_cmd = FineAlign.
///
/// AGC source: `Comanche055/EXTENDED_VERBS.agc`, IMUFINEK label.
#[test]
fn v42_enter_confirm_sets_fine_align() {
    let mut state = SimState::new();

    // V42 ENTER — Flash einschalten
    press_keys(&mut state, &[KEY_VERB, KEY_4, KEY_2, KEY_ENTER]);
    assert!(state.ctx.display.flash_state, "Flash nach V42 erwartet");
    assert_eq!(state.ctx.guidance.alignment_cmd, None);

    // ENTER — Bestätigung
    press_keys(&mut state, &[KEY_ENTER]);

    assert_eq!(
        state.ctx.guidance.alignment_cmd,
        Some(agc_core::context::AlignmentCommand::FineAlign),
        "alignment_cmd nach V42→ENTER muss FineAlign sein"
    );
    assert!(
        !state.ctx.display.flash_state,
        "Flash muss nach ENTER gestoppt sein"
    );
}
