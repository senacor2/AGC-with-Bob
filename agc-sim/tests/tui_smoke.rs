//! Smoke-Tests für die DSKY-TUI-Infrastruktur (Task 12).
//!
//! Diese Tests verifizieren, dass:
//! 1. `SimState::new()` ohne Crash startet.
//! 2. Mehrere `sim_tick()`-Aufrufe stabil laufen.
//! 3. V37E (Programm-Wahl P00 via Verb 37, ENTER) keine Abstürze erzeugt.
//! 4. Das Key-Mapping korrekt arbeitet.
//! 5. Segment-Rendering keine Panics erzeugt.

use agc_sim::sim_loop::{map_key, sim_tick, SimState};
use agc_sim::tui::segment7::{format_blank, format_decimal, format_two_digit};
use crossterm::event::{KeyCode, KeyModifiers};

// ---------------------------------------------------------------------------
// Smoke-Test: SimState startet ohne Crash
// ---------------------------------------------------------------------------

#[test]
fn sim_state_new_no_crash() {
    let state = SimState::new();
    // Initiale LEO-Bahn: Höhe ca. 400 km
    let r = state.traj.position.magnitude();
    assert!(
        r > 6_700_000.0 && r < 6_800_000.0,
        "Startposition außerhalb erwarteter LEO-Höhe: r={r}"
    );
    assert_eq!(state.tick, 0);
    assert!(!state.comp_acty);
}

// ---------------------------------------------------------------------------
// Smoke-Test: 100 Ticks laufen stabil
// ---------------------------------------------------------------------------

#[test]
fn sim_tick_100_steps_no_crash() {
    let mut state = SimState::new();
    for _ in 0..100 {
        sim_tick(&mut state);
    }
    assert_eq!(state.tick, 100);
    // Bahn sollte noch plausibel sein
    let r = state.traj.position.magnitude();
    assert!(r > 6_000_000.0, "Trajektorie kollabiert: r={r}");
}

// ---------------------------------------------------------------------------
// Smoke-Test: V37E (Verb=37, ENTER) — kein Crash
// ---------------------------------------------------------------------------

#[test]
fn v37e_no_crash() {
    use agc_core::display::verb_noun::{KEY_0, KEY_3, KEY_7, KEY_ENTER, KEY_VERB};

    let mut state = SimState::new();

    // Tastenfolge: VERB, 3, 7, ENTER
    for &key in &[KEY_VERB, KEY_3, KEY_7, KEY_ENTER] {
        state.dsky.push_key(key);
        sim_tick(&mut state);
    }

    // Verb=37 sollte gesetzt worden sein
    assert_eq!(
        state.ctx.display.verb.val(),
        37,
        "Verb nach V37E nicht 37, sondern {}",
        state.ctx.display.verb.val()
    );
    // KEY_0 fehlt — kein Crash ist das Ziel
    let _ = KEY_0;
}

// ---------------------------------------------------------------------------
// Smoke-Test: Key-Mapping Grundfunktion
// ---------------------------------------------------------------------------

#[test]
fn key_mapping_digits() {
    use agc_core::display::verb_noun::{
        KEY_0, KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_9,
    };

    let pairs = [
        ('0', KEY_0),
        ('1', KEY_1),
        ('2', KEY_2),
        ('3', KEY_3),
        ('4', KEY_4),
        ('5', KEY_5),
        ('6', KEY_6),
        ('7', KEY_7),
        ('8', KEY_8),
        ('9', KEY_9),
    ];

    for (ch, expected) in pairs {
        let mapped = map_key(KeyCode::Char(ch), KeyModifiers::empty());
        assert_eq!(
            mapped,
            Some(expected),
            "Zeichen '{ch}' → erwartet {expected}, got {mapped:?}"
        );
    }
}

#[test]
fn key_mapping_verb_noun_enter() {
    use agc_core::display::verb_noun::{KEY_ENTER, KEY_NOUN, KEY_VERB};

    assert_eq!(
        map_key(KeyCode::Char('v'), KeyModifiers::empty()),
        Some(KEY_VERB)
    );
    assert_eq!(
        map_key(KeyCode::Char('V'), KeyModifiers::empty()),
        Some(KEY_VERB)
    );
    assert_eq!(
        map_key(KeyCode::Char('n'), KeyModifiers::empty()),
        Some(KEY_NOUN)
    );
    assert_eq!(
        map_key(KeyCode::Enter, KeyModifiers::empty()),
        Some(KEY_ENTER)
    );
    // 'e' und 'E' sind ebenfalls ENTER (für V37E-Eingabe ohne Enter-Taste)
    assert_eq!(
        map_key(KeyCode::Char('e'), KeyModifiers::empty()),
        Some(KEY_ENTER)
    );
    assert_eq!(
        map_key(KeyCode::Char('E'), KeyModifiers::empty()),
        Some(KEY_ENTER)
    );
}

#[test]
fn key_mapping_ctrl_c_no_agc_code() {
    // Ctrl+C soll nicht als AGC-Keycode landen (wird als Beenden-Signal behandelt)
    let mapped = map_key(KeyCode::Char('c'), KeyModifiers::CONTROL);
    // Mit CONTROL-Modifier: kein CLR
    assert_ne!(
        mapped,
        Some(agc_core::display::verb_noun::KEY_CLR),
        "Ctrl+C darf nicht als CLR interpretiert werden"
    );
}

#[test]
fn key_mapping_quit_returns_none() {
    // 'q' ist kein AGC-Keycode, sondern Beenden
    assert_eq!(map_key(KeyCode::Char('q'), KeyModifiers::empty()), None);
}

// ---------------------------------------------------------------------------
// Smoke-Test: Segment-Rendering
// ---------------------------------------------------------------------------

#[test]
fn segment7_digits_no_panic() {
    for d in 0u8..10 {
        let (top, mid, bot) = format_two_digit(d * 10 + d);
        assert!(!top.is_empty());
        assert!(!mid.is_empty());
        assert!(!bot.is_empty());
    }
}

#[test]
fn segment7_decimal_format() {
    for val in [-99999i32, -1, 0, 1, 99999] {
        let (top, mid, bot) = format_decimal(val);
        assert!(!top.is_empty(), "top leer für {val}");
        assert!(!mid.is_empty(), "mid leer für {val}");
        assert!(!bot.is_empty(), "bot leer für {val}");
    }
}

#[test]
fn segment7_blank_no_panic() {
    let (top, mid, bot) = format_blank();
    assert!(!top.is_empty());
    assert!(!mid.is_empty());
    assert!(!bot.is_empty());
}

// ---------------------------------------------------------------------------
// Smoke-Test: COMP ACTY Blink-Logik
// ---------------------------------------------------------------------------

#[test]
fn comp_acty_cycles() {
    let mut state = SimState::new();
    // Tick 0–3: COMP ACTY ein
    for _ in 0..4 {
        sim_tick(&mut state);
        assert!(
            state.comp_acty,
            "comp_acty bei tick {} nicht true",
            state.tick
        );
    }
    // Tick 4–39: COMP ACTY aus
    for _ in 0..36 {
        sim_tick(&mut state);
        assert!(
            !state.comp_acty,
            "comp_acty bei tick {} nicht false",
            state.tick
        );
    }
    // Tick 40–43: wieder ein
    for _ in 0..4 {
        sim_tick(&mut state);
        assert!(
            state.comp_acty,
            "comp_acty bei tick {} nicht true",
            state.tick
        );
    }
}
