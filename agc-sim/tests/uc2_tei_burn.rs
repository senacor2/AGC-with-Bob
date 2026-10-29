//! UC-2 Integrationstest: TEI-Vorbereitung (P30 → P40)
//!
//! Verifiziert den Trans-Earth-Injection-Burn-Ablauf:
//! 1. Szenario `tei_pre.scenario` laden (Mondorbit 110 km)
//! 2. `V37E30E` → PROG=30, P30State gesetzt
//! 3. `V06 N44 ENTR` → HAPO/HPER/TFF in R1–R3 (nicht Blank)
//! 4. `V37E40E` → PROG=40
//! 5. `V16 N40 ENTR` → Monitor-Modus aktiv, N40 zeigt TTOGO/VG/DVTOTAL
//!
//! DSKY-Notation: `V37E30E` = VERB 3 7 ENTER 3 0 ENTER
//! Quelle: "The Apollo Guidance Computer", Kap. 5 „External ΔV Targeting".

#[path = "test_utils.rs"]
mod test_utils;

use agc_core::display::verb_noun::{
    DisplayRegister, KEY_0, KEY_1, KEY_3, KEY_4, KEY_6, KEY_7, KEY_ENTER, KEY_NOUN, KEY_VERB,
};

const SCENARIO: &str = "docs/scenarios/tei_pre.scenario";

// ---------------------------------------------------------------------------
// UC-2-A: Zentralkörper — Mondorbit wird korrekt erkannt
// ---------------------------------------------------------------------------

#[test]
fn uc2_moon_body_detected() {
    use agc_physics::bodies::Body;

    let state = test_utils::load_scenario_state(SCENARIO);
    assert_eq!(
        state.step_body,
        Body::Moon,
        "tei_pre.scenario muss Body::Moon ergeben (r < 3 000 000 m)"
    );
}

// ---------------------------------------------------------------------------
// UC-2-B: P30 starten — PROG=30
// ---------------------------------------------------------------------------

#[test]
fn uc2_p30_start() {
    let mut state = test_utils::load_scenario_state(SCENARIO);

    // V37E30E — Programm 30 starten
    test_utils::press_keys(
        &mut state,
        &[
            KEY_VERB, KEY_3, KEY_7, KEY_ENTER, // V37E
            KEY_3, KEY_0, KEY_ENTER, // 30E
        ],
    );

    assert_eq!(
        state.ctx.display.major_mode.val(),
        30,
        "major_mode nach V37E30E erwartet 30"
    );
    assert_eq!(
        state.ctx.guidance.active_program,
        Some(30),
        "active_program nach V37E30E erwartet Some(30)"
    );
}

// ---------------------------------------------------------------------------
// UC-2-C: N44 zeigt sofort Kreisbahnparameter nach P30-Start (kein Δv nötig)
//
// P30 berechnet nach V37E30E direkt aus dem aktuellen Bahnzustandsvektor —
// auch ohne vorherige Δv-Eingabe (delta_v_lv = [0,0,0] = Null-Manöver).
// Das Ergebnis ist die aktuelle Kreisbahn-Apogee/Perigee.
// ---------------------------------------------------------------------------

#[test]
fn uc2_n44_blank_when_p30_no_output() {
    let mut state = test_utils::load_scenario_state(SCENARIO);

    // P30 starten — ohne Δv-Eingabe berechnet P30 die aktuelle Kreisbahn
    test_utils::press_keys(
        &mut state,
        &[KEY_VERB, KEY_3, KEY_7, KEY_ENTER, KEY_3, KEY_0, KEY_ENTER],
    );

    // ctx.current_sv muss vor dem Dispatch gesetzt sein — sim_tick macht das,
    // aber beim direkten Tastatur-Dispatch im Test läuft kein Tick.
    // Wir setzen current_sv manuell auf den tei_pre-Szenario-Zustandsvektor.
    {
        use agc_core::navigation::state_vector::StateVector;
        use agc_core::types::{Met, Vec3};
        let pos = state.traj.position;
        let vel = state.traj.velocity;
        state.ctx.current_sv = StateVector::new(
            Vec3::new(pos.x, pos.y, pos.z),
            Vec3::new(vel.x, vel.y, vel.z),
            Met::from_seconds(state.traj.time_s),
        );
    }

    // P30 nochmals auslösen (mit gesetztem current_sv)
    test_utils::press_keys(
        &mut state,
        &[KEY_VERB, KEY_3, KEY_7, KEY_ENTER, KEY_3, KEY_0, KEY_ENTER],
    );

    // V06 N44 ENTER — einmalige Anzeige HAPO/HPER/TFF
    test_utils::press_keys(
        &mut state,
        &[
            KEY_VERB, KEY_0, KEY_6, KEY_ENTER, KEY_NOUN, KEY_4, KEY_4, KEY_ENTER,
        ],
    );

    // Nach P30-Berechnung aus gültigem SV muss R1 Werte enthalten (nicht Blank)
    let r1_blank = matches!(state.ctx.display.r1, DisplayRegister::Blank);
    assert!(
        !r1_blank,
        "R1 (HAPO) darf nicht Blank sein — P30 berechnet sofort aus aktuellem SV"
    );
}

// ---------------------------------------------------------------------------
// UC-2-C2: N44 zeigt Werte wenn P30-Output manuell gesetzt wird
// ---------------------------------------------------------------------------

#[test]
fn uc2_n44_shows_values_when_p30_output_set() {
    use agc_core::guidance::p30_37::P30Output;
    use agc_core::types::Vec3;

    let mut state = test_utils::load_scenario_state(SCENARIO);

    // P30 starten
    test_utils::press_keys(
        &mut state,
        &[KEY_VERB, KEY_3, KEY_7, KEY_ENTER, KEY_3, KEY_0, KEY_ENTER],
    );

    // P30-Output manuell setzen (simuliert berechnetes Manöver)
    state.ctx.guidance.p30 = Some(P30Output {
        delta_v_ref: Vec3::new(100.0, 0.0, 0.0),
        h_apo: 5_000_000.0,
        h_per: 1_000_000.0,
        tff_s: 3600.0,
    });

    // V06 N44 ENTER — einmalige Anzeige HAPO/HPER/TFF
    test_utils::press_keys(
        &mut state,
        &[
            KEY_VERB, KEY_0, KEY_6, KEY_ENTER, KEY_NOUN, KEY_4, KEY_4, KEY_ENTER,
        ],
    );

    // Mit P30-Output darf R1 nicht Blank sein
    let r1_blank = matches!(state.ctx.display.r1, DisplayRegister::Blank);
    assert!(
        !r1_blank,
        "R1 (HAPO) darf nicht Blank sein wenn P30-Output gesetzt ist"
    );
}

// ---------------------------------------------------------------------------
// UC-2-D: P40 starten — PROG=40
// ---------------------------------------------------------------------------

#[test]
fn uc2_p40_start() {
    let mut state = test_utils::load_scenario_state(SCENARIO);

    // V37E40E — Programm 40 starten
    test_utils::press_keys(
        &mut state,
        &[
            KEY_VERB, KEY_3, KEY_7, KEY_ENTER, // V37E
            KEY_4, KEY_0, KEY_ENTER, // 40E
        ],
    );

    assert_eq!(
        state.ctx.display.major_mode.val(),
        40,
        "major_mode nach V37E40E erwartet 40"
    );
    assert_eq!(
        state.ctx.guidance.active_program,
        Some(40),
        "active_program nach V37E40E erwartet Some(40)"
    );
}

// ---------------------------------------------------------------------------
// UC-2-E: V16 N40 — Monitor-Modus aktiv
// ---------------------------------------------------------------------------

#[test]
fn uc2_v16_n40_monitor_mode() {
    let mut state = test_utils::load_scenario_state(SCENARIO);

    // P40 starten
    test_utils::press_keys(
        &mut state,
        &[KEY_VERB, KEY_3, KEY_7, KEY_ENTER, KEY_4, KEY_0, KEY_ENTER],
    );

    // V16 N40 ENTER — Monitor-Modus
    test_utils::press_keys(
        &mut state,
        &[
            KEY_VERB, KEY_1, KEY_6, KEY_ENTER, KEY_NOUN, KEY_4, KEY_0, KEY_ENTER,
        ],
    );

    assert_eq!(
        state.ctx.display.verb.val(),
        16,
        "Verb muss nach V16 N40 = 16 sein"
    );
    assert_eq!(
        state.ctx.display.noun.val(),
        40,
        "Noun muss nach V16 N40 = 40 sein"
    );
}

// ---------------------------------------------------------------------------
// UC-2-F: Energieerhaltung im Mondorbit (Sub-Task 8 Verifikation)
// ---------------------------------------------------------------------------

#[test]
fn uc2_moon_orbit_energy_stable() {
    use agc_physics::bodies::{Body, MU_MOON, R_MOON};
    use agc_physics::math::Vec3;
    use agc_physics::trajectory::{propagate, StepParams};

    let content = include_str!("../../docs/scenarios/tei_pre.scenario");
    let data = agc_sim::scenarios::parse_scenario_str(content).unwrap();
    let state0 = data.to_state();

    let params = StepParams {
        body: Body::Moon,
        thrust_accel: Vec3::ZERO,
        alpha_rad: 0.0,
        body_radius_m: R_MOON,
    };

    // Mondorbit-Periode: T = 2π√(r³/μ_moon)
    let r = state0.position.magnitude();
    let period = 2.0 * core::f64::consts::PI * (r * r * r / MU_MOON).sqrt();

    let final_state = propagate(state0, period, 1000, &params);

    fn orbital_energy_moon(pos: Vec3, vel: Vec3) -> f64 {
        0.5 * vel.magnitude_sq() - MU_MOON / pos.magnitude()
    }

    let e0 = orbital_energy_moon(state0.position, state0.velocity);
    let e1 = orbital_energy_moon(final_state.position, final_state.velocity);
    let rel_err = ((e1 - e0) / e0).abs();

    assert!(
        rel_err < 1e-2,
        "Mondorbit Energiefehler nach einer Periode zu groß: rel={rel_err:.2e} (erlaubt < 1 %)"
    );
}
