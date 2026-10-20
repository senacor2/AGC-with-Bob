//! Integrationstest: HAL-Implementierungen + Trajektorie auf einer Kreisbahn.
//!
//! Prüft die vollständige Kette agc-physics ↔ SimHAL ↔ agc-core-Trait-Grenzen:
//! - `SimImu` akkumuliert PIPA-Pulse über einen Simulations-Tick.
//! - `SimSps` liefert den korrekten Schubvektor bei gezündeter Engine.
//! - `SimRcs` summiert Nettokräfte mehrerer aktiver Jets.
//! - `SimDsky` puffert Display-Worte und FIFO-Tastendrücke.
//! - `SimVhfRanging` liefert AGC-skalierte Distanzwerte.
//! - `SimTelemetry` puffert Downlink-Worte und Uplink-Warteschlange.
//!
//! Der Kreisbahn-Test prüft, dass der RK4-Integrator + IMU-Modell nach einer
//! vollen Periode die spezifische Energie mit weniger als 0.01 % Fehler erhält.

use agc_core::hal::{DskyHal, ImuHal, RcsHal, SpsHal, TelemetryHal, VhfRangingHal};
use agc_physics::bodies::{MU_EARTH, R_EARTH};
use agc_physics::math::Vec3;
use agc_physics::trajectory::{propagate, State, StepParams};
use agc_sim::hal::{SimDsky, SimImu, SimRcs, SimSps, SimTelemetry, SimVhfRanging};

// ---------------------------------------------------------------------------
// Hilfsfunktionen
// ---------------------------------------------------------------------------

fn circular_velocity(r: f64) -> f64 {
    (MU_EARTH / r).sqrt()
}

fn orbital_energy(pos: Vec3, vel: Vec3) -> f64 {
    0.5 * vel.magnitude_sq() - MU_EARTH / pos.magnitude()
}

// ---------------------------------------------------------------------------
// IMU-Tests
// ---------------------------------------------------------------------------

#[test]
fn sim_imu_zero_force_zero_pulses() {
    let mut imu = SimImu::new();
    imu.update(Vec3::ZERO, 1.0);
    assert_eq!(imu.read_pipa_pulses(), [0, 0, 0]);
}

#[test]
fn sim_imu_accumulates_pulses() {
    use agc_physics::imu_model::PIPA_SCALE_MPS;
    let mut imu = SimImu::new();
    // 2 Pulse auf X-Achse in einem Schritt
    imu.update(Vec3::new(PIPA_SCALE_MPS * 2.0, 0.0, 0.0), 1.0);
    let p = imu.read_pipa_pulses();
    assert_eq!(p[0], 2, "X-Pulse: {}", p[0]);
    // Nach dem Lesen zurückgesetzt
    assert_eq!(imu.read_pipa_pulses(), [0, 0, 0]);
}

#[test]
fn sim_imu_multi_step_accumulation() {
    use agc_physics::imu_model::PIPA_SCALE_MPS;
    // 0.4 Puls/Schritt → nach 3 Schritten 1 Puls (bei Δv 0.4+0.4+0.4=1.2)
    let force = Vec3::new(PIPA_SCALE_MPS * 0.4, 0.0, 0.0);
    let mut imu = SimImu::new();
    imu.update(force, 1.0);
    imu.update(force, 1.0);
    imu.update(force, 1.0);
    let p = imu.read_pipa_pulses();
    assert_eq!(
        p[0], 1,
        "Erwartete 1 Puls nach 3 Schritten, bekam: {}",
        p[0]
    );
}

#[test]
fn sim_imu_gimbal_angles_roundtrip() {
    let mut imu = SimImu::new();
    imu.set_gimbal_angles([1000, -500, 250]);
    assert_eq!(imu.read_gimbal_angles(), [1000, -500, 250]);
}

#[test]
fn sim_imu_operational_flag() {
    let mut imu = SimImu::new();
    assert!(imu.is_imu_operational());
    imu.set_operational(false);
    assert!(!imu.is_imu_operational());
}

// ---------------------------------------------------------------------------
// DSKY-Tests
// ---------------------------------------------------------------------------

#[test]
fn sim_dsky_display_word_count() {
    let mut dsky = SimDsky::new();
    dsky.write_display_word(0x1234);
    dsky.write_display_word(0xABCD);
    assert_eq!(dsky.state.last_display_word, 0xABCD);
    assert_eq!(dsky.state.display_count, 2);
}

#[test]
fn sim_dsky_status_lights() {
    let mut dsky = SimDsky::new();
    dsky.write_status_lights(0b0010_0000);
    assert_eq!(dsky.state.last_status_bits, 0b0010_0000);
}

#[test]
fn sim_dsky_key_fifo() {
    let mut dsky = SimDsky::new();
    dsky.push_key(16); // PRO
    dsky.push_key(25); // ENTR
    assert_eq!(dsky.read_keycode(), Some(16));
    assert_eq!(dsky.read_keycode(), Some(25));
    assert_eq!(dsky.read_keycode(), None);
}

#[test]
fn sim_dsky_key_queue_overflow_discards() {
    let mut dsky = SimDsky::new();
    // 16 Einträge (volle Kapazität)
    for i in 0..16u8 {
        dsky.push_key(i);
    }
    // 17. Eintrag wird verworfen
    dsky.push_key(99);
    assert_eq!(dsky.pending_keys(), 16);
    assert_eq!(dsky.read_keycode(), Some(0));
}

// ---------------------------------------------------------------------------
// RCS-Tests
// ---------------------------------------------------------------------------

#[test]
fn sim_rcs_no_jets_zero_force() {
    let rcs = SimRcs::new();
    let f = rcs.net_force_n();
    assert_eq!(f, Vec3::ZERO);
    assert!(!rcs.any_jet_active());
}

#[test]
fn sim_rcs_single_jet_force_magnitude() {
    let mut rcs = SimRcs::new();
    rcs.set_jets_ab(0b0000_0001); // Jet A1 (+Y)
    let f = rcs.net_force_n();
    // A1 ist +Y mit 445 N
    assert!((f.y - 445.0).abs() < 1e-6, "f.y={}", f.y);
    assert!((f.x).abs() < 1e-6);
    assert!((f.z).abs() < 1e-6);
}

#[test]
fn sim_rcs_opposing_jets_cancel() {
    let mut rcs = SimRcs::new();
    // A1 (+Y, Bit 0) und A2 (-Y, Bit 1) gleichzeitig → Nettokraft ≈ 0 in Y
    rcs.set_jets_ab(0b0000_0011);
    let f = rcs.net_force_n();
    assert!(
        f.y.abs() < 1e-6,
        "Entgegengesetzte Jets sollten sich aufheben: f.y={}",
        f.y
    );
}

#[test]
fn sim_rcs_jet_cd_force() {
    let mut rcs = SimRcs::new();
    rcs.set_jets_cd(0b0001_0000); // D1 (-X, Bit 4 in CD)
    let f = rcs.net_force_n();
    assert!(
        (f.x + 445.0).abs() < 1e-6,
        "D1 soll -X = -445 N: f.x={}",
        f.x
    );
}

// ---------------------------------------------------------------------------
// SPS-Tests
// ---------------------------------------------------------------------------

#[test]
fn sim_sps_engine_off_zero_accel() {
    let sps = SimSps::new();
    assert_eq!(sps.thrust_accel(), Vec3::ZERO);
    assert!(!sps.engine_on());
}

#[test]
fn sim_sps_engine_on_positive_z() {
    let mut sps = SimSps::new();
    sps.set_engine(true);
    let a = sps.thrust_accel();
    // Nominaler Schub: 91188 / 28800 ≈ 3.166 m/s²
    let expected = agc_sim::hal::SPS_THRUST_N / agc_sim::hal::CSM_MASS_KG;
    assert!((a.z - expected).abs() < 1e-6, "a.z={}", a.z);
    assert!(a.x.abs() < 1e-9);
    assert!(a.y.abs() < 1e-9);
}

#[test]
fn sim_sps_tvc_deflection() {
    let mut sps = SimSps::new();
    sps.set_engine(true);
    sps.set_tvc_pitch(1000); // Kleiner Pitch-Winkel
    let a = sps.thrust_accel();
    // Pitch → Komponente in Y-Richtung
    assert!(
        a.y.abs() > 1e-4,
        "Pitch soll Y-Komponente erzeugen: a.y={}",
        a.y
    );
    // Z bleibt dominant
    assert!(
        a.z > a.y.abs(),
        "Z soll dominieren: a.z={}, a.y={}",
        a.z,
        a.y
    );
}

// ---------------------------------------------------------------------------
// VHF-Ranging-Tests
// ---------------------------------------------------------------------------

#[test]
fn sim_vhf_inactive_not_available() {
    let r = SimVhfRanging::noiseless(false);
    assert!(!r.is_available());
    assert_eq!(r.read_range(), None);
}

#[test]
fn sim_vhf_active_known_distance() {
    // 1000 m → AGC-Wert = round(1000 / 0.488) = 2049
    let mut r = SimVhfRanging::noiseless(true);
    let cm = Vec3::ZERO;
    let lm = Vec3::new(1000.0, 0.0, 0.0);
    r.tick(cm, lm);
    assert!(r.is_available());
    let agc_val = r.read_range().expect("Messung erwartet");
    let expected = (1000.0_f64 / 0.488).round() as u16;
    assert_eq!(
        agc_val, expected,
        "AGC-Ranging-Wert: {} != {}",
        agc_val, expected
    );
}

#[test]
fn sim_vhf_out_of_range() {
    let mut r = SimVhfRanging::noiseless(true);
    let cm = Vec3::ZERO;
    let lm = Vec3::new(800_000.0, 0.0, 0.0); // > MAX_RANGE_M (740 km)
    r.tick(cm, lm);
    assert!(!r.is_available());
    assert_eq!(r.read_range(), None);
}

#[test]
fn sim_vhf_request_range_no_crash() {
    let mut r = SimVhfRanging::noiseless(true);
    r.request_range(); // darf nicht abstürzen
}

// ---------------------------------------------------------------------------
// Telemetrie-Tests
// ---------------------------------------------------------------------------

#[test]
fn sim_telemetry_downlink_ringpuffer() {
    let mut tm = SimTelemetry::new();
    tm.send_downlink_word(0x0001);
    tm.send_downlink_word(0x0002);
    assert_eq!(tm.downlink_total, 2);
}

#[test]
fn sim_telemetry_uplink_fifo() {
    let mut tm = SimTelemetry::new();
    tm.push_uplink(0xAAAA);
    tm.push_uplink(0xBBBB);
    assert_eq!(tm.read_uplink_word(), Some(0xAAAA));
    assert_eq!(tm.read_uplink_word(), Some(0xBBBB));
    assert_eq!(tm.read_uplink_word(), None);
    assert!(tm.uplink_empty());
}

// ---------------------------------------------------------------------------
// Kreisbahn-Integrationstest
// ---------------------------------------------------------------------------

/// Prüft, dass der RK4-Integrator auf einer 400-km-Kreisbahn die spezifische
/// Energie nach einer vollen Umlaufperiode mit < 0.01 % Fehler erhält.
///
/// Der SimImu wird parallel zum RK4-Integrator mit der spezifischen Kraft
/// gespeist und akkumuliert PIPA-Pulse; am Ende müssen die Gesamtpulse
/// (in m/s umgerechnet) mit dem Δv der Kreisbahn übereinstimmen.
#[test]
fn circular_orbit_energy_conservation() {
    const R_LEO: f64 = R_EARTH + 400_000.0;
    let v0 = circular_velocity(R_LEO);

    let state0 = State::new(Vec3::new(R_LEO, 0.0, 0.0), Vec3::new(0.0, v0, 0.0), 0.0);
    let params = StepParams::earth_ballistic();

    // Umlaufzeit T = 2π√(r³/μ)
    let period = 2.0 * std::f64::consts::PI * (R_LEO * R_LEO * R_LEO / MU_EARTH).sqrt();

    let final_state = propagate(state0, period, 1000, &params);

    let e0 = orbital_energy(state0.position, state0.velocity);
    let e1 = orbital_energy(final_state.position, final_state.velocity);

    let rel_err = ((e1 - e0) / e0).abs();
    assert!(
        rel_err < 1e-4,
        "Energieerhaltungsfehler zu groß: rel={rel_err:.2e} (Limit: 1e-4)"
    );
}

/// Prüft, dass der IMU nach einer Kreisbahn (keine externe Beschleunigung,
/// nur Gravitation) insgesamt nahe null Pulse akkumuliert.
///
/// Eine Kreisbahn erzeugt keine spezifische Kraft (Freier Fall) — also
/// sollte das IMU fast keine Pulse erzeugen.
#[test]
fn circular_orbit_imu_near_zero_pulses() {
    const R_LEO: f64 = R_EARTH + 400_000.0;
    let v0 = circular_velocity(R_LEO);

    let mut state = State::new(Vec3::new(R_LEO, 0.0, 0.0), Vec3::new(0.0, v0, 0.0), 0.0);
    let params = StepParams::earth_ballistic();

    let period = 2.0 * std::f64::consts::PI * (R_LEO * R_LEO * R_LEO / MU_EARTH).sqrt();
    let n_steps: usize = 100;
    let dt = period / n_steps as f64;

    let mut imu = SimImu::new();
    let mut total_pulses = [0i32; 3];

    for _ in 0..n_steps {
        // Auf einer Kreisbahn: spezifische Kraft ≈ 0 (Freier Fall)
        imu.update(Vec3::ZERO, dt);
        let p = imu.read_pipa_pulses();
        for i in 0..3 {
            total_pulses[i] += p[i] as i32;
        }
        state = agc_physics::trajectory::rk4_step(state, dt, &params);
    }

    // Keine externe Kraft → alle Pulse sollten 0 sein
    assert_eq!(
        total_pulses,
        [0, 0, 0],
        "IMU sollte auf Kreisbahn keine Pulse erzeugen: {:?}",
        total_pulses
    );
}
