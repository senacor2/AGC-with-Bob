//! Trajektorie-Propagation: RK4-Integrator mit Gravitationsmodell und Aerodynamik.
//!
//! Integriert die Bewegungsgleichungen eines Raumfahrzeugs unter dem Einfluss von:
//! - Zwöikörper-Gravitation + J2-Störterm (immer aktiv)
//! - Aerodynamischen Kräften (Drag + Lift, nur wenn die Kapsel in der Atmosphäre ist)
//! - Externer Schubkraft (SPS, RCS — als konstante Beschleunigung pro Zeitschritt)
//!
//! Der Integrator verwendet ein klassisches Runge-Kutta-Verfahren 4. Ordnung (RK4).
//! Für Zeitschritte ≲ 1 s ist die globale Fehlerordnung O(Δt⁴).
//!
//! # Einheiten
//! - Position: m (ECI J2000)
//! - Geschwindigkeit: m/s (ECI J2000)
//! - Zeit: Sekunden (MET)
//! - Schub-/Aero-Beschleunigung: m/s²
//!
//! # AGC-Quelle
//! `Comanche055/ORBITAL_INTEGRATION.agc`, ORBITING/KEPPREP-Routinen.
//! Der RK4-Integrator ersetzt den AGC-Kepler-Propagator durch ein allgemeineres
//! Verfahren, das Aerodynamik direkt in die Zustandsableitung einschließt.

use crate::aero;
use crate::atmosphere;
use crate::bodies::Body;
use crate::gravity::gravity;
use crate::math::Vec3;

// ---------------------------------------------------------------------------
// Zustandsvektor
// ---------------------------------------------------------------------------

/// Physikalischer Zustandsvektor des Raumfahrzeugs.
///
/// Verwendet als In- und Output des RK4-Integrators.
///
/// # Einheiten
/// - `position`: m (ECI J2000)
/// - `velocity`: m/s (ECI J2000)
/// - `time_s`:   Sekunden (MET)
#[derive(Clone, Copy, Debug)]
pub struct State {
    /// Positionsvektor in ECI (m).
    pub position: Vec3,
    /// Geschwindigkeitsvektor in ECI (m/s).
    pub velocity: Vec3,
    /// Mission Elapsed Time (s).
    pub time_s: f64,
}

impl State {
    /// Erstellt einen neuen Zustandsvektor.
    pub const fn new(position: Vec3, velocity: Vec3, time_s: f64) -> Self {
        Self {
            position,
            velocity,
            time_s,
        }
    }
}

// ---------------------------------------------------------------------------
// Integrationsparameter
// ---------------------------------------------------------------------------

/// Parameter für einen RK4-Integrationsschritt.
///
/// Zusammengefasst, damit der Integrator keine langen Parameterlisten benötigt.
#[derive(Clone, Copy, Debug)]
pub struct StepParams {
    /// Zentralkörper (Erde oder Mond).
    pub body: Body,
    /// Externe Beschleunigung durch Triebwerke (m/s², ECI), z. B. SPS-Schub.
    pub thrust_accel: Vec3,
    /// Anstellwinkel der Kapsel (Radiant) — für Aerodynamik.
    ///
    /// `0.0` wenn kein aerodynamischer Auftrieb gewünscht oder außerhalb der Atm.
    pub alpha_rad: f64,
    /// Radius des Zentralkörpers (m) — Oberfläche bei `altitude = |r| − radius`.
    pub body_radius_m: f64,
}

impl StepParams {
    /// Standardparameter: Erde, kein Schub, kein Anstellwinkel.
    pub const fn earth_ballistic() -> Self {
        Self {
            body: Body::Earth,
            thrust_accel: Vec3::ZERO,
            alpha_rad: 0.0,
            body_radius_m: crate::bodies::R_EARTH,
        }
    }
}

// ---------------------------------------------------------------------------
// Zustandsableitung
// ---------------------------------------------------------------------------

/// Berechnet die Zustandsableitung (ṙ, v̇) für einen gegebenen Zustand.
///
/// v̇ = a_grav(r) + a_aero(r, v) + a_thrust
///
/// Aerodynamik wird nur aktiviert wenn die Höhe < 120 km beträgt
/// (oberhalb vernachlässigbar).
fn deriv(pos: Vec3, vel: Vec3, params: &StepParams) -> (Vec3, Vec3) {
    // Gravitationsbeschleunigung (Zweikörper + J2)
    let a_grav = gravity(pos, params.body);

    // Aerodynamische Beschleunigung
    let r = pos.magnitude();
    let altitude_m = r - params.body_radius_m;
    let a_aero = if altitude_m < 120_000.0 && altitude_m > -1_000.0 {
        aero_accel(pos, vel, altitude_m, params.alpha_rad)
    } else {
        Vec3::ZERO
    };

    let a_total = a_grav + a_aero + params.thrust_accel;

    // ṙ = v, v̇ = a
    (vel, a_total)
}

/// Aerodynamische Beschleunigung in ECI (m/s²).
///
/// Drag wirkt entgegen der Flugrichtung.  Lift wirkt senkrecht zur Flugrichtung
/// in der Ebene, die durch v und den Körperpol definiert wird (vereinfacht:
/// Lift zeigt radial nach außen wenn α > 0).
fn aero_accel(pos: Vec3, vel: Vec3, altitude_m: f64, alpha_rad: f64) -> Vec3 {
    let v_mag = vel.magnitude();
    if v_mag < 1.0 {
        return Vec3::ZERO;
    }

    let rho = atmosphere::density(altitude_m);
    let c_sound = atmosphere::speed_of_sound(altitude_m);
    let mach = v_mag / c_sound;
    let dyn_pressure = 0.5 * rho * v_mag * v_mag;

    let (f_drag, f_lift) = aero::aero_forces(dyn_pressure, mach, alpha_rad);

    // Fahrzeugmasse (Apollo CM ≈ 5 900 kg mit Restpropellant, Wiedereintritt)
    const CM_MASS_KG: f64 = 5_900.0;

    // Drag-Beschleunigung: entgegen Flugrichtung
    let v_hat = vel.scale(1.0 / v_mag);
    let a_drag = v_hat.scale(-f_drag / CM_MASS_KG);

    // Lift-Beschleunigung: senkrecht zu v in der (v, r̂)-Ebene
    let r_hat = match pos.normalize() {
        Some(h) => h,
        None => return a_drag,
    };
    // Komponente von r̂ senkrecht zu v̂
    let r_perp = r_hat - v_hat.scale(r_hat.dot(v_hat));
    let r_perp_mag = r_perp.magnitude();
    let a_lift = if r_perp_mag > 1e-9 {
        r_perp.scale(f_lift / (CM_MASS_KG * r_perp_mag))
    } else {
        Vec3::ZERO
    };

    a_drag + a_lift
}

// ---------------------------------------------------------------------------
// RK4-Integrator
// ---------------------------------------------------------------------------

/// Führt einen einzelnen RK4-Integrationsschritt durch.
///
/// Integriert die Bewegungsgleichungen von `state` um `dt` Sekunden.
///
/// # Parameter
/// - `state`:  Aktueller Zustandsvektor
/// - `dt`:     Zeitschritt in Sekunden (typisch 0.1 … 10 s)
/// - `params`: Integrationsparameter (Zentralkörper, Schub, Aerodynamik)
///
/// # Rückgabe
/// Neuer Zustandsvektor nach dem Zeitschritt.
///
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc`, ORBITING routine.
pub fn rk4_step(state: State, dt: f64, params: &StepParams) -> State {
    let r0 = state.position;
    let v0 = state.velocity;

    // k1
    let (dr1, dv1) = deriv(r0, v0, params);

    // k2
    let r2 = r0 + dr1.scale(dt * 0.5);
    let v2 = v0 + dv1.scale(dt * 0.5);
    let (dr2, dv2) = deriv(r2, v2, params);

    // k3
    let r3 = r0 + dr2.scale(dt * 0.5);
    let v3 = v0 + dv2.scale(dt * 0.5);
    let (dr3, dv3) = deriv(r3, v3, params);

    // k4
    let r4 = r0 + dr3.scale(dt);
    let v4 = v0 + dv3.scale(dt);
    let (dr4, dv4) = deriv(r4, v4, params);

    // Gewichtetes Mittel
    let r_new = r0 + (dr1 + dr2.scale(2.0) + dr3.scale(2.0) + dr4).scale(dt / 6.0);
    let v_new = v0 + (dv1 + dv2.scale(2.0) + dv3.scale(2.0) + dv4).scale(dt / 6.0);

    State {
        position: r_new,
        velocity: v_new,
        time_s: state.time_s + dt,
    }
}

/// Propagiert einen Zustandsvektor über eine Gesamtdauer mit mehreren RK4-Schritten.
///
/// Unterteilt `total_dt` in `n_steps` gleich große Teilschritte.
///
/// # Parameter
/// - `state`:    Ausgangszustand
/// - `total_dt`: Gesamtzeitdauer in Sekunden
/// - `n_steps`:  Anzahl RK4-Teilschritte (≥ 1)
/// - `params`:   Integrationsparameter
///
/// # Rückgabe
/// Zustand nach `total_dt` Sekunden.
pub fn propagate(state: State, total_dt: f64, n_steps: usize, params: &StepParams) -> State {
    if n_steps == 0 {
        return state;
    }
    let dt = total_dt / n_steps as f64;
    let mut s = state;
    for _ in 0..n_steps {
        s = rk4_step(s, dt, params);
    }
    s
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bodies::{MU_EARTH, R_EARTH};

    /// Kreisbahn-Radius für eine LEO-Bahn (400 km Höhe).
    const R_LEO: f64 = R_EARTH + 400_000.0;

    /// Kreisbahngeschwindigkeit bei R_LEO.
    fn circular_velocity(r: f64) -> f64 {
        libm::sqrt(MU_EARTH / r)
    }

    /// Spezifische mechanische Energie einer Keplerbahn (J/kg).
    fn orbital_energy(pos: Vec3, vel: Vec3) -> f64 {
        0.5 * vel.magnitude_sq() - MU_EARTH / pos.magnitude()
    }

    #[test]
    fn circular_orbit_energy_conservation() {
        // Kreisbahn in 400 km — Energie muss nach einer vollen Periode konstant bleiben.
        let v0 = circular_velocity(R_LEO);
        let state = State::new(Vec3::new(R_LEO, 0.0, 0.0), Vec3::new(0.0, v0, 0.0), 0.0);
        let params = StepParams::earth_ballistic();

        // Umlaufzeit: T = 2π√(r³/μ) ≈ 5559 s für 400 km
        let period = 2.0 * core::f64::consts::PI * libm::sqrt(R_LEO * R_LEO * R_LEO / MU_EARTH);
        let n_steps = 1000usize;

        let final_state = propagate(state, period, n_steps, &params);

        let e0 = orbital_energy(state.position, state.velocity);
        let e1 = orbital_energy(final_state.position, final_state.velocity);

        // Relative Energieerhaltung besser als 1e-4 (0.01 %)
        let rel_err = ((e1 - e0) / e0).abs();
        assert!(rel_err < 1e-4, "Energiefehler zu groß: rel={rel_err:.2e}");
    }

    #[test]
    fn circular_orbit_radius_conservation() {
        // Nach einer Periode muss der Abstand zur Erde ≈ konstant sein.
        let v0 = circular_velocity(R_LEO);
        let state = State::new(Vec3::new(R_LEO, 0.0, 0.0), Vec3::new(0.0, v0, 0.0), 0.0);
        let params = StepParams::earth_ballistic();
        let period = 2.0 * core::f64::consts::PI * libm::sqrt(R_LEO * R_LEO * R_LEO / MU_EARTH);
        let final_state = propagate(state, period, 2000, &params);

        let r0 = state.position.magnitude();
        let r1 = final_state.position.magnitude();
        let rel_err = ((r1 - r0) / r0).abs();
        assert!(rel_err < 1e-3, "Radiusfehler: {rel_err:.2e}");
    }

    #[test]
    fn rk4_single_step_zero_dt() {
        let state = State::new(
            Vec3::new(R_LEO, 0.0, 0.0),
            Vec3::new(0.0, circular_velocity(R_LEO), 0.0),
            0.0,
        );
        let params = StepParams::earth_ballistic();
        let result = rk4_step(state, 0.0, &params);
        // Bei dt=0 ändert sich nichts
        assert!((result.position.x - state.position.x).abs() < 1e-9);
    }
}
