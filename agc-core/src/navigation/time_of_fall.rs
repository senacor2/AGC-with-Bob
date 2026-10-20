//! Zeit-bis-Aufprall Subroutine: Bisektion auf Kepler-Propagation.
//!
//! Berechnet die Zeit, bis ein frei fallender Körper einen Zielradius
//! unterschreitet. Algorithmus: binäre Suche auf [`kepler_step`].
//!
//! # Einheiten
//! Positionen in Metern, Geschwindigkeiten in m/s, Zeit in Sekunden,
//! Gravitationsparameter μ in m³/s².
//!
//! # AGC-Quelle
//! `Comanche055/TIME_OF_FREE_FALL.agc`, TFFCONIC routine.

use crate::alarm::AlarmState;
use crate::navigation::conic::kepler_step;
use crate::navigation::state_vector::StateVector;

/// Maximale Bisektion-Iterationen.
const BISECT_ITER: usize = 50;

/// Toleranz auf den Radius in Metern.
const RADIUS_TOL: f64 = 1.0;

/// Berechnet die freie Fallzeit bis zum Zielradius `r_terminal`.
///
/// Verwendet binäre Suche auf [`kepler_step`]: findet Δt so, dass
/// `|r(t + Δt)| ≈ r_terminal`. Die obere Suchgrenze ist die halbe
/// Kepler-Periode `T/2 = π·√(a³/μ)`, wobei die große Halbachse `a`
/// aus der Vis-Viva-Gleichung folgt.
///
/// # Parameter
/// - `sv`: Ausgangs-Zustandsvektor (Position in m, Geschwindigkeit in m/s)
/// - `r_terminal`: Zielradius in m (muss < |sv.position| sein)
/// - `mu`: Gravitationsparameter in m³/s²
/// - `alarm`: Alarmzustand (wird an `kepler_step` weitergegeben)
///
/// # Rückgabe
/// Fallzeit in Sekunden, oder `None` bei ungültiger Eingabe.
///
/// # Einschränkungen
/// - Gibt `None` zurück wenn `r_terminal >= |sv.position|`
/// - Gibt `None` zurück wenn `mu <= 0` oder `r_terminal <= 0`
/// - Setzt voraus, dass der Körper tatsächlich auf `r_terminal` zufällt
///
/// AGC source: `Comanche055/TIME_OF_FREE_FALL.agc`, TFFCONIC routine.
pub fn time_of_free_fall(
    sv: StateVector,
    r_terminal: f64,
    mu: f64,
    alarm: &mut AlarmState,
) -> Option<f64> {
    if mu <= 0.0 || r_terminal <= 0.0 {
        return None;
    }

    let r0 = sv.position.magnitude();
    if r_terminal >= r0 {
        return None;
    }

    // Große Halbachse aus Vis-Viva: a = μ·r0 / (2·μ − r0·v²)
    let v0_sq = sv.velocity.magnitude_sq();
    let denom = 2.0 * mu - r0 * v0_sq;
    if denom <= 0.0 {
        // Hyperbolische oder parabolische Bahn — Bisektion nicht anwendbar
        return None;
    }
    let a = mu * r0 / denom;
    if a <= 0.0 {
        return None;
    }

    // Obere Grenze: halbe Kepler-Periode
    let t_half_period = core::f64::consts::PI * libm::sqrt(a * a * a / mu);

    let mut t_lo = 0.0_f64;
    let mut t_hi = t_half_period;

    for _ in 0..BISECT_ITER {
        let t_mid = 0.5 * (t_lo + t_hi);
        let sv_mid = kepler_step(sv, t_mid, mu, alarm);
        let r_mid = sv_mid.position.magnitude();

        if libm::fabs(r_mid - r_terminal) < RADIUS_TOL {
            return Some(t_mid);
        }

        // r fällt monoton → r_mid > r_terminal bedeutet noch nicht weit genug gefallen
        if r_mid > r_terminal {
            t_lo = t_mid;
        } else {
            t_hi = t_mid;
        }
    }

    Some(0.5 * (t_lo + t_hi))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::alarm::AlarmState;
    use crate::navigation::state_vector::StateVector;
    use crate::types::{Met, Vec3};

    const MU_EARTH: f64 = 3.986_004_418e14;
    const R_EARTH: f64 = 6_371_000.0;

    #[test]
    fn free_fall_from_leo_to_surface() {
        let r0 = 6_678_000.0_f64;
        let v_circ = libm::sqrt(MU_EARTH / r0);
        let sv = StateVector::new(
            Vec3::new(r0, 0.0, 0.0),
            Vec3::new(0.0, v_circ, 0.0),
            Met::from_seconds(0.0),
        );
        let mut alarm = AlarmState::new();
        let result = time_of_free_fall(sv, R_EARTH, MU_EARTH, &mut alarm);
        assert!(result.is_some(), "Erwartet Some, bekam None");
        let t = result.unwrap();
        assert!(
            t > 1000.0 && t < 4000.0,
            "Fallzeit {t:.1} s außerhalb des plausiblen Bereichs [1000, 4000]"
        );
    }

    #[test]
    fn invalid_terminal_radius_above_orbit() {
        let r0 = 6_678_000.0_f64;
        let v_circ = libm::sqrt(MU_EARTH / r0);
        let sv = StateVector::new(
            Vec3::new(r0, 0.0, 0.0),
            Vec3::new(0.0, v_circ, 0.0),
            Met::from_seconds(0.0),
        );
        let mut alarm = AlarmState::new();
        // r_terminal > r0 → None
        let result = time_of_free_fall(sv, r0 + 1000.0, MU_EARTH, &mut alarm);
        assert!(result.is_none(), "Erwartet None bei r_terminal > r0");
    }

    #[test]
    fn free_fall_zero_mu_returns_none() {
        let r0 = 6_678_000.0_f64;
        let sv = StateVector::new(
            Vec3::new(r0, 0.0, 0.0),
            Vec3::new(0.0, 7700.0, 0.0),
            Met::from_seconds(0.0),
        );
        let mut alarm = AlarmState::new();
        let result = time_of_free_fall(sv, R_EARTH, 0.0, &mut alarm);
        assert!(result.is_none(), "Erwartet None bei mu = 0");
    }
}
