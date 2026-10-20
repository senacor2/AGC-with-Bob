//! Gravitationsmodell: Erde (J2) und Mond (Punkt).
//!
//! Liefert den Beschleunigungsvektor für einen Körper im Gravitationsfeld
//! der Erde oder des Mondes.  Für die Erde wird der J2-Zonalkoeffizient
//! berücksichtigt; der Mond wird als Punktmasse modelliert.
//!
//! # Einheiten
//! Eingabe: Positionsvektor in m (ECI J2000).
//! Ausgabe: Beschleunigungsvektor in m/s².
//!
//! # Referenz
//! Vallado, *Fundamentals of Astrodynamics and Applications*, 4th ed., §9.6.2.
//!
//! # AGC-Quelle
//! `Comanche055/ORBITAL_INTEGRATION.agc`, KEPPREP routine.

use crate::bodies::Body;
use crate::math::Vec3;

// ---------------------------------------------------------------------------
// Reine Zweikörper-Beschleunigung
// ---------------------------------------------------------------------------

/// Gravitationsbeschleunigung des Zweikörper-Problems (m/s²).
///
/// `a = −μ / r³ · r⃗`
///
/// # Parameter
/// - `pos`:  Positionsvektor in ECI (m)
/// - `mu`:   Gravitationsparameter (m³/s²)
///
/// # Rückgabe
/// Beschleunigungsvektor in m/s².  Bei `r = 0` wird Nullvektor zurückgegeben.
///
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc`, KEPPREP routine.
#[inline]
pub fn two_body(pos: Vec3, mu: f64) -> Vec3 {
    let r2 = pos.magnitude_sq();
    if r2 == 0.0 {
        return Vec3::ZERO;
    }
    let r = libm::sqrt(r2);
    let r3 = r2 * r;
    pos.scale(-mu / r3)
}

// ---------------------------------------------------------------------------
// J2-Störbeschleunigung
// ---------------------------------------------------------------------------

/// J2-Störbeschleunigung in kartesischen ECI-Koordinaten (m/s²).
///
/// Berechnet den Beitrag des ersten Zonalkoeffizienten J2 zur
/// Gravitationsbeschleunigung.
///
/// ## Formel (Vallado §9.6.2, Gl. 9-31)
/// ```text
/// k  = (3/2) · J2 · μ · R_eq² / r⁵
/// aₓ = k · x · (5·z²/r² − 1)
/// aᵧ = k · y · (5·z²/r² − 1)
/// a_z = k · z · (5·z²/r² − 3)
/// ```
///
/// # Parameter
/// - `pos`:  Positionsvektor in m (ECI J2000)
/// - `body`: Zentralkörper
///
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc`, KEPPREP routine.
pub fn j2_perturbation(pos: Vec3, body: Body) -> Vec3 {
    let r2 = pos.magnitude_sq();
    if r2 == 0.0 {
        return Vec3::ZERO;
    }
    let r = libm::sqrt(r2);
    let r5 = r2 * r2 * r;

    let mu = body.mu();
    let j2 = body.j2();
    let req = body.radius();

    let k = 1.5 * j2 * mu * req * req / r5;
    let z2_r2 = pos.z * pos.z / r2;

    Vec3::new(
        k * pos.x * (5.0 * z2_r2 - 1.0),
        k * pos.y * (5.0 * z2_r2 - 1.0),
        k * pos.z * (5.0 * z2_r2 - 3.0),
    )
}

// ---------------------------------------------------------------------------
// Kombinierte Beschleunigung
// ---------------------------------------------------------------------------

/// Gesamte Gravitationsbeschleunigung (Zweikörper + J2) in m/s².
///
/// Für den Mond wird J2 mit berücksichtigt, aber sein Einfluss ist
/// verglichen mit der Erde sehr klein.
///
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc`, KEPPREP routine.
pub fn gravity(pos: Vec3, body: Body) -> Vec3 {
    let a_tb = two_body(pos, body.mu());
    let a_j2 = j2_perturbation(pos, body);
    a_tb + a_j2
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bodies::{MU_EARTH, R_EARTH};

    #[test]
    fn two_body_surface_magnitude() {
        // Auf der Erdoberfläche: |a| ≈ 9.82 m/s²
        let r = R_EARTH;
        let pos = Vec3::new(r, 0.0, 0.0);
        let a = two_body(pos, MU_EARTH);
        let mag = a.magnitude();
        // g = μ/r² ≈ 9.820 m/s²
        let expected = MU_EARTH / (r * r);
        assert!(
            (mag - expected).abs() < 1e-3,
            "mag={mag}, expected={expected}"
        );
    }

    #[test]
    fn two_body_zero_returns_zero() {
        let a = two_body(Vec3::ZERO, MU_EARTH);
        assert_eq!(a, Vec3::ZERO);
    }

    #[test]
    fn j2_equatorial_x_only() {
        // Auf dem Äquator zeigt a_J2 in −x-Richtung (zusätzliche Abbremskomponente)
        let pos = Vec3::new(R_EARTH + 400_000.0, 0.0, 0.0);
        let a = j2_perturbation(pos, Body::Earth);
        // z=0 → a_z = 0, aₓ < 0 (zusätzliche Attraktion)
        assert!(a.x < 0.0, "Erwarte negative x-Komponente, a={a:?}");
        assert!(a.z.abs() < 1e-15);
    }
}
