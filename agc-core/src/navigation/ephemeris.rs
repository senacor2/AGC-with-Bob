//! Niederpräzisions-Ephemeris für Mond und Sonne (Vallado §5.4).
//!
//! Berechnet ECI-Positionen (J2000-ähnlich) für Mond und Sonne aus der
//! Ground Elapsed Time (GET) des AGC in Centisekunden.
//!
//! # Algorithmus
//! Vallado, "Fundamentals of Astrodynamics and Applications", 4. Aufl., §5.4:
//! Low-Precision Analytical Ephemeris (genauigkeit ~0.3° Winkel).
//!
//! # Koordinatensystem
//! ECI (Earth-Centered Inertial), J2000-Äquinox.
//! Alle Positionen in Metern (m).
//!
//! # no_std
//! Alle Winkelfunktionen kommen aus `libm` — kein `std`.

use crate::types::Vec3;

/// Umrechnungsfaktor: Grad → Radiant.
const DEG_TO_RAD: f64 = core::f64::consts::PI / 180.0;

/// Astronomische Einheit in Metern (IAU 2012).
const AU_M: f64 = 1.495_978_707e11;

/// Mittlerer Mondabstand in Kilometern (für Referenz-Plausibilitätsprüfung).
#[allow(dead_code)]
const MOON_MEAN_DIST_KM: f64 = 384_400.0;

// ---------------------------------------------------------------------------
// Interne Hilfsfunktion
// ---------------------------------------------------------------------------

/// Normalisiert einen Winkel (Grad) auf [0°, 360°).
#[inline]
fn normalize_deg(deg: f64) -> f64 {
    let r = deg % 360.0;
    if r < 0.0 {
        r + 360.0
    } else {
        r
    }
}

/// Konvertiert ekliptikale Koordinaten (λ, β, Δ) in ECI-kartesische Koordinaten.
///
/// # Parameter
/// - `lambda_deg`: Ekliptikale Länge [Grad]
/// - `beta_deg`:   Ekliptikale Breite [Grad]
/// - `dist_m`:     Abstand vom Erdmittelpunkt [m]
/// - `t`:          Julianische Jahrhunderte seit J2000
///
/// # Rückgabe
/// ECI-Positionsvektor [m].
fn ecliptic_to_eci(lambda_deg: f64, beta_deg: f64, dist_m: f64, t: f64) -> Vec3 {
    // Schiefe der Ekliptik ε [Grad]: Vallado Gl. 5-20
    // ε = 23.439° − 0.0130°·t  (Langzeit-Variation vernachlässigt)
    let eps_deg = 23.439 - 0.0130 * t;
    let eps = eps_deg * DEG_TO_RAD;

    let lam = lambda_deg * DEG_TO_RAD;
    let beta = beta_deg * DEG_TO_RAD;

    // Ekliptikale kartesische Koordinaten
    let x_ecl = dist_m * libm::cos(beta) * libm::cos(lam);
    let y_ecl = dist_m * libm::cos(beta) * libm::sin(lam);
    let z_ecl = dist_m * libm::sin(beta);

    // Rotation um x-Achse um -ε: ECI = R_x(-ε) · [x, y, z]_ecl
    let cos_e = libm::cos(eps);
    let sin_e = libm::sin(eps);

    Vec3::new(
        x_ecl,
        cos_e * y_ecl - sin_e * z_ecl,
        sin_e * y_ecl + cos_e * z_ecl,
    )
}

// ---------------------------------------------------------------------------
// Öffentliche API
// ---------------------------------------------------------------------------

/// Berechnet die ECI-Position des Mondes aus der AGC-Ground-Elapsed-Time.
///
/// Verwendet Valladoʼs Low-Precision Lunar Ephemeris (§5.4, Table 5-3).
/// Genauigkeit: ≈ 0.3° in Winkel, ≈ 1–2 % in Abstand.
///
/// # Einheiten
/// - Eingabe `get_centisecs`: AGC-Zeit in Centisekunden seit GET=0 [cs]
/// - Ausgabe: ECI-Positionsvektor [m]
///
/// # AGC-Quelle
/// Entspricht dem Ephemeris-Einlesen in `Comanche055/PLANETARY_INERTIAL_ORIENTATION.agc`.
///
/// # Literatur
/// Vallado, "Fundamentals of Astrodynamics and Applications", 4. Aufl., §5.4.
pub fn moon_position(get_centisecs: f64) -> Vec3 {
    // Julianische Jahrhunderte seit J2000.
    // 1 JC = 36 525 Tage × 86 400 s/d × 100 cs/s = 315 576 000 000 cs
    let t = get_centisecs / 315_576_000_000.0;

    // Fundamentalargumente [Grad] — Vallado Table 5-3
    // Mittlere ekliptikale Länge des Mondes
    let lambda_0 = normalize_deg(218.316_4 + 481_267.881_2 * t);
    // Elongation des Mondes von der Sonne
    let d = normalize_deg(297.850_2 + 445_267.111_5 * t);
    // Sonnen-Anomalie (mittlere Anomalie der Sonne)
    let m_sun = normalize_deg(357.529_1 + 35_999.050_3 * t);
    // Mond-Anomalie (mittlere Anomalie des Mondes)
    let m_moon = normalize_deg(134.963_4 + 477_198.867_6 * t);
    // Breitenargument (Winkelabstand des Mondes vom aufsteigenden Knoten)
    let u = normalize_deg(93.272_1 + 483_202.017_5 * t);

    // Grad → Radiant für Winkelfunktionen
    let d_r = d * DEG_TO_RAD;
    let m_sun_r = m_sun * DEG_TO_RAD;
    let m_moon_r = m_moon * DEG_TO_RAD;
    let u_r = u * DEG_TO_RAD;

    // Längenkorrektur Δλ [Grad] — Vallado Table 5-3 (6 Hauptterme)
    let delta_lambda = 6.288_742 * libm::sin(m_moon_r)
        + 1.274_018 * libm::sin(2.0 * d_r - m_moon_r)
        + 0.658_309 * libm::sin(2.0 * d_r)
        + 0.213_616 * libm::sin(2.0 * m_moon_r)
        - 0.185_596 * libm::sin(m_sun_r)
        - 0.114_336 * libm::sin(2.0 * u_r);

    // Breitenkorrektur Δβ [Grad] — Vallado Table 5-3 (3 Hauptterme)
    let delta_beta = 5.128_122 * libm::sin(u_r)
        + 0.280_602 * libm::sin(m_moon_r + u_r)
        + 0.277_693 * libm::sin(m_moon_r - u_r);

    // Entfernungskorrektur Δr [km] — Vallado Table 5-3
    let delta_r_km = -20.905_355 * libm::cos(m_moon_r)
        - 3.699_111 * libm::cos(2.0 * d_r - m_moon_r)
        - 2.955_968 * libm::cos(2.0 * d_r)
        - 0.569_925 * libm::cos(2.0 * m_moon_r)
        + 0.048_888 * libm::cos(m_sun_r)
        + 0.246_158 * libm::cos(2.0 * u_r);

    // Mittlere Entfernung des Mondes [km] (Vallado Gl. 5-19)
    let r_mean_km = 385_000.529;
    let r_km = r_mean_km + delta_r_km;
    let r_m = r_km * 1000.0;

    let lambda_deg = lambda_0 + delta_lambda;
    let beta_deg = delta_beta;

    ecliptic_to_eci(lambda_deg, beta_deg, r_m, t)
}

/// Berechnet die ECI-Geschwindigkeit des Mondes durch numerische Differentiation.
///
/// Zentraldifferenz mit dt = 10 Centisekunden (0.1 s):
/// v ≈ (r(t+dt) − r(t−dt)) / (2·dt_s)
///
/// # Einheiten
/// - Eingabe `get_centisecs`: AGC-Zeit in Centisekunden [cs]
/// - Ausgabe: ECI-Geschwindigkeitsvektor [m/s]
///
/// # AGC-Quelle
/// Entspricht numerischer Ableitung im Navigations-Filter
/// (`Comanche055/PLANETARY_INERTIAL_ORIENTATION.agc`).
pub fn moon_velocity(get_centisecs: f64) -> Vec3 {
    let dt_cs = 10.0_f64; // 10 Centisekunden = 0.1 s  (genug Auflösung bei korrektem t in JC)
    let dt_s = dt_cs / 100.0; // Centisekunden → Sekunden

    let r_plus = moon_position(get_centisecs + dt_cs);
    let r_minus = moon_position(get_centisecs - dt_cs);

    // Zentraldifferenz: (r(t+dt) − r(t-dt)) / (2·dt)
    (r_plus - r_minus).scale(1.0 / (2.0 * dt_s))
}

/// Berechnet die ECI-Position der Sonne aus der AGC-Ground-Elapsed-Time.
///
/// Verwendet Valladoʼs Low-Precision Solar Ephemeris (§5.4).
/// Genauigkeit: ≈ 0.01° in Winkel.
///
/// # Einheiten
/// - Eingabe `get_centisecs`: AGC-Zeit in Centisekunden seit GET=0 [cs]
/// - Ausgabe: ECI-Positionsvektor [m]
///
/// # AGC-Quelle
/// Entspricht dem Sonnen-Ephemeris in `Comanche055/PLANETARY_INERTIAL_ORIENTATION.agc`.
///
/// # Literatur
/// Vallado, "Fundamentals of Astrodynamics and Applications", 4. Aufl., §5.4.
pub fn sun_position(get_centisecs: f64) -> Vec3 {
    // Julianische Jahrhunderte seit J2000.
    // 1 JC = 36 525 Tage × 86 400 s/d × 100 cs/s = 315 576 000 000 cs
    let t = get_centisecs / 315_576_000_000.0;

    // Mittlere Länge der Sonne [Grad] — Vallado Gl. 5-17
    let lambda_sun_0 = normalize_deg(280.460 + 36_000.771 * t);
    // Mittlere Anomalie der Sonne [Grad] — Vallado Gl. 5-17
    let m_sun = normalize_deg(357.529_109_2 + 35_999.050_34 * t);

    let m_sun_r = m_sun * DEG_TO_RAD;

    // Ekliptikale Länge der Sonne [Grad] (Gl. 5-18, 2 Korrekturen)
    let lambda_ecliptic = lambda_sun_0
        + 1.914_666_471 * libm::sin(m_sun_r)
        + 0.019_994_643 * libm::sin(2.0 * m_sun_r);

    // Sonnen-Abstand [AU] — Vallado Gl. 5-18
    let r_au = 1.000_140_612
        - 0.016_708_617 * libm::cos(m_sun_r)
        - 0.000_139_589 * libm::cos(2.0 * m_sun_r);

    let r_m = r_au * AU_M;

    // Breite β = 0 für die Sonne (Ekliptik-Ebene)
    ecliptic_to_eci(lambda_ecliptic, 0.0, r_m, t)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{moon_position, moon_velocity, sun_position};

    /// Mondabstand bei GET=0 muss im plausiblen Bereich 350 000–410 000 km liegen.
    #[test]
    fn moon_distance_plausible() {
        let r = moon_position(0.0);
        let dist_km = r.magnitude() / 1000.0;
        assert!(
            dist_km >= 350_000.0 && dist_km <= 410_000.0,
            "Mondabstand unplausibel: {:.0} km",
            dist_km
        );
    }

    /// Sonnenabstand bei GET=0 muss im plausiblen Bereich 147–152 Mio km liegen.
    #[test]
    fn sun_distance_plausible() {
        let r = sun_position(0.0);
        let dist_km = r.magnitude() / 1000.0;
        assert!(
            dist_km >= 147_000_000.0 && dist_km <= 152_000_000.0,
            "Sonnenabstand unplausibel: {:.0} km",
            dist_km
        );
    }

    /// Mondgeschwindigkeit bei GET=0 muss ca. 1022 m/s ± 200 m/s betragen.
    #[test]
    fn moon_velocity_magnitude_plausible() {
        let v = moon_velocity(0.0);
        let speed = v.magnitude();
        assert!(
            speed >= 822.0 && speed <= 1222.0,
            "Mondgeschwindigkeit unplausibel: {:.1} m/s",
            speed
        );
    }

    /// Referenzwert für Mondposition bei GET=0 (J2000-Epoche):
    /// Erwartet nahe (−279 183 599, −254 824 122, −73 209 500) m ± 5 000 km.
    ///
    /// Referenzwert berechnet mit identischer Vallado-Näherungsformel bei t=0
    /// (Toleranz wegen Näherung ~0.3° Winkelfehler ≈ 2 000 km in Abstand).
    #[test]
    fn moon_position_reference_get0() {
        let r = moon_position(0.0);
        let ref_x = -279_183_599.0_f64;
        let ref_y = -254_824_122.0_f64;
        let ref_z = -73_209_500.0_f64;

        let tolerance_m = 5_000.0 * 1000.0; // 5 000 km in Metern

        let dx = (r.x - ref_x).abs();
        let dy = (r.y - ref_y).abs();
        let dz = (r.z - ref_z).abs();

        assert!(
            dx < tolerance_m,
            "x-Abweichung zu groß: {:.0} m (Limit: {:.0} m)",
            dx,
            tolerance_m
        );
        assert!(
            dy < tolerance_m,
            "y-Abweichung zu groß: {:.0} m (Limit: {:.0} m)",
            dy,
            tolerance_m
        );
        assert!(
            dz < tolerance_m,
            "z-Abweichung zu groß: {:.0} m (Limit: {:.0} m)",
            dz,
            tolerance_m
        );
    }
}
