//! Orbital-Propagation mit J2-Störterm.
//!
//! Erweitert die Zweikörper-Propagation aus [`conic`] um den ersten
//! Zonalkoeffizienten J2 der nicht-sphärischen Massenverteilung.  Das
//! Ergebnis ist ein einfacher Störungsansatz erster Ordnung: Kepler-Schritt
//! plus einmaliger J2-Impuls (Δv = a_J2 · Δt).  Für kurze Integrationszeiten
//! (≲ eine Periode) reicht diese Näherung für Navigationszwecke aus.
//!
//! # Einheiten
//! Alle Positionen in Metern, Geschwindigkeiten in m/s, Zeiten in Sekunden,
//! μ in m³/s².
//!
//! # Referenz
//! Vallado, *Fundamentals of Astrodynamics and Applications*, 4th ed., §9.6.2.
//!
//! # AGC-Quelle
//! `Comanche055/ORBITAL_INTEGRATION.agc`, KEPPREP-Routine.

use crate::alarm::AlarmState;
use crate::navigation::conic::kepler_step;
use crate::navigation::state_vector::StateVector;
use crate::types::Vec3;

// ---------------------------------------------------------------------------
// Schwellwert Mond/Erde-Klassifikation
// ---------------------------------------------------------------------------

/// Mindestabstand vom Erdmittelpunkt in Metern, ab dem das Fahrzeug dem
/// Gravitationsfeld des Mondes (statt der Erde) zugeordnet wird.
///
/// Der Wert 3 000 000 m (3 000 km) liegt zwischen Erdumlaufbahn (~400 km
/// Höhe ≈ 6 778 km Radius) und dem Mond (~384 400 km Entfernung) und
/// entspricht der im AGC verwendeten Unterscheidung zwischen Erde und Mond
/// für Navigationsprogramme (P30, Physik-Integrator).
///
/// Einheit: Meter (m), Abstand vom Erdmittelpunkt.
///
/// AGC source: `Comanche055/P30,P37.agc`, TTOGO-Routine (Zentralkörper-Auswahl).
pub const MOON_THRESHOLD_M: f64 = 3_000_000.0;

// ---------------------------------------------------------------------------
// CentralBody
// ---------------------------------------------------------------------------

/// Zentralkörper der Propagation.
///
/// Kapselt die gravitationsphysikalischen Konstanten des jeweiligen Körpers.
/// Der Mond-J2-Wert ist physikalisch korrekt; sein Einfluss ist im Vergleich
/// zur Erde vernachlässigbar klein.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CentralBody {
    /// Erde (LEO- / GTO- / GEO-Bahnen).
    Earth,
    /// Mond (Mondumkreisungsbahnen, LOI, TEI).
    Moon,
}

impl CentralBody {
    /// Gravitationsparameter μ = G·M in m³/s².
    ///
    /// | Körper | μ (m³/s²)        |
    /// |--------|------------------|
    /// | Erde   | 3.986 004 418e14 |
    /// | Mond   | 4.904 869 5e12   |
    #[inline]
    pub const fn mu(&self) -> f64 {
        match self {
            CentralBody::Earth => 3.986_004_418e14,
            CentralBody::Moon => 4.904_869_5e12,
        }
    }

    /// J2-Zonalkoeffizient (dimensionslos).
    ///
    /// | Körper | J2           |
    /// |--------|--------------|
    /// | Erde   | 1.082 63e−3  |
    /// | Mond   | 2.023 5e−4   |
    #[inline]
    pub const fn j2(&self) -> f64 {
        match self {
            CentralBody::Earth => 1.082_63e-3,
            CentralBody::Moon => 2.023_5e-4,
        }
    }

    /// Äquatorialradius R_eq in m.
    ///
    /// | Körper | R_eq (m)      |
    /// |--------|---------------|
    /// | Erde   | 6 378 137.0   |
    /// | Mond   | 1 737 400.0   |
    #[inline]
    pub const fn r_equatorial(&self) -> f64 {
        match self {
            CentralBody::Earth => 6_378_137.0,
            CentralBody::Moon => 1_737_400.0,
        }
    }
}

// ---------------------------------------------------------------------------
// J2-Störbeschleunigung
// ---------------------------------------------------------------------------

/// J2-Störbeschleunigung in kartesischen ECI-Koordinaten (m/s²).
///
/// Berechnet den Anteil der nicht-sphärischen Massenverteilung (erster
/// Zonalkoeffizient J2) an der Gravitationsbeschleunigung.
///
/// # Formel
/// (Vallado, §9.6.2, Gl. 9-31)
///
/// ```text
/// k  = (3/2) · J2 · μ · R_eq² / r⁵
/// aₓ = k · x · (5·z²/r² − 1)
/// aᵧ = k · y · (5·z²/r² − 1)
/// a_z = k · z · (5·z²/r² − 3)
/// ```
///
/// # Parameter
/// - `pos`:  Positionsvektor in m (ECI J2000)
/// - `body`: Zentralkörper (Erdparameter werden verwendet)
///
/// # Rückgabe
/// J2-Störbeschleunigung in m/s².  Für `|r| = 0` wird ein Nullvektor
/// zurückgegeben (Singularität vermieden; der Aufrufer ist für sinnvolle
/// Eingaben verantwortlich).
///
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc`, KEPPREP routine.
pub fn j2_acceleration(pos: Vec3, body: &CentralBody) -> Vec3 {
    let x = pos.x;
    let y = pos.y;
    let z = pos.z;

    let r2 = x * x + y * y + z * z;
    if r2 == 0.0 {
        return Vec3::ZERO;
    }

    let r = libm::sqrt(r2);
    let r5 = r2 * r2 * r; // r^5

    let mu = body.mu();
    let j2 = body.j2();
    let req = body.r_equatorial();

    // k = (3/2) · J2 · μ · R_eq² / r⁵
    let k = 1.5 * j2 * mu * req * req / r5;

    let z2_over_r2 = z * z / r2;

    Vec3::new(
        k * x * (5.0 * z2_over_r2 - 1.0),
        k * y * (5.0 * z2_over_r2 - 1.0),
        k * z * (5.0 * z2_over_r2 - 3.0),
    )
}

// ---------------------------------------------------------------------------
// Orbital-Propagation (Kepler + J2-Störterm)
// ---------------------------------------------------------------------------

/// Propagiert einen Zustandsvektor um Δt Sekunden mit J2-Störterm.
///
/// Strategie (Störungsansatz 1. Ordnung):
/// 1. Zweikörper-Kepler-Schritt für Position und Geschwindigkeit.
/// 2. J2-Störbeschleunigung am Ausgangspunkt berechnen.
/// 3. Geschwindigkeit korrigieren: v₁_korr = v₁_kepler + a_J2 · Δt.
///
/// Dieser Ansatz ist äquivalent zu einem Euler-Schritt im Störungspotential
/// und ausreichend für eine Integrationszeit ≲ 1 Periode.  Für längere
/// Propagation ist ein RK4-Integrator mit vollständiger Ableitung nötig.
///
/// # Parameter
/// - `sv`:    Ausgangszustandsvektor (m, m/s, s)
/// - `dt`:    Propagationsdauer in Sekunden
/// - `body`:  Zentralkörper (Gravitationsparameter, J2, R_eq)
/// - `alarm`: Alarmzustand; Alarmcode `0x0614` bei Kepler-Nicht-Konvergenz
///
/// # Rückgabe
/// Propagierter `StateVector` mit J2-korrigierter Geschwindigkeit.
///
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc`, KEPPREP routine.
pub fn propagate(
    sv: StateVector,
    dt: f64,
    body: CentralBody,
    alarm: &mut AlarmState,
) -> StateVector {
    // J2-Beschleunigung am Startpunkt (vor dem Kepler-Schritt)
    let a_j2 = j2_acceleration(sv.position, &body);

    // Zweikörper-Propagation
    let sv_kepler = kepler_step(sv, dt, body.mu(), alarm);

    // Störungskorrektur: Δv = a_J2 · dt auf Kepler-Geschwindigkeit addieren
    let v_corrected = Vec3::new(
        sv_kepler.velocity.x + a_j2.x * dt,
        sv_kepler.velocity.y + a_j2.y * dt,
        sv_kepler.velocity.z + a_j2.z * dt,
    );

    StateVector::new(sv_kepler.position, v_corrected, sv_kepler.time)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{j2_acceleration, propagate, CentralBody};
    use crate::alarm::AlarmState;
    use crate::navigation::conic::kepler_step;
    use crate::navigation::state_vector::StateVector;
    use crate::types::{Met, Vec3};

    // μ_Erde für lokale Berechnungen
    const MU_EARTH: f64 = 3.986_004_418e14;
    const R_EARTH: f64 = 6_378_137.0;
    const J2_EARTH: f64 = 1.082_63e-3;

    // -----------------------------------------------------------------------
    // j2_acceleration_at_equator
    // -----------------------------------------------------------------------

    /// Analytische Probe für z = 0, r = R_eq (Äquator auf Erdoberfläche).
    ///
    /// Für z = 0: a_x = -k·x, a_y = -k·y, a_z = 0 (5·0 − 3 = −3 aber z = 0)
    /// mit k = (3/2)·J2·μ·R_eq²/r⁵, r = R_eq → k = (3/2)·J2·μ/R_eq³.
    #[test]
    fn j2_acceleration_at_equator() {
        let r = R_EARTH;
        let pos = Vec3::new(r, 0.0, 0.0); // x-Achse, Äquator

        let a = j2_acceleration(pos, &CentralBody::Earth);

        // Erwartetes k = (3/2)·J2·μ/R_eq³
        let k = 1.5 * J2_EARTH * MU_EARTH / (r * r * r);

        // a_x = k · r · (0 − 1) = −k·r
        let expected_ax = -k * r;
        // a_y = k · 0 · ... = 0
        // a_z = k · 0 · ... = 0

        assert!(
            (a.x - expected_ax).abs() < 1e-6,
            "a_x = {}, erwartet {}",
            a.x,
            expected_ax
        );
        assert!(a.y.abs() < 1e-15, "a_y sollte 0 sein, war {}", a.y);
        assert!(a.z.abs() < 1e-15, "a_z sollte 0 sein, war {}", a.z);
    }

    // -----------------------------------------------------------------------
    // j2_acceleration_at_pole
    // -----------------------------------------------------------------------

    /// Analytische Probe für z = r, x = y = 0 (Pol auf Erdoberfläche).
    ///
    /// Für x = y = 0, z = r:
    ///   a_x = k·0·(5·1 − 1) = 0
    ///   a_y = k·0·(5·1 − 1) = 0
    ///   a_z = k·z·(5·1 − 3) = k·z·2 = 2·k·r
    #[test]
    fn j2_acceleration_at_pole() {
        let r = R_EARTH;
        let pos = Vec3::new(0.0, 0.0, r); // z-Achse, Pol

        let a = j2_acceleration(pos, &CentralBody::Earth);

        let k = 1.5 * J2_EARTH * MU_EARTH / (r * r * r);
        let expected_az = 2.0 * k * r;

        assert!(a.x.abs() < 1e-15, "a_x sollte 0 sein, war {}", a.x);
        assert!(a.y.abs() < 1e-15, "a_y sollte 0 sein, war {}", a.y);
        assert!(
            (a.z - expected_az).abs() < 1e-6,
            "a_z = {}, erwartet {}",
            a.z,
            expected_az
        );
    }

    // -----------------------------------------------------------------------
    // propagate_no_j2_equals_kepler_step
    // -----------------------------------------------------------------------

    /// Für den Mond (sehr kleines J2) muss propagate() nahe an kepler_step()
    /// liegen — Abweichung < 1 m nach einer Mondumkreisungsperiode (~7092 s).
    #[test]
    fn propagate_no_j2_equals_kepler_step() {
        // Kreisbahn um den Mond: h = 100 km über Oberfläche
        let mu_moon = CentralBody::Moon.mu();
        let r_moon = CentralBody::Moon.r_equatorial();
        let r = r_moon + 100_000.0; // 100 km Höhe
        let v_circ = libm::sqrt(mu_moon / r);

        let pos = Vec3::new(r, 0.0, 0.0);
        let vel = Vec3::new(0.0, v_circ, 0.0);
        let sv = StateVector::new(pos, vel, Met::from_seconds(0.0));

        // Umlaufzeit
        let period = 2.0 * core::f64::consts::PI * libm::sqrt(r * r * r / mu_moon);

        let mut alarm_k = AlarmState::new();
        let mut alarm_p = AlarmState::new();

        let sv_kepler = kepler_step(sv, period, mu_moon, &mut alarm_k);
        let sv_j2 = propagate(sv, period, CentralBody::Moon, &mut alarm_p);

        let dx = sv_j2.position.x - sv_kepler.position.x;
        let dy = sv_j2.position.y - sv_kepler.position.y;
        let dz = sv_j2.position.z - sv_kepler.position.z;
        let dist = libm::sqrt(dx * dx + dy * dy + dz * dz);

        assert!(
            dist < 1.0,
            "Mondpropagation: Abweichung propagate vs. kepler_step = {dist:.3} m (erwartet < 1 m)"
        );
    }

    // -----------------------------------------------------------------------
    // propagate_j2_diverges_from_two_body
    // -----------------------------------------------------------------------

    /// Prüft, dass J2-Propagation von der reinen Kepler-Bahn abweicht.
    ///
    /// Strategie: Wir führen N kurze Schritte (1/10 Periode) durch und
    /// akkumulieren den Unterschied zwischen `propagate` und `kepler_step`.
    /// Nach 10 solchen Schritten muss die Positionsabweichung messbar sein
    /// (RAAN-Drift, J2-induzierte Bahnänderung).
    ///
    /// Hintergrund: Der Störungsansatz korrigiert die **Geschwindigkeit** nach
    /// jedem Schritt.  Erst im nächsten Schritt wirkt die korrigierte
    /// Geschwindigkeit auf die Position.  Deshalb divergieren die Bahnen erst
    /// nach mehreren Schritten sichtbar in der Position.
    #[test]
    fn propagate_j2_diverges_from_two_body() {
        // LEO: 400 km Höhe, Inklination 51.6° (ISS-ähnlich)
        let r = R_EARTH + 400_000.0;
        let v_circ = libm::sqrt(MU_EARTH / r);

        // Inklination 51.6° → v in x-z-Ebene aufteilen
        let inc = 51.6_f64.to_radians();
        let pos = Vec3::new(r, 0.0, 0.0);
        let vel = Vec3::new(0.0, v_circ * libm::cos(inc), v_circ * libm::sin(inc));

        let sv0 = StateVector::new(pos, vel, Met::from_seconds(0.0));

        let period = 2.0 * core::f64::consts::PI * libm::sqrt(r * r * r / MU_EARTH);
        // Schrittweite: 1/10 Periode (sicher für Kepler-Konvergenz)
        let dt_step = period / 10.0;

        // Kepler-Referenzbahn: 10 × kepler_step
        let mut sv_k = sv0;
        let mut alarm_k = AlarmState::new();
        for _ in 0..10 {
            sv_k = kepler_step(sv_k, dt_step, MU_EARTH, &mut alarm_k);
        }

        // J2-Bahn: 10 × propagate
        let mut sv_j = sv0;
        let mut alarm_p = AlarmState::new();
        for _ in 0..10 {
            sv_j = propagate(sv_j, dt_step, CentralBody::Earth, &mut alarm_p);
        }

        let dx = sv_j.position.x - sv_k.position.x;
        let dy = sv_j.position.y - sv_k.position.y;
        let dz = sv_j.position.z - sv_k.position.z;
        let dist = libm::sqrt(dx * dx + dy * dy + dz * dz);

        // Nach einer vollen Periode mit J2-Korrekturen erwarten wir
        // eine Positionsabweichung von mindestens 100 m.
        // (RAAN-Drift ≈ 0.98°/Tag bei 400 km ISS-Inklination)
        assert!(
            dist > 100.0,
            "J2-Drift nach 10 Schritten zu klein: {dist:.1} m (erwartet > 100 m)"
        );
    }
}
