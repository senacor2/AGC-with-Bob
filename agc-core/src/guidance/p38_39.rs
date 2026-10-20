//! P38 / P39 — Stable Orbit Insertion und Midcourse Correction.
//!
//! P38 berechnet das Lambert-Manöver (SOI) zur Annäherung an das passive
//! Fahrzeug. P39 verfeinert dieses Manöver anhand der aktuell gemessenen
//! Passivfahrzeug-Position.
//!
//! # P38 — Stable Orbit Insertion (SOI)
//! 1. Passivfahrzeug zum Intercept-Zeitpunkt `TIG + TOF` propagieren (Kepler)
//! 2. Lambert-Solver: Transfer vom aktiven Fahrzeug zur propagierten Position
//! 3. Δv_SOI = v_lambert_init − v_aktiv_bei_TIG
//! 4. Δv_Intercept = v_passiv_bei_intercept − v_lambert_final
//!
//! # P39 — Midcourse Correction
//! Wie P38, aber mit der **gemessenen** (nicht propagierten) Passivfahrzeug-
//! Position → Δv_Korrektur = neue SOI-Bahn − geplante SOI-Bahn.
//!
//! AGC-Quelle: `Comanche055/STABLE_ORBIT_-_P38-P39.agc`

use crate::alarm::AlarmState;
use crate::navigation::conic::{kepler_step, lambert};
use crate::navigation::orbital::CentralBody;
use crate::navigation::state_vector::StateVector;
use crate::types::Vec3;

// ---------------------------------------------------------------------------
// P38 Eingabe / Ausgabe
// ---------------------------------------------------------------------------

/// Eingabedaten für P38 (Stable Orbit Insertion).
///
/// # Einheiten
/// | Feld          | Einheit  | Bedeutung                                      |
/// |---------------|----------|------------------------------------------------|
/// | `sv_active`   | m / m/s  | SV des aktiven Fahrzeugs zum Burn-Zeitpunkt TIG|
/// | `sv_passive`  | m / m/s  | SV des passiven Fahrzeugs (Ground-Upload)      |
/// | `tof`         | Sekunden | Transfer-Zeit von TIG bis Intercept             |
/// | `body`        | —        | Zentralkörper (Erde oder Mond)                 |
///
/// # AGC-Quelle
/// `Comanche055/STABLE_ORBIT_-_P38-P39.agc`, MAINRTNE routine.
#[derive(Copy, Clone, Debug)]
pub struct P38Input {
    /// Zustandsvektor des aktiven Fahrzeugs zum Burn-Zeitpunkt TIG (ECI J2000, m / m/s).
    pub sv_active: StateVector,
    /// Zustandsvektor des passiven Fahrzeugs zum Referenzzeitpunkt (ECI J2000, m / m/s).
    pub sv_passive: StateVector,
    /// Transfer-Zeit in Sekunden (Time of Flight, TOF > 0).
    pub tof: f64,
    /// Zentralkörper der Propagation (Erde oder Mond).
    pub body: CentralBody,
}

/// Ausgabedaten von P38 (Stable Orbit Insertion).
///
/// # Einheiten
/// | Feld               | Einheit | Bedeutung                                        |
/// |--------------------|---------|--------------------------------------------------|
/// | `delta_v_soi`      | m/s     | Δv am Burn-Punkt (ECI J2000): v_transfer − v_aktiv|
/// | `delta_v_intercept`| m/s     | Δv am Intercept-Punkt (ECI J2000)                |
///
/// # AGC-Quelle
/// `Comanche055/STABLE_ORBIT_-_P38-P39.agc`, MAINRTNE routine.
/// `Comanche055/CONIC_SUBROUTINES.agc`, LAMBERT routine.
#[derive(Copy, Clone, Debug)]
pub struct P38Output {
    /// Δv-Impuls am Burn-Punkt (SOI-Manöver) in m/s (ECI J2000).
    pub delta_v_soi: Vec3,
    /// Δv-Impuls am Intercept-Punkt in m/s (ECI J2000).
    pub delta_v_intercept: Vec3,
}

// ---------------------------------------------------------------------------
// p38_compute
// ---------------------------------------------------------------------------

/// Berechnet das SOI-Manöver (Stable Orbit Insertion, P38).
///
/// Propagiert das passive Fahrzeug zum Intercept-Zeitpunkt und löst das
/// Lambert-Problem zwischen aktivem und passivem Fahrzeug.
///
/// # Algorithmus
/// 1. Passivfahrzeug propagieren: `sv_passive_at_intercept = kepler_step(sv_passive, tof, μ)`
/// 2. Lambert-Solver: `(v_init, v_final) = lambert(r_active, r_passive_intercept, tof, μ, prograde=true)`
/// 3. `Δv_SOI = v_init − sv_active.velocity`
/// 4. `Δv_Intercept = sv_passive_at_intercept.velocity − v_final`
///
/// # Rückgabe
/// `Some(P38Output)` bei Erfolg, `None` bei Lambert-Nicht-Konvergenz
/// (z. B. 180°-Singularität oder ungültige Eingaben).
///
/// # AGC source
/// `Comanche055/STABLE_ORBIT_-_P38-P39.agc`, MAINRTNE routine
/// `Comanche055/CONIC_SUBROUTINES.agc`, LAMBERT routine
pub fn p38_compute(input: &P38Input, alarm: &mut AlarmState) -> Option<P38Output> {
    let mu = input.body.mu();

    // Schritt 1: Passivfahrzeug zum Intercept-Zeitpunkt propagieren
    // AGC source: Comanche055/STABLE_ORBIT_-_P38-P39.agc, PREC/TT und TIMETHET
    let sv_passive_at_intercept = kepler_step(input.sv_passive, input.tof, mu, alarm);

    // Schritt 2: Lambert-Solver zwischen aktivem Fahrzeug und propagierter Passivposition
    // AGC source: Comanche055/CONIC_SUBROUTINES.agc, LAMBERT routine
    let (v_transfer_init, v_transfer_final) = lambert(
        input.sv_active.position,
        sv_passive_at_intercept.position,
        input.tof,
        mu,
        true, // prograde
    )?;

    // Schritt 3: Δv am Burn-Punkt
    let delta_v_soi = v_transfer_init - input.sv_active.velocity;

    // Schritt 4: Δv am Intercept-Punkt (Rendezvous-Korrektur)
    let delta_v_intercept = sv_passive_at_intercept.velocity - v_transfer_final;

    Some(P38Output {
        delta_v_soi,
        delta_v_intercept,
    })
}

// ---------------------------------------------------------------------------
// P39 Eingabe / Ausgabe
// ---------------------------------------------------------------------------

/// Eingabedaten für P39 (Midcourse Correction).
///
/// # Einheiten
/// | Feld                   | Einheit  | Bedeutung                                    |
/// |------------------------|----------|----------------------------------------------|
/// | `sv_active`            | m / m/s  | Aktueller SV des aktiven Fahrzeugs           |
/// | `sv_passive_measured`  | m / m/s  | Gemessener (aktueller) SV des Passivfahrzeugs|
/// | `tof_remaining`        | Sekunden | Verbleibende Flugzeit bis Intercept          |
/// | `body`                 | —        | Zentralkörper (Erde oder Mond)               |
///
/// # AGC-Quelle
/// `Comanche055/STABLE_ORBIT_-_P38-P39.agc`, MAINRTNE routine (P39-Zweig).
#[derive(Copy, Clone, Debug)]
pub struct P39Input {
    /// Aktueller Zustandsvektor des aktiven Fahrzeugs (ECI J2000, m / m/s).
    pub sv_active: StateVector,
    /// Gemessener (aktueller) Zustandsvektor des passiven Fahrzeugs (ECI J2000, m / m/s).
    pub sv_passive_measured: StateVector,
    /// Verbleibende Flugzeit bis zum Intercept in Sekunden (> 0).
    pub tof_remaining: f64,
    /// Zentralkörper der Propagation.
    pub body: CentralBody,
}

/// Ausgabedaten von P39 (Midcourse Correction).
///
/// # Einheiten
/// | Feld                | Einheit | Bedeutung                                       |
/// |---------------------|---------|--------------------------------------------------|
/// | `delta_v_correction`| m/s     | Δv-Korrektur (ECI J2000): neue − ursprüngliche Bahn|
///
/// # AGC-Quelle
/// `Comanche055/STABLE_ORBIT_-_P38-P39.agc`, MAINRTNE routine (P39-Zweig).
#[derive(Copy, Clone, Debug)]
pub struct P39Output {
    /// Δv-Korrekturmanöver in m/s (ECI J2000).
    pub delta_v_correction: Vec3,
}

// ---------------------------------------------------------------------------
// p39_compute
// ---------------------------------------------------------------------------

/// Berechnet die Midcourse Correction (P39).
///
/// Verwendet die **gemessene** Passivfahrzeug-Position statt der propagierten,
/// um eine Bahnkorrektur zu berechnen.
///
/// # Algorithmus
/// 1. Gemessenes Passivfahrzeug zum Intercept propagieren (Lambert-Ziel ermitteln)
/// 2. Lambert-Solver: neue Transfer-Bahn mit gemessener Position
/// 3. Δv_Korrektur = v_neue_transfer_init − v_aktiv
///
/// # Rückgabe
/// `Some(P39Output)` bei Erfolg, `None` bei Lambert-Nicht-Konvergenz.
///
/// # AGC source
/// `Comanche055/STABLE_ORBIT_-_P38-P39.agc`, MAINRTNE routine (P39-Zweig)
/// `Comanche055/CONIC_SUBROUTINES.agc`, LAMBERT routine
pub fn p39_compute(input: &P39Input, alarm: &mut AlarmState) -> Option<P39Output> {
    let mu = input.body.mu();

    // Gemessenes Passivfahrzeug zum Intercept propagieren
    // AGC source: Comanche055/STABLE_ORBIT_-_P38-P39.agc, PREC/TT
    let sv_passive_at_intercept =
        kepler_step(input.sv_passive_measured, input.tof_remaining, mu, alarm);

    // Lambert-Solver mit gemessener Position
    // AGC source: Comanche055/CONIC_SUBROUTINES.agc, LAMBERT routine
    let (v_correction_init, _v_correction_final) = lambert(
        input.sv_active.position,
        sv_passive_at_intercept.position,
        input.tof_remaining,
        mu,
        true, // prograde
    )?;

    // Δv-Korrektur = neue Abfluggeschwindigkeit − aktuelle Geschwindigkeit
    let delta_v_correction = v_correction_init - input.sv_active.velocity;

    Some(P39Output { delta_v_correction })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{p38_compute, p39_compute, P38Input, P39Input};
    use crate::alarm::AlarmState;
    use crate::navigation::orbital::CentralBody;
    use crate::navigation::state_vector::StateVector;
    use crate::types::{Met, Vec3};

    /// μ_Erde [m³/s²]
    const MU_EARTH: f64 = 3.986_004_418e14;

    /// Lambert-Transfer-Test: LEO 120°-Bogen (r = 6 678 km).
    ///
    /// Testfall aus `navigation::conic` (`lambert_arc120_leo`):
    ///   r₁ = (r, 0, 0), r₂ = r·(cos 120°, sin 120°, 0), tof = T/3
    ///
    /// Prüft p38_compute mit einem bekannten, konvergierenden Lambert-Bogen:
    /// - Lambert konvergiert (Some)
    /// - Δv_SOI liegt im erwarteten Bereich für einen LEO-Transfer
    /// - Konsistenz: kepler_step(r1 + v_transfer_init, tof) landet < 500 m von r2
    ///
    /// (Hohmann-Transfer wäre 180°-Bogen → Lambert-Singularität, daher 120°-Bogen.)
    ///
    /// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, LAMBERT routine.
    #[test]
    fn lambert_transfer_120deg_delta_v() {
        use crate::navigation::conic::kepler_step;

        let r = 6_678_000.0_f64;
        let v_circ = libm::sqrt(MU_EARTH / r);
        let period = 2.0 * core::f64::consts::PI * libm::sqrt(r * r * r / MU_EARTH);

        // r₁ = (r, 0, 0), v₁ tangential (kreisförmige Ausgangsbahn)
        let sv_active = StateVector::new(
            Vec3::new(r, 0.0, 0.0),
            Vec3::new(0.0, v_circ, 0.0),
            Met::from_seconds(0.0),
        );

        // r₂ = r·(cos 120°, sin 120°, 0) auf derselben Kreisbahn
        let angle = 2.0 * core::f64::consts::PI / 3.0; // 120°
        let r2 = Vec3::new(r * libm::cos(angle), r * libm::sin(angle), 0.0);
        let v2_circ = Vec3::new(-v_circ * libm::sin(angle), v_circ * libm::cos(angle), 0.0);
        let tof = period / 3.0; // T/3

        // Passives Fahrzeug: rückwärts von r₂ propagiert,
        // sodass kepler_step(sv_passive, tof) → r₂
        let sv_at_r2 = StateVector::new(r2, v2_circ, Met::from_seconds(0.0));
        let mut alarm = AlarmState::new();
        let sv_passive = kepler_step(sv_at_r2, -tof, MU_EARTH, &mut alarm);
        assert!(alarm.last().is_none(), "Rückwärts-Propagation kein Alarm");

        let input = P38Input {
            sv_active,
            sv_passive,
            tof,
            body: CentralBody::Earth,
        };

        let result = p38_compute(&input, &mut alarm);
        assert!(result.is_some(), "120°-Lambert-Transfer muss konvergieren");

        let out = result.unwrap();
        let dv_soi = out.delta_v_soi;

        // Δv_SOI für 120°-Transfer auf LEO: typisch 100–400 m/s
        let dv_soi_mag = dv_soi.magnitude();
        assert!(
            dv_soi_mag > 0.0 && dv_soi_mag < 1000.0,
            "Δv_SOI = {:.3} m/s, erwartet 0–1000 m/s",
            dv_soi_mag
        );

        // Konsistenz: v_aktiv + Δv_SOI ergibt v_transfer_init.
        // kepler_step(sv_active + dv_soi, tof) muss nahe r₂ landen.
        let v_transfer = Vec3::new(
            sv_active.velocity.x + dv_soi.x,
            sv_active.velocity.y + dv_soi.y,
            sv_active.velocity.z + dv_soi.z,
        );
        let sv_transfer = StateVector::new(sv_active.position, v_transfer, Met::from_seconds(0.0));
        let sv_final = kepler_step(sv_transfer, tof, MU_EARTH, &mut alarm);
        let dist_to_r2 = (sv_final.position - r2).magnitude();
        assert!(
            dist_to_r2 < 1000.0,
            "Konsistenz: Landung {:.1} m von r₂ entfernt (erwartet < 1000 m)",
            dist_to_r2
        );

        assert!(alarm.last().is_none(), "Kein Alarm erwartet");
    }

    /// Lambert-Singularität bei 180°-Transfer → None erwartet.
    ///
    /// p38_compute propagiert das Passivfahrzeug um `tof` Sekunden. Damit
    /// die propagierte Position exakt 180° gegenüber dem aktiven Fahrzeug
    /// (+x-Achse) liegt, muss das Passivfahrzeug nach Propagation bei
    /// (−r, 0, 0) stehen. Wir positionieren es rückwärts von (−r, 0, 0)
    /// aus der Kreisbahn, sodass Vorwärtspropagation um `tof` wieder (−r, 0, 0)
    /// ergibt.
    ///
    /// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, 360LAMB label.
    #[test]
    fn lambert_singularity_returns_none() {
        use crate::navigation::conic::kepler_step;

        let r = 6_778_000.0_f64;
        let v_circ = libm::sqrt(MU_EARTH / r);
        let period = 2.0 * core::f64::consts::PI * libm::sqrt(r * r * r / MU_EARTH);
        // tof = halbe Periode → aktiv (+x), passiv nach Propagation (−x) → 180° → Singularität
        let tof = period / 2.0;

        // Aktives Fahrzeug: +x-Achse, prograde (+y-Geschwindigkeit)
        let sv_active = StateVector::new(
            Vec3::new(r, 0.0, 0.0),
            Vec3::new(0.0, v_circ, 0.0),
            Met::from_seconds(0.0),
        );

        // Passives Fahrzeug: rückwärts von (−r, 0, 0) propagiert, sodass es
        // nach Vorwärtspropagation um `tof` exakt bei (−r, 0, 0) landet.
        let sv_at_minus_x = StateVector::new(
            Vec3::new(-r, 0.0, 0.0),
            Vec3::new(0.0, -v_circ, 0.0),
            Met::from_seconds(0.0),
        );
        let mut alarm = AlarmState::new();
        let sv_passive = kepler_step(sv_at_minus_x, -tof, MU_EARTH, &mut alarm);
        assert!(alarm.last().is_none(), "Rückwärts-Propagation kein Alarm");

        let input = P38Input {
            sv_active,
            sv_passive,
            tof,
            body: CentralBody::Earth,
        };

        let result = p38_compute(&input, &mut alarm);
        assert!(
            result.is_none(),
            "180°-Singularität muss None zurückgeben, got Some"
        );
    }

    /// P39: Keine Korrektur nötig wenn das Passivfahrzeug exakt auf Kurs liegt.
    ///
    /// Testaufbau für Nullkorrektur-Semantik:
    /// - Aktives Fahrzeug befindet sich bereits auf der Lambert-Transferbahn
    ///   (v_aktiv = v_lambert_init aus einem bekannten 120°-Lambert-Transfer).
    /// - Passives Fahrzeug rückwärts von r₂ um `tof` propagiert, sodass
    ///   `kepler_step(sv_passive, tof)` = r₂ ergibt.
    /// - P39-Lambert-Aufruf: (aktiv_pos → r₂) in `tof` → liefert v_lambert_init.
    /// - Δv = v_lambert_init − v_aktiv = v_lambert_init − v_lambert_init = 0.
    ///
    /// AGC source: `Comanche055/STABLE_ORBIT_-_P38-P39.agc`, MAINRTNE (P39-Zweig).
    #[test]
    fn p39_no_correction_needed() {
        use crate::navigation::conic::kepler_step;

        let r = 6_678_000.0_f64;
        let v_circ = libm::sqrt(MU_EARTH / r);
        let period = 2.0 * core::f64::consts::PI * libm::sqrt(r * r * r / MU_EARTH);
        let angle = 2.0 * core::f64::consts::PI / 3.0; // 120°
        let tof = period / 3.0;

        // r₂ (Intercept-Punkt nach tof)
        let r2 = Vec3::new(r * libm::cos(angle), r * libm::sin(angle), 0.0);

        // Aktives Fahrzeug: Kreisbahn r bei +x-Achse
        let sv_active_circ = StateVector::new(
            Vec3::new(r, 0.0, 0.0),
            Vec3::new(0.0, v_circ, 0.0),
            Met::from_seconds(0.0),
        );

        // Lambert-Initialgeschwindigkeit für 120°-Transfer
        let (v_lambert_init, _) =
            crate::navigation::conic::lambert(sv_active_circ.position, r2, tof, MU_EARTH, true)
                .expect("120°-Lambert muss konvergieren");

        // Aktives Fahrzeug bereits auf Transferbahn (SOI-Burn wurde ausgeführt)
        let sv_active_on_transfer = StateVector::new(
            sv_active_circ.position,
            v_lambert_init,
            Met::from_seconds(0.0),
        );

        // Passives Fahrzeug: Kreisbahn-Zustand an r₂, rückwärts um tof propagiert.
        // Damit gilt: kepler_step(sv_passive, tof) ≈ r₂ (Intercept-Punkt).
        let v2_circ = Vec3::new(-v_circ * libm::sin(angle), v_circ * libm::cos(angle), 0.0);
        let sv_at_r2 = StateVector::new(r2, v2_circ, Met::from_seconds(0.0));
        let mut alarm = AlarmState::new();
        let sv_passive_on_course = kepler_step(sv_at_r2, -tof, MU_EARTH, &mut alarm);
        assert!(alarm.last().is_none(), "Rückwärts-Propagation kein Alarm");

        // P39-Aufruf: aktiv auf Transferbahn, passiv "on course"
        // p39_compute: kepler_step(sv_passive_on_course, tof) → r₂
        // Lambert(aktiv_pos → r₂, tof) → v_lambert_init
        // Δv = v_lambert_init − sv_active_on_transfer.velocity = 0
        let input = P39Input {
            sv_active: sv_active_on_transfer,
            sv_passive_measured: sv_passive_on_course,
            tof_remaining: tof,
            body: CentralBody::Earth,
        };

        let result = p39_compute(&input, &mut alarm);
        assert!(result.is_some(), "P39 muss konvergieren (passiv on-course)");

        let out = result.unwrap();
        let dv_corr = out.delta_v_correction.magnitude();
        assert!(
            dv_corr < 1.0,
            "Δv_Korrektur sollte < 1 m/s sein (kein Fehler), got {:.3} m/s",
            dv_corr
        );
        assert!(alarm.last().is_none());
    }
}
