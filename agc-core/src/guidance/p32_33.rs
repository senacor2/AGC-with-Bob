//! P32 / P33 — Co-Elliptic Sequence Initiation (CSI) und Constant Delta Height (CDH).
//!
//! P32 (CSI — Co-Elliptic Sequence Initiation) berechnet das Δv-Manöver, das das
//! aktive Fahrzeug auf eine ko-elliptische Bahn bringt. Das Δv liegt in der
//! Horizontalebene des aktiven Fahrzeugs (radiale Komponente = 0).
//!
//! P33 (CDH — Constant Delta Height) berechnet das Δv zur Nivellierung der
//! Höhendifferenz zwischen aktiver und passiver Bahn, sodass `diff_alt` konstant bleibt.
//!
//! # Ko-elliptische Bahn (Konzept)
//! Bei CSI befindet sich das aktive Fahrzeug zunächst auf einer anderen Höhe als
//! das passive. Das CSI-Manöver bringt die Bahn des aktiven Fahrzeugs auf eine
//! elliptische Bahn, deren Apogee/Perigee in der gleichen Orbitebene liegt wie
//! das passive Fahrzeug, aber mit einer festen Höhendifferenz (CDH-Punkt).
//!
//! # Einheiten
//! - Positionen: m (ECI J2000)
//! - Geschwindigkeiten: m/s (ECI J2000)
//! - Zeiten: Sekunden seit TEPHEM
//! - diff_alt: m (Höhendifferenz)
//!
//! # AGC-Quelle
//! `Comanche055/P32-P33,_P72-P73.agc`, P32-Routine, CSI/A-Subroutine, P33-Routine,
//! CDHMVR-Subroutine.

use crate::alarm::{raise, AlarmState};
use crate::navigation::conic::kepler_step;
use crate::navigation::orbital::CentralBody;
use crate::navigation::state_vector::StateVector;
use crate::types::Vec3;

// ---------------------------------------------------------------------------
// Konstanten
// ---------------------------------------------------------------------------

/// Mindest-Zeitabstand zwischen CSI und CDH [s].
///
/// AGC-Kriterium: T_CDH − T_CSI ≥ 10 Minuten = 600 s.
/// AGC source: `Comanche055/P32-P33,_P72-P73.agc`, P32-Routine Assumption (2).
const CSI_CDH_MIN_DELTA_T: f64 = 600.0;

/// Maximale Höhendifferenz bei CDH [m].
///
/// AGC-Kriterium: diff_alt ≤ 50 NM ≈ 92 600 m.
/// AGC source: `Comanche055/P32-P33,_P72-P73.agc`, P33-Routine.
const CDH_MAX_DIFF_ALT: f64 = 92_600.0;

// ---------------------------------------------------------------------------
// P32 — Co-Elliptic Sequence Initiation (CSI)
// ---------------------------------------------------------------------------

/// Eingabedaten für P32 (CSI).
///
/// # Einheiten
/// | Feld         | Einheit | Bedeutung                                           |
/// |--------------|---------|-----------------------------------------------------|
/// | `sv_active`  | m / m/s | SV des aktiven Fahrzeugs zum CSI-Zeitpunkt (ECI)   |
/// | `sv_passive` | m / m/s | SV des passiven Fahrzeugs zum CSI-Zeitpunkt (ECI)  |
/// | `t_csi_to_cdh`| s      | Zeit von CSI bis CDH-Manöver [s]                   |
/// | `body`       | —       | Zentralkörper                                       |
///
/// AGC source: `Comanche055/P32-P33,_P72-P73.agc`, P32-Routine.
#[derive(Copy, Clone, Debug)]
pub struct CsiInput {
    /// Zustandsvektor des aktiven Fahrzeugs zum CSI-Zeitpunkt (ECI J2000, m / m/s).
    pub sv_active: StateVector,
    /// Zustandsvektor des passiven Fahrzeugs zum CSI-Zeitpunkt (ECI J2000, m / m/s).
    pub sv_passive: StateVector,
    /// Zeitspanne von CSI-Zündung bis CDH-Zündung [s].
    pub t_csi_to_cdh: f64,
    /// Zentralkörper.
    pub body: CentralBody,
}

/// Ausgabedaten von P32 (CSI).
///
/// # Einheiten
/// | Feld            | Einheit | Bedeutung                                       |
/// |-----------------|---------|-------------------------------------------------|
/// | `delta_v_csi`   | m/s     | Δv am CSI-Zündpunkt (ECI J2000, horizontal)    |
/// | `t1_to_t2`      | s       | Zeit von CSI bis CDH [s]                        |
///
/// AGC source: `Comanche055/P32-P33,_P72-P73.agc`, P32-Routine.
#[derive(Copy, Clone, Debug)]
pub struct CsiOutput {
    /// Δv-Impuls am CSI-Zündpunkt in m/s (ECI J2000).
    ///
    /// Die radiale Komponente ist null (Manöver in der Horizontalebene).
    pub delta_v_csi: Vec3,
    /// Zeitspanne von CSI bis CDH [s].
    pub t1_to_t2: f64,
}

/// Berechnet das CSI-Manöver (P32).
///
/// Algorithmus:
/// 1. CDH-Zeitpunkt durch Propagation des passiven Fahrzeugs bestimmen
///    (`t_csi_to_cdh` Sekunden voraus).
/// 2. Passives Fahrzeug zum CDH-Zeitpunkt propagieren → `sv_passive_cdh`.
/// 3. Aktives Fahrzeug propagieren → `sv_active_cdh`.
/// 4. Gewünschte Höhe am CDH: mittlere Höhe beider Fahrzeuge.
/// 5. Δv_CSI: along-track Korrektur, sodass das aktive Fahrzeug am CDH-Punkt
///    den richtigen Radius erreicht.
///    Vereinfacht (ko-elliptische Näherung):
///    ```text
///    v_circ_target = √(μ / r_passive_cdh)   (Kreisbahngeschwindigkeit)
///    dv_along = v_circ_target − |v_active_at_csi| (Magnitude-Differenz)
///    delta_v_csi = dv_along · v̂_active      (in Flugrichtung)
///    ```
/// 6. Überprüfung: `t_csi_to_cdh ≥ CSI_CDH_MIN_DELTA_T` (Alarm 0x0320 bei Verletzung).
///
/// # Parameter
/// - `input`: CSI-Eingabedaten
/// - `alarm`: Alarmzustand
///
/// # Rückgabe
/// `Some(CsiOutput)` bei Erfolg, `None` bei Alarm.
///
/// # AGC-Quelle
/// `Comanche055/P32-P33,_P72-P73.agc`, P32-Routine + CSI/A-Subroutine.
pub fn csi_compute(input: &CsiInput, alarm: &mut AlarmState) -> Option<CsiOutput> {
    if input.t_csi_to_cdh < CSI_CDH_MIN_DELTA_T {
        raise(alarm, 0x0320);
        return None;
    }

    let mu = input.body.mu();

    // Aktives und passives Fahrzeug zum CDH-Zeitpunkt propagieren
    let sv_active_cdh = kepler_step(input.sv_active, input.t_csi_to_cdh, mu, alarm);
    if alarm.last().is_some() {
        return None;
    }
    let sv_passive_cdh = kepler_step(input.sv_passive, input.t_csi_to_cdh, mu, alarm);
    if alarm.last().is_some() {
        return None;
    }

    // Radius des passiven Fahrzeugs am CDH-Punkt
    let r_passive_cdh = sv_passive_cdh.position.magnitude();
    if r_passive_cdh < 1.0 {
        raise(alarm, 0x0320);
        return None;
    }

    // Geschwindigkeit des aktiven Fahrzeugs am CDH-Punkt
    let v_active_cdh_mag = sv_active_cdh.velocity.magnitude();
    if v_active_cdh_mag < 1.0 {
        raise(alarm, 0x0320);
        return None;
    }

    // Kreisbahngeschwindigkeit am passiven CDH-Radius
    let v_circ_passive = libm::sqrt(mu / r_passive_cdh);

    // Δv: Differenz zwischen Kreisbahngeschwindigkeit am passiven Radius
    // und der aktuellen Geschwindigkeitsmagnitude des aktiven Fahrzeugs am CDH
    let dv_mag = v_circ_passive - v_active_cdh_mag;

    // Richtung: entlang des Geschwindigkeitsvektors des aktiven Fahrzeugs am CSI-Zeitpunkt
    let v_active_csi = input.sv_active.velocity;
    let v_active_csi_hat = v_active_csi.normalize();
    let delta_v_csi = match v_active_csi_hat {
        Some(vhat) => vhat.scale(dv_mag),
        None => {
            raise(alarm, 0x0320);
            return None;
        }
    };

    Some(CsiOutput {
        delta_v_csi,
        t1_to_t2: input.t_csi_to_cdh,
    })
}

// ---------------------------------------------------------------------------
// P33 — Constant Delta Height (CDH)
// ---------------------------------------------------------------------------

/// Eingabedaten für P33 (CDH).
///
/// # Einheiten
/// | Feld              | Einheit | Bedeutung                                          |
/// |-------------------|---------|----------------------------------------------------|
/// | `sv_active_cdh`   | m / m/s | SV des aktiven Fahrzeugs zum CDH-Zeitpunkt (ECI)  |
/// | `sv_passive_cdh`  | m / m/s | SV des passiven Fahrzeugs zum CDH-Zeitpunkt (ECI) |
/// | `body`            | —       | Zentralkörper                                      |
///
/// AGC source: `Comanche055/P32-P33,_P72-P73.agc`, P33-Routine.
#[derive(Copy, Clone, Debug)]
pub struct CdhInput {
    /// Zustandsvektor des aktiven Fahrzeugs zum CDH-Zeitpunkt (ECI J2000, m / m/s).
    pub sv_active_cdh: StateVector,
    /// Zustandsvektor des passiven Fahrzeugs zum CDH-Zeitpunkt (ECI J2000, m / m/s).
    pub sv_passive_cdh: StateVector,
    /// Zentralkörper.
    pub body: CentralBody,
}

/// Ausgabedaten von P33 (CDH).
///
/// # Einheiten
/// | Feld            | Einheit | Bedeutung                                       |
/// |-----------------|---------|-------------------------------------------------|
/// | `delta_v_cdh`   | m/s     | Δv am CDH-Zündpunkt (ECI J2000)                |
/// | `diff_alt`      | m       | Höhendifferenz zwischen passivem und aktivem SV |
///
/// AGC source: `Comanche055/P32-P33,_P72-P73.agc`, P33-Routine.
#[derive(Copy, Clone, Debug)]
pub struct CdhOutput {
    /// Δv-Impuls am CDH-Zündpunkt in m/s (ECI J2000).
    pub delta_v_cdh: Vec3,
    /// Höhendifferenz `|r_passive| − |r_active|` am CDH-Punkt [m].
    pub diff_alt: f64,
}

/// Berechnet das CDH-Manöver (P33).
///
/// Das CDH-Manöver gleicht die Geschwindigkeit des aktiven Fahrzeugs so an,
/// dass die radiale Geschwindigkeitskomponente annulliert wird — was einer
/// kreisförmigen Bahn auf der aktuellen Höhe entspricht.
///
/// Algorithmus:
/// 1. `diff_alt = |r_passive| − |r_active|` (Höhendifferenz)
/// 2. Radialer Anteil von `v_active` entfernen:
///    ```text
///    r_hat = r_active / |r_active|
///    v_radial = (v_active · r_hat) * r_hat
///    v_tangential = v_active − v_radial
///    delta_v_cdh = −v_radial   (Radialkomponente auf 0 bringen)
///    ```
/// 3. Überprüfung: `|diff_alt| ≤ CDH_MAX_DIFF_ALT` (Alarm 0x0330 bei Verletzung).
///
/// # Parameter
/// - `input`: CDH-Eingabedaten
/// - `alarm`: Alarmzustand
///
/// # Rückgabe
/// `Some(CdhOutput)` bei Erfolg, `None` bei Alarm.
///
/// # AGC-Quelle
/// `Comanche055/P32-P33,_P72-P73.agc`, P33-Routine + CDHMVR-Subroutine.
pub fn cdh_compute(input: &CdhInput, alarm: &mut AlarmState) -> Option<CdhOutput> {
    let r_active = input.sv_active_cdh.position;
    let v_active = input.sv_active_cdh.velocity;
    let r_passive = input.sv_passive_cdh.position;

    let r_active_mag = r_active.magnitude();
    let r_passive_mag = r_passive.magnitude();

    if r_active_mag < 1.0 || r_passive_mag < 1.0 {
        raise(alarm, 0x0330);
        return None;
    }

    // Höhendifferenz
    let diff_alt = r_passive_mag - r_active_mag;

    // Grenzüberprüfung
    if libm::fabs(diff_alt) > CDH_MAX_DIFF_ALT {
        raise(alarm, 0x0330);
        return None;
    }

    // Radiale Einheitsrichtung des aktiven Fahrzeugs
    let r_hat = r_active.normalize()?;

    // Radiale Geschwindigkeitskomponente
    let v_radial_mag = v_active.dot(r_hat);
    let v_radial = r_hat.scale(v_radial_mag);

    // CDH-Manöver: radiale Komponente aufheben
    let delta_v_cdh = v_radial.scale(-1.0);

    Some(CdhOutput {
        delta_v_cdh,
        diff_alt,
    })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{cdh_compute, csi_compute, CdhInput, CsiInput};
    use crate::alarm::AlarmState;
    use crate::navigation::orbital::CentralBody;
    use crate::navigation::state_vector::StateVector;
    use crate::types::{Met, Vec3};

    fn leo_sv(r_m: f64) -> StateVector {
        let mu = 3.986_004_418e14_f64;
        let v = libm::sqrt(mu / r_m);
        StateVector::new(
            Vec3::new(r_m, 0.0, 0.0),
            Vec3::new(0.0, v, 0.0),
            Met::from_seconds(0.0),
        )
    }

    // -----------------------------------------------------------------------
    // Test 1: CSI — nominaler Ablauf
    // -----------------------------------------------------------------------
    #[test]
    fn csi_compute_nominal() {
        // Aktives Fahrzeug 20 km unter passivem (Annäherung von unten)
        let r_active = 6_758_000.0_f64; // 380 km
        let r_passive = 6_778_000.0_f64; // 400 km

        let sv_active = leo_sv(r_active);
        let sv_passive = leo_sv(r_passive);

        let input = CsiInput {
            sv_active,
            sv_passive,
            t_csi_to_cdh: 1800.0, // 30 Minuten
            body: CentralBody::Earth,
        };
        let mut alarm = AlarmState::new();
        let result = csi_compute(&input, &mut alarm);
        assert!(result.is_some(), "CSI soll Some liefern");
        let out = result.unwrap();
        assert!(alarm.last().is_none(), "Kein Alarm erwartet");
        assert!(
            out.t1_to_t2 >= 600.0,
            "T1→T2 soll ≥ 600 s sein: {}",
            out.t1_to_t2
        );
        // Δv soll von Null verschieden sein
        assert!(
            out.delta_v_csi.magnitude() > 0.1,
            "Δv_CSI soll > 0: {}",
            out.delta_v_csi.magnitude()
        );
    }

    // -----------------------------------------------------------------------
    // Test 2: CSI — zu kurze Zeitspanne → Alarm 0x0320
    // -----------------------------------------------------------------------
    #[test]
    fn csi_compute_too_short_alarm() {
        let sv = leo_sv(6_778_000.0);
        let input = CsiInput {
            sv_active: sv,
            sv_passive: sv,
            t_csi_to_cdh: 300.0, // < 600 s minimum
            body: CentralBody::Earth,
        };
        let mut alarm = AlarmState::new();
        let result = csi_compute(&input, &mut alarm);
        assert!(result.is_none(), "Zu kurze Zeit soll None ergeben");
        assert_eq!(alarm.last(), Some(0x0320), "Alarm 0x0320 erwartet");
    }

    // -----------------------------------------------------------------------
    // Test 3: CDH — nominaler Ablauf (Kreisbahn → Δv ≈ 0)
    // -----------------------------------------------------------------------
    #[test]
    fn cdh_compute_circular_orbit_no_radial() {
        // Beide Fahrzeuge auf Kreisbahn: keine radiale Geschwindigkeit → Δv ≈ 0
        let r_active = 6_778_000.0_f64;
        let r_passive = 6_798_000.0_f64; // 20 km höher

        let sv_active = leo_sv(r_active);
        let sv_passive = leo_sv(r_passive);

        let input = CdhInput {
            sv_active_cdh: sv_active,
            sv_passive_cdh: sv_passive,
            body: CentralBody::Earth,
        };
        let mut alarm = AlarmState::new();
        let result = cdh_compute(&input, &mut alarm);
        assert!(result.is_some(), "CDH soll Some liefern");
        let out = result.unwrap();
        assert!(alarm.last().is_none(), "Kein Alarm erwartet");

        // Auf Kreisbahn ist v_radial = 0 → Δv ≈ 0
        assert!(
            out.delta_v_cdh.magnitude() < 1e-6,
            "Δv_CDH soll ≈ 0 auf Kreisbahn: {}",
            out.delta_v_cdh.magnitude()
        );
        // Höhendifferenz: 20 km
        assert!(
            (out.diff_alt - 20_000.0).abs() < 1.0,
            "diff_alt soll ≈ 20000 m: {}",
            out.diff_alt
        );
    }

    // -----------------------------------------------------------------------
    // Test 4: CDH — radiale Geschwindigkeit → Δv annulliert sie
    // -----------------------------------------------------------------------
    #[test]
    fn cdh_compute_removes_radial_velocity() {
        let r_active = 6_778_000.0_f64;
        let mu = 3.986_004_418e14_f64;
        let v_circ = libm::sqrt(mu / r_active);

        // Aktives Fahrzeug mit 50 m/s radialer Geschwindigkeit (nach außen)
        let sv_active = StateVector::new(
            Vec3::new(r_active, 0.0, 0.0),
            Vec3::new(50.0, v_circ, 0.0), // 50 m/s radial
            Met::from_seconds(0.0),
        );
        let sv_passive = leo_sv(r_active + 20_000.0);

        let input = CdhInput {
            sv_active_cdh: sv_active,
            sv_passive_cdh: sv_passive,
            body: CentralBody::Earth,
        };
        let mut alarm = AlarmState::new();
        let result = cdh_compute(&input, &mut alarm);
        assert!(result.is_some(), "CDH soll Some liefern");
        let out = result.unwrap();

        // delta_v_cdh soll die radiale Komponente annullieren (≈ -50 m/s in x)
        assert!(
            (out.delta_v_cdh.x - (-50.0)).abs() < 0.1,
            "Δv_CDH.x soll ≈ -50 m/s: {}",
            out.delta_v_cdh.x
        );
    }

    // -----------------------------------------------------------------------
    // Test 5: CDH — zu große Höhendifferenz → Alarm 0x0330
    // -----------------------------------------------------------------------
    #[test]
    fn cdh_compute_diff_alt_too_large_alarm() {
        let r_active = 6_778_000.0_f64;
        // 200 km Differenz > 92 600 m Maximum
        let r_passive = r_active + 200_000.0;

        let sv_active = leo_sv(r_active);
        let sv_passive = leo_sv(r_passive);

        let input = CdhInput {
            sv_active_cdh: sv_active,
            sv_passive_cdh: sv_passive,
            body: CentralBody::Earth,
        };
        let mut alarm = AlarmState::new();
        let result = cdh_compute(&input, &mut alarm);
        assert!(result.is_none(), "Zu große Differenz soll None ergeben");
        assert_eq!(alarm.last(), Some(0x0330), "Alarm 0x0330 erwartet");
    }
}
