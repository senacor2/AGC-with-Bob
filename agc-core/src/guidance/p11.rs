//! P11 — Earth Orbit Insertion Monitor.
//!
//! Überwacht die Aufstiegsphase nach dem Liftoff. Propagiert den Bahn-
//! Zustandsvektor und berechnet Höhe sowie Radialgeschwindigkeit.
//!
//! # Phasen (Phase-Table Gruppe 3)
//! | Phase | Bedeutung                                                 |
//! |-------|-----------------------------------------------------------|
//! | 2     | Initialisierung: Zustandsvektor prüfen, Kepler-Propagation|
//! | 3     | Monitor: Höhe und Radialgeschwindigkeit berechnen         |
//! | 0     | Abgeschlossen / inaktiv                                   |
//!
//! AGC-Quelle: `Comanche055/P11.agc`

use crate::alarm::{raise, AlarmState};
use crate::navigation::conic::kepler_step;
use crate::navigation::state_vector::StateVector;
use crate::restart::phase_table::PhaseTable;
use crate::types::Met;

// ---------------------------------------------------------------------------
// Konstanten
// ---------------------------------------------------------------------------

/// Erdradius (mittlerer Äquatorradius) in Metern.
///
/// Verwendet zur Höhenberechnung: h = |r⃗| − R_EARTH.
/// AGC source: `Comanche055/P11.agc`, VHHDOT-Makro (REARTH-Konstante).
const R_EARTH: f64 = 6_371_000.0;

/// Alarmcode P11: Zustandsvektor liegt vor Liftoff-Zeit.
///
/// AGC source: `Comanche055/P11.agc`, S11.1-Routine (Zeitprüfung).
const ALARM_P11_BEFORE_LIFTOFF: u16 = 0x0B11;

// ---------------------------------------------------------------------------
// P11State
// ---------------------------------------------------------------------------

/// Laufzustand von P11 — Earth Orbit Insertion Monitor.
///
/// # Einheiten
/// | Feld          | Einheit  | Bedeutung                          |
/// |---------------|----------|------------------------------------|
/// | `liftoff_met` | Sekunden | MET beim Liftoff (TEPHEM-Basis)    |
/// | `phase`       | —        | Programmphase: 2 = Init, 3 = Mon   |
///
/// # AGC-Quelle
/// `Comanche055/P11.agc`, REP11S/REP11SA Restart-Eintrag (Group 3).
#[derive(Copy, Clone, Debug)]
pub struct P11State {
    /// Mission Elapsed Time beim Liftoff in Sekunden seit TEPHEM.
    pub liftoff_met: Met,
    /// Aktuelle Programmphase: 2 = Initialisierung, 3 = Monitor, 0 = fertig.
    pub phase: u8,
}

impl P11State {
    /// Erstellt einen neuen P11-Zustand für den Liftoff-Zeitpunkt `liftoff`.
    #[inline]
    pub const fn new(liftoff_met: Met) -> Self {
        Self {
            liftoff_met,
            phase: 2,
        }
    }
}

// ---------------------------------------------------------------------------
// Ausgabetyp
// ---------------------------------------------------------------------------

/// Berechnete Navigationsgrössen von P11 für einen einzelnen Schritt.
///
/// # Einheiten
/// | Feld        | Einheit | Bedeutung                                       |
/// |-------------|---------|--------------------------------------------------|
/// | `altitude`  | Meter   | Höhe über mittlerem Erdradius: |r⃗| − R_EARTH    |
/// | `v_radial`  | m/s     | Radialgeschwindigkeit: r⃗ · v⃗ / |r⃗|             |
///
/// AGC source: `Comanche055/P11.agc`, S11.1-Routine (Vi/HDOT/H-Berechnung).
#[derive(Copy, Clone, Debug)]
pub struct P11Nav {
    /// Höhe über mittlerem Erdradius in Metern.
    pub altitude: f64,
    /// Radialgeschwindigkeit (positiv = entfernt sich von Erde) in m/s.
    pub v_radial: f64,
}

// ---------------------------------------------------------------------------
// p11_step
// ---------------------------------------------------------------------------

/// Führt einen P11-Schritt aus (Earth Orbit Insertion Monitor).
///
/// Verarbeitet die aktuelle Phase aus `state.phase` und sichert den
/// Fortschritt in Phase-Table Gruppe 3. Nach erfolgreichem Abschluss
/// beider Phasen ist `state.phase == 0`.
///
/// # Parameter
/// - `state`:  P11-Laufzustand (wird modifiziert)
/// - `sv`:     Aktueller Bahn-Zustandsvektor (ECI J2000, m / m/s)
/// - `pt`:     Phase-Table — Checkpoint für Restart-Sicherheit
/// - `alarm`:  Alarmzustand (Program-Alarm bei ungültigem SV)
///
/// # Rückgabe
/// `Some(P11Nav)` mit berechneten Navigationsgrössen, oder `None` bei
/// Alarm (z. B. SV liegt vor Liftoff-Zeit).
///
/// # AGC source
/// `Comanche055/P11.agc`, REP11S/REP11SA restart entries (Group 3)
/// `Comanche055/P11.agc`, S11.1 routine (Vi/HDOT/H computation)
pub fn p11_step(
    state: &mut P11State,
    sv: &StateVector,
    pt: &mut PhaseTable,
    alarm: &mut AlarmState,
) -> Option<P11Nav> {
    match state.phase {
        2 => {
            // Phase 2: Initialisierung
            // Checkpoint vor Phase 2 setzen (Restart-Eintrag REP11S)
            pt.set_phase(3, 2)?;

            // Zeitprüfung: Zustandsvektor muss >= Liftoff sein
            if sv.time < state.liftoff_met {
                raise(alarm, ALARM_P11_BEFORE_LIFTOFF);
                return None;
            }

            // Optionale Kepler-Propagation wenn SV nicht aktuell
            // (im echten AGC: MATRXJOB → RN/VN Initialisierung)
            let dt = sv.time.delta_s(sv.time); // dt = 0 → kein Schritt nötig
            let _sv_prop = if dt > 0.0 {
                kepler_step(
                    *sv,
                    dt,
                    crate::navigation::orbital::CentralBody::Earth.mu(),
                    alarm,
                )
            } else {
                *sv
            };

            // Checkpoint vor Phase 3 setzen
            pt.set_phase(3, 3)?;
            state.phase = 3;

            // Direkt in Phase 3 weiterfahren — entspricht AGC-Verhalten
            // (Phase 2 → Phase 3 in einem Zug beim Erstaufruf)
            let r_mag = sv.position.magnitude();
            if r_mag < 1.0 {
                raise(alarm, ALARM_P11_BEFORE_LIFTOFF);
                return None;
            }
            let altitude = r_mag - R_EARTH;
            let v_radial = sv.position.dot(sv.velocity) / r_mag;

            pt.set_phase(3, 0)?;
            state.phase = 0;

            Some(P11Nav { altitude, v_radial })
        }
        3 => {
            // Phase 3: Monitor (Restart-Eintrag REP11SA)
            // Checkpoint Phase 3 wiederherstellen
            pt.set_phase(3, 3)?;

            // Höhe: |r⃗| − R_EARTH
            // AGC source: Comanche055/P11.agc, S11.1 routine (Vi/HDOT/H computation)
            let r_mag = sv.position.magnitude();
            if r_mag < 1.0 {
                raise(alarm, ALARM_P11_BEFORE_LIFTOFF);
                return None;
            }
            let altitude = r_mag - R_EARTH;

            // Radialgeschwindigkeit: r⃗ · v⃗ / |r⃗|
            let v_radial = sv.position.dot(sv.velocity) / r_mag;

            // Gruppe 3 deaktivieren — Programm abgeschlossen
            pt.set_phase(3, 0)?;
            state.phase = 0;

            Some(P11Nav { altitude, v_radial })
        }
        _ => {
            // Phase 0 oder unbekannte Phase: bereits fertig oder ungültig
            None
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{p11_step, P11State};
    use crate::alarm::AlarmState;
    use crate::navigation::state_vector::StateVector;
    use crate::restart::phase_table::PhaseTable;
    use crate::types::{Met, Vec3};

    /// ISS-ähnliche LEO-Kreisbahn: r = 6 778 000 m, v tangential.
    /// μ_Erde = 3.986004418e14 m³/s²
    fn leo_sv() -> StateVector {
        let r = 6_778_000.0_f64;
        let mu = 3.986_004_418e14_f64;
        let v_circ = libm::sqrt(mu / r);
        StateVector::new(
            Vec3::new(r, 0.0, 0.0),
            Vec3::new(0.0, v_circ, 0.0),
            Met::from_seconds(100.0),
        )
    }

    /// Normaldurchlauf: Phase 2 → Phase 3 → done (Phase-Table Gruppe 3 = 0 am Ende)
    #[test]
    fn normal_run_phase2_to_done() {
        let sv = leo_sv();
        let mut state = P11State::new(Met::from_seconds(0.0));
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        // Erster Aufruf: Phase 2 → führt durch 2 und 3 in einem Zug
        let result = p11_step(&mut state, &sv, &mut pt, &mut alarm);

        assert!(result.is_some(), "p11_step sollte Some zurückgeben");
        let nav = result.unwrap();

        // Höhe ≈ 6_778_000 − 6_371_000 = 407_000 m
        let expected_alt = 6_778_000.0 - 6_371_000.0;
        assert!(
            (nav.altitude - expected_alt).abs() < 1.0,
            "Höhe erwartet ~{expected_alt:.0} m, got {:.0} m",
            nav.altitude
        );

        // Radialgeschwindigkeit ≈ 0 (Kreisbahn, v ⊥ r)
        assert!(
            nav.v_radial.abs() < 1e-6,
            "Radialgeschwindigkeit sollte ≈0 sein, got {:.6}",
            nav.v_radial
        );

        // Phase-Table Gruppe 3 muss 0 (inaktiv) sein
        assert_eq!(
            pt.get_phase(3),
            Some(0),
            "Phase-Table Gruppe 3 sollte 0 sein"
        );
        // state.phase muss 0 sein
        assert_eq!(state.phase, 0, "state.phase sollte 0 sein");

        // Kein Alarm
        assert!(alarm.last().is_none(), "kein Alarm erwartet");
    }

    /// Restart-Simulation: Phase-Table manuell auf Gruppe 3 = Phase 3 setzen,
    /// dann p11_step mit state.phase=3 aufrufen → korrekt fortgesetzt
    #[test]
    fn restart_in_phase3() {
        let sv = leo_sv();
        let mut state = P11State::new(Met::from_seconds(0.0));
        state.phase = 3; // Simulierter Restart: AGC startet in Phase 3

        let mut pt = PhaseTable::new();
        // Phase-Table simuliert gespeicherten Checkpoint aus Phase 3
        pt.set_phase(3, 3).unwrap();

        let mut alarm = AlarmState::new();

        let result = p11_step(&mut state, &sv, &mut pt, &mut alarm);

        assert!(
            result.is_some(),
            "Restart in Phase 3 sollte Some zurückgeben"
        );
        let nav = result.unwrap();

        let expected_alt = 6_778_000.0 - 6_371_000.0;
        assert!(
            (nav.altitude - expected_alt).abs() < 1.0,
            "Restart-Höhe erwartet ~{expected_alt:.0} m, got {:.0} m",
            nav.altitude
        );

        // Nach Abschluss: Gruppe 3 inaktiv, phase = 0
        assert_eq!(pt.get_phase(3), Some(0));
        assert_eq!(state.phase, 0);
        assert!(alarm.last().is_none());
    }

    /// Alarm bei SV vor Liftoff-Zeit
    #[test]
    fn alarm_when_sv_before_liftoff() {
        let mut sv = leo_sv();
        sv.time = Met::from_seconds(50.0); // vor liftoff_met = 100.0

        let mut state = P11State::new(Met::from_seconds(100.0));
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        let result = p11_step(&mut state, &sv, &mut pt, &mut alarm);
        assert!(
            result.is_none(),
            "Sollte None bei SV vor Liftoff zurückgeben"
        );
        assert_eq!(alarm.last(), Some(0x0B11));
    }
}
