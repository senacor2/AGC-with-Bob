//! P20 / P22 — Rendezvous-Navigations-Programme.
//!
//! P20 (Rendezvous Navigation Program) aktualisiert den CSM-Zustandsvektor über
//! einen skalaren Kalman-Update-Schritt, basierend auf VHF-Ranging-Messungen
//! (`VhfRangingHal`). Die Schleife läuft phasenweise und ist über die Phase-Table
//! restart-sicher.
//!
//! P22 (Landmark Tracking) berechnet den Middle Gimbal Angle (MGA) aus dem
//! aktuellen Positions-/Geschwindigkeitsvektor — das Maß für den Winkel zwischen
//! der Orbitebene und dem Body-Z-Axis-Referenz.
//!
//! # Einheiten
//! - Positions: m (ECI J2000)
//! - Geschwindigkeit: m/s (ECI J2000)
//! - MGA: Radiant
//! - VHF-Range: Rohablesewert aus `VhfRangingHal::read_range()` in 1/128 NM (≈ 14.4 m)
//!
//! # AGC-Quelle
//! `Comanche055/P20-P25.agc`, PROG20-Routine und PROG22-Routine.

use crate::alarm::{raise, AlarmState};
use crate::hal::VhfRangingHal;
use crate::navigation::measurement::incorporate_measurement;
use crate::navigation::state_vector::StateVector;
use crate::restart::phase_table::PhaseTable;
use crate::types::Vec3;

// ---------------------------------------------------------------------------
// VHF-Ranging-Maßstab
// ---------------------------------------------------------------------------

/// Umrechnungsfaktor VHF-Range-Rohablesewert → Meter.
///
/// 1 LSB = 1 NM / 2048 ≈ 0.9043 m (12-Bit-Wert, Vollskala ≈ 2000 NM).
/// AGC source: `Comanche055/R31.agc`, RANGERD routine.
const VHF_RANGE_LSB_M: f64 = 1852.0 / 2048.0; // ≈ 0.9043 m

// ---------------------------------------------------------------------------
// P20State
// ---------------------------------------------------------------------------

/// Zustand des Rendezvous-Navigationsprogramms P20.
///
/// # AGC-Quelle
/// `Comanche055/P20-P25.agc`, PROG20-Routine.
#[derive(Copy, Clone, Debug)]
pub struct P20State {
    /// Aktuelle Programmphase.
    ///
    /// | Wert | Bedeutung                              |
    /// |------|----------------------------------------|
    /// | 2    | Initialisierungsphase                  |
    /// | 3    | Tracking-Schleife (Kalman-Update)      |
    /// | 0    | Fertig / Idle                          |
    pub phase: u8,

    /// Aktueller propagierter Zustandsvektor des aktiven Fahrzeugs (ECI J2000, m / m/s).
    pub sv_active: StateVector,

    /// Tracking-Mark-Zähler (entspricht `TRKMKCNT` im AGC).
    pub mark_count: u16,

    /// Positionsfehlervektor δx aus dem Kalman-Filter [m].
    ///
    /// Wird bei jeder neuen Ranging-Messung aktualisiert.
    pub delta_x: Vec3,
}

impl P20State {
    /// Erstellt einen neuen `P20State` mit Initialisierungsphase.
    ///
    /// # Parameter
    /// - `sv`: Ausgangszustandsvektor des aktiven Fahrzeugs
    pub fn new(sv: StateVector) -> Self {
        Self {
            phase: 2,
            sv_active: sv,
            mark_count: 0,
            delta_x: Vec3::ZERO,
        }
    }
}

// ---------------------------------------------------------------------------
// p20_step
// ---------------------------------------------------------------------------

/// Führt einen P20-Rendezvous-Navigationsschritt aus.
///
/// # Phase 2 — Initialisierung
/// Setzt Zähler auf Null, Phase-Table-Gruppe 6 Checkpoint 3, wechselt zu Phase 3.
///
/// # Phase 3 — Tracking-Schleife
/// Liest VHF-Range, bildet Mess-Jacobi-Zeile als Richtungseinheitsvektor zur
/// geschätzten Passivfahrzeug-Position und führt einen skalaren Kalman-Update
/// durch. Inkrementiert `mark_count`.
///
/// # Parameter
/// - `state`: P20-Programmzustand (wird in-place aktualisiert)
/// - `sv_passive`: aktuell propagierter Zustandsvektor des passiven Fahrzeugs
///   (Ground-Upload oder vorheriger P20-Schritt) — wird für die Richtungsberechnung benötigt
/// - `ranging`: VHF-Ranging-HAL (optional; wenn nicht verfügbar → Alarm 0x0201)
/// - `pt`: Phase-Table für Restart-Checkpoints
/// - `alarm`: Alarmzustand
///
/// # Rückgabe
/// `Some(())` bei Erfolg, `None` bei Alarm oder ungültigem Zustand.
///
/// # AGC-Quelle
/// `Comanche055/P20-P25.agc`, PROG20-Routine.
pub fn p20_step(
    state: &mut P20State,
    sv_passive: &StateVector,
    ranging: &mut impl VhfRangingHal,
    pt: &mut PhaseTable,
    alarm: &mut AlarmState,
) -> Option<()> {
    match state.phase {
        2 => {
            // Phase 2: Initialisierung
            state.mark_count = 0;
            state.delta_x = Vec3::ZERO;
            // Checkpoint setzen: nach Restart weiter in Phase 3
            pt.set_phase(6, 3)?;
            state.phase = 3;
            Some(())
        }
        3 => {
            // Phase 3: Kalman-Update via VHF-Ranging
            if !ranging.is_available() {
                raise(alarm, 0x0201);
                return None;
            }
            let raw_range = ranging.read_range()?;
            // Gemessene Distanz in Metern
            let obs_range = raw_range as f64 * VHF_RANGE_LSB_M;

            // Richtungsvektor vom aktiven zum passiven Fahrzeug
            let r_rel = sv_passive.position - state.sv_active.position;
            let r_rel_norm = r_rel.normalize();

            let h_row = match r_rel_norm {
                Some(v) => v,
                None => {
                    raise(alarm, 0x0201);
                    return None;
                }
            };

            // Geschätzte Distanz aus dem Zustandsvektor
            let est_range = r_rel.magnitude();

            // Skalarer Kalman-Update
            // w = 1.0: Einheitsmessrauschen (vereinfacht, da kein W-Matrix-Task)
            state.delta_x =
                incorporate_measurement(obs_range - est_range, h_row, 1.0, state.delta_x);

            // Positions-Update anwenden
            state.sv_active = StateVector::new(
                state.sv_active.position + state.delta_x,
                state.sv_active.velocity,
                state.sv_active.time,
            );
            // delta_x nach Anwendung zurücksetzen
            state.delta_x = Vec3::ZERO;

            state.mark_count = state.mark_count.saturating_add(1);

            // Checkpoint erneuern
            pt.set_phase(6, 3)?;

            Some(())
        }
        _ => {
            // Phase 0 oder ungültig → fertig / kein Fortschritt
            None
        }
    }
}

// ---------------------------------------------------------------------------
// p22_mga
// ---------------------------------------------------------------------------

/// Berechnet den Middle Gimbal Angle (MGA) für das Landmark-Tracking-Programm P22.
///
/// Der MGA misst den Winkel zwischen dem Drehimpulsvektor der Bahn `h = v × r`
/// und der Body-Z-Achse `[0, 0, 1]`.
///
/// ```text
/// h = v × r
/// MGA = arccos(|h_hat · ê_z|)
/// ```
///
/// # Parameter
/// - `sv`: aktueller Zustandsvektor (ECI J2000, m / m/s)
///
/// # Rückgabe
/// MGA in Radiant [0, π/2].
/// `None` wenn `|v × r| = 0` (Singularität: Geschwindigkeit parallel zu Position).
///
/// # AGC-Quelle
/// `Comanche055/P20-P25.agc`, PROG22-Routine.
pub fn p22_mga(sv: &StateVector) -> Option<f64> {
    // h = v × r (Drehimpulsrichtung)
    let h = sv.velocity.cross(sv.position);
    let h_hat = h.normalize()?;

    // Winkel zwischen h_hat und ê_z = [0, 0, 1]
    // dot(h_hat, ê_z) = h_hat.z
    let cos_mga = libm::fabs(h_hat.z).clamp(0.0, 1.0);
    Some(libm::acos(cos_mga))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{p20_step, p22_mga, P20State};
    use crate::alarm::AlarmState;
    use crate::hal::vhf_ranging::mock::MockVhfRanging;
    use crate::navigation::state_vector::StateVector;
    use crate::restart::phase_table::PhaseTable;
    use crate::types::{Met, Vec3};

    fn leo_sv() -> StateVector {
        let r = 6_778_000.0_f64;
        let mu = 3.986_004_418e14_f64;
        let v = libm::sqrt(mu / r);
        StateVector::new(
            Vec3::new(r, 0.0, 0.0),
            Vec3::new(0.0, v, 0.0),
            Met::from_seconds(0.0),
        )
    }

    // -----------------------------------------------------------------------
    // Test 1: P20 Initialisierungsphase
    // -----------------------------------------------------------------------
    #[test]
    fn p20_init_phase() {
        let sv = leo_sv();
        // Passives Fahrzeug 10 km voraus (entlang y-Achse)
        let sv_passive = StateVector::new(
            Vec3::new(sv.position.x, 10_000.0, 0.0),
            sv.velocity,
            sv.time,
        );
        let mut state = P20State::new(sv);
        let mut ranging = MockVhfRanging {
            available: false,
            range_value: None,
        };
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        assert_eq!(state.phase, 2);
        let result = p20_step(&mut state, &sv_passive, &mut ranging, &mut pt, &mut alarm);
        assert!(result.is_some(), "Phase 2 soll Some liefern");
        assert_eq!(state.phase, 3, "Nach Phase 2 soll Phase 3 aktiv sein");
        assert_eq!(state.mark_count, 0);
        assert_eq!(pt.get_phase(6), Some(3));
    }

    // -----------------------------------------------------------------------
    // Test 2: P20 Tracking-Schritt ohne Ranging → Alarm
    // -----------------------------------------------------------------------
    #[test]
    fn p20_tracking_no_ranging_alarm() {
        let sv = leo_sv();
        let sv_passive = StateVector::new(
            Vec3::new(sv.position.x, 10_000.0, 0.0),
            sv.velocity,
            sv.time,
        );
        let mut state = P20State {
            phase: 3,
            ..P20State::new(sv)
        };
        let mut ranging = MockVhfRanging {
            available: false,
            range_value: None,
        };
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        let result = p20_step(&mut state, &sv_passive, &mut ranging, &mut pt, &mut alarm);
        assert!(result.is_none(), "Ohne Ranging soll None kommen");
        assert_eq!(alarm.last(), Some(0x0201), "Alarm 0x0201 erwartet");
    }

    // -----------------------------------------------------------------------
    // Test 3: P20 Tracking-Schritt mit Ranging
    // -----------------------------------------------------------------------
    #[test]
    fn p20_tracking_step_with_ranging() {
        let sv = leo_sv();
        // Passives Fahrzeug 10 km in y-Richtung
        let dist_m = 10_000.0_f64;
        let sv_passive =
            StateVector::new(Vec3::new(sv.position.x, dist_m, 0.0), sv.velocity, sv.time);

        // VHF-Range in LSB (1 LSB ≈ 0.9043 m)
        // 10000 m / 0.9043 ≈ 11058 LSB
        let range_lsb = (dist_m / (1852.0 / 2048.0)).round() as u16;
        let mut ranging = MockVhfRanging {
            available: true,
            range_value: Some(range_lsb),
        };

        let mut state = P20State {
            phase: 3,
            ..P20State::new(sv)
        };
        let mut pt = PhaseTable::new();
        pt.set_phase(6, 3).unwrap();
        let mut alarm = AlarmState::new();

        let result = p20_step(&mut state, &sv_passive, &mut ranging, &mut pt, &mut alarm);
        assert!(result.is_some(), "Tracking-Schritt soll Some liefern");
        assert!(alarm.last().is_none(), "Kein Alarm erwartet");
        assert_eq!(state.mark_count, 1, "Mark-Count soll 1 sein");
    }

    // -----------------------------------------------------------------------
    // Test 4: P20 Phase 0 → None
    // -----------------------------------------------------------------------
    #[test]
    fn p20_phase_zero_returns_none() {
        let sv = leo_sv();
        let sv_passive = sv;
        let mut state = P20State {
            phase: 0,
            ..P20State::new(sv)
        };
        let mut ranging = MockVhfRanging {
            available: false,
            range_value: None,
        };
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        let result = p20_step(&mut state, &sv_passive, &mut ranging, &mut pt, &mut alarm);
        assert!(result.is_none(), "Phase 0 soll None liefern");
    }

    // -----------------------------------------------------------------------
    // Test 5: p22_mga — Kreisbahn in xy-Ebene → MGA = 90°
    // -----------------------------------------------------------------------
    #[test]
    fn p22_mga_equatorial_orbit() {
        // r = (r,0,0), v = (0,v,0) → h = v × r = (0,0, -r·v)
        // h_hat = (0,0,-1) → |h_hat.z| = 1 → arccos(1) = 0
        // Anmerkung: h = v × r = (v_y*r_z - v_z*r_y, v_z*r_x - v_x*r_z, v_x*r_y - v_y*r_x)
        //   = (0*0 - 0*0, 0*r - 0*0, 0*0 - v*r) = (0, 0, -v*r)
        // h_hat = (0, 0, -1) → |z| = 1 → MGA = arccos(1) = 0
        let sv = leo_sv();
        let mga = p22_mga(&sv);
        assert!(mga.is_some());
        let mga_val = mga.unwrap();
        // MGA ≈ 0 für äquatoriale Kreisbahn in der xy-Ebene
        assert!(
            mga_val < 0.01,
            "MGA für äquatoriale Bahn erwartet ≈ 0, got {mga_val}"
        );
    }

    // -----------------------------------------------------------------------
    // Test 6: p22_mga — Null-Impuls-Vektor → None
    // -----------------------------------------------------------------------
    #[test]
    fn p22_mga_zero_velocity_returns_none() {
        let sv = StateVector::new(
            Vec3::new(6_778_000.0, 0.0, 0.0),
            Vec3::ZERO,
            Met::from_seconds(0.0),
        );
        assert!(p22_mga(&sv).is_none(), "Null-Velocity soll None ergeben");
    }
}
