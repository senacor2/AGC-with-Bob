//! P40, P41, P47 — SPS-Burn-Programme.
//!
//! P40 führt den SPS-Haupttriebwerksbrand durch (Ignition → Burn → Cutoff).
//! P41 ist der Gimbal-Motor-Test vor dem Burn. P47 berechnet das Post-Burn
//! Trim-Manöver aus dem IMU-Δv-Restfehler.
//!
//! # Phasenmodell P40 (Phase-Table Gruppe 4)
//! | Phase | Bedeutung                                                      |
//! |-------|----------------------------------------------------------------|
//! | 2     | Pre-Ignition: Brennzeit berechnen, Triebwerk zünden            |
//! | 3     | Burn: Zeit akkumulieren, auf Cutoff-Kriterium warten           |
//! | 4     | Cutoff: Triebwerk abschalten, Phase-Table zurücksetzen         |
//! | 0     | Abgeschlossen / inaktiv                                        |
//!
//! # Phasenmodell P41 (Phase-Table Gruppe 4)
//! | Phase | Bedeutung                                                      |
//! |-------|----------------------------------------------------------------|
//! | 2     | Pitch-Gimbal-Sequenz (+2°, −2°, 0°)                            |
//! | 3     | Yaw-Gimbal-Sequenz  (+2°, −2°, 0°)                             |
//! | 4     | Trim: Gimbals auf 0 setzen                                     |
//! | 0     | Abgeschlossen / inaktiv                                        |
//!
//! # AGC-Quelle
//! `Comanche055/P40-P47.agc`

use crate::alarm::{raise, AlarmState};
use crate::hal::sps::SpsHal;
use crate::navigation::powered_flight::burn_time;
use crate::restart::phase_table::PhaseTable;

// ---------------------------------------------------------------------------
// Konstanten
// ---------------------------------------------------------------------------

/// SPS-Nenn-Isp (spezifischer Impuls) [s].
///
/// Wert aus AGC-Paddaten: 314.5 s für das SPS-Triebwerk (AJ10-137).
///
/// AGC source: `Comanche055/P40-P47.agc`, FENG / EMDOT-Konstanten.
pub const SPS_ISP: f64 = 314.5;

/// SPS-Nennschub [N].
///
/// 20 500 lb-Schub = 91 188 N (Nominalbetrieb). Hier als Annäherung: 97 855 N.
/// Im echten AGC pad-geladen als FENG (20 500 lbs).
///
/// AGC source: `Comanche055/P40-P47.agc`, FENG-Konstante (9.1188544 B-7 M.Newtons/E4).
pub const SPS_THRUST_N: f64 = 97_855.0;

/// Alarmcode P40: Ungültige Eingabeparameter für die Brennzeit-Berechnung.
///
/// Wird ausgelöst, wenn `burn_time()` None zurückgibt (z.B. delta_v ≤ 0).
///
/// AGC source: `Comanche055/P40-P47.agc`, P40CSM / S40.1-Routine.
const ALARM_P40_BAD_PARAMS: u16 = 0x0440;

/// Gimbal-Testausschlag in TVC-LSB.
///
/// 1 LSB ≈ 85.41 Bogensekunden (≈ 0.023°). 84 LSB ≈ +2°.
///
/// AGC source: `Comanche055/P40-P47.agc`, S40.6 GIMDTEST +2ACTDEG-Konstante.
const GIMBAL_TEST_STEP: i16 = 84;

// ---------------------------------------------------------------------------
// P40State — SPS Burn
// ---------------------------------------------------------------------------

/// Laufzustand von P40 — SPS-Haupttriebwerksbrand.
///
/// # Einheiten
/// | Feld              | Einheit  | Bedeutung                                      |
/// |-------------------|----------|------------------------------------------------|
/// | `delta_v_target`  | m/s      | Zielbetrag der Geschwindigkeitsänderung        |
/// | `isp`             | s        | spez. Impuls des Triebwerks (SPS: 314.5 s)     |
/// | `thrust`          | N        | Schub des Triebwerks (SPS: 97 855 N)           |
/// | `mass`            | kg       | Startmasse vor dem Burn                        |
/// | `t_burn`          | s        | berechnete Brenndauer (aus Tsiolkovsky-Gl.)    |
/// | `t_elapsed`       | s        | seit Ignition verstrichene Zeit                |
///
/// # AGC-Quelle
/// `Comanche055/P40-P47.agc`, P40CSM / IGNITION / ENGINOFF / SPSOFF-Routinen.
#[derive(Copy, Clone, Debug)]
pub struct P40State {
    /// Aktuelle Programmphase (2 = Pre-Ignition, 3 = Burn, 4 = Cutoff, 0 = done).
    pub phase: u8,
    /// Gewünschte Δv-Größe [m/s] — muss > 0.
    pub delta_v_target: f64,
    /// Spezifischer Impuls des Triebwerks [s].
    pub isp: f64,
    /// Triebwerksschub [N].
    pub thrust: f64,
    /// Fahrzeugmasse vor dem Burn [kg].
    pub mass: f64,
    /// Berechnete Brenndauer [s] — 0.0 bis Phase 2 abgeschlossen.
    pub t_burn: f64,
    /// Seit Ignition verstrichene Zeit [s] — akkumuliert in Phase 3.
    pub t_elapsed: f64,
    /// Akkumuliertes Delta-V seit Ignition [m/s] — DVTOTAL (N40 R3).
    pub dv_accumulated: f64,
}

impl P40State {
    /// Erstellt einen neuen P40-Zustand.
    ///
    /// - `delta_v_target`: gewünschtes Δv [m/s] — muss > 0.
    /// - `isp`:            spez. Impuls [s] (verwende [`SPS_ISP`]).
    /// - `thrust`:         Schub [N] (verwende [`SPS_THRUST_N`]).
    /// - `mass`:           Startmasse [kg] — muss > 0.
    #[inline]
    pub const fn new(delta_v_target: f64, isp: f64, thrust: f64, mass: f64) -> Self {
        Self {
            phase: 2,
            delta_v_target,
            isp,
            thrust,
            mass,
            t_burn: 0.0,
            t_elapsed: 0.0,
            dv_accumulated: 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// p40_step
// ---------------------------------------------------------------------------

/// Führt einen P40-Schritt aus (SPS-Burn-Phasenzustand).
///
/// # Phasen
/// - **Phase 2** (Pre-Ignition): Berechnet die Brenndauer mit der
///   Tsiolkovsky-Gleichung (`burn_time()`), zündet das Triebwerk,
///   setzt Phase-Table Gruppe 4 Checkpoint 3.
/// - **Phase 3** (Burn): Akkumuliert die verstrichene Zeit um `dt`.
///   Sobald `t_elapsed ≥ t_burn`, wird auf Phase 4 gewechselt.
/// - **Phase 4** (Cutoff): Schaltet das Triebwerk ab, setzt
///   Phase-Table Gruppe 4 auf 0, geht in Phase 0.
/// - **Phase 0** (done): Gibt `None` zurück.
///
/// # Parameter
/// - `state`:  P40-Laufzustand.
/// - `sps`:    SPS-Hardware-Abstraktion.
/// - `pt`:     Phase-Table für Restart-Checkpoints.
/// - `alarm`:  Alarm-Zustand.
/// - `dt`:     Zeitschritt [s] — relevant für Phase 3.
///
/// # Rückgabe
/// `Some(())` bei erfolgreichem Phasenabschluss, `None` bei Alarm oder done.
///
/// # AGC-Quelle
/// `Comanche055/P40-P47.agc`, P40CSM / IGNITION / ENGINOFF / SPSOFF.
pub fn p40_step<S: SpsHal>(
    state: &mut P40State,
    sps: &mut S,
    pt: &mut PhaseTable,
    alarm: &mut AlarmState,
    dt: f64,
) -> Option<()> {
    match state.phase {
        2 => {
            // Phase 2: Brennzeit berechnen, Triebwerk zünden.
            match burn_time(state.delta_v_target, state.isp, state.thrust, state.mass) {
                None => {
                    raise(alarm, ALARM_P40_BAD_PARAMS);
                    None
                }
                Some((t_burn, _m_final)) => {
                    state.t_burn = t_burn;
                    sps.set_engine(true);
                    // Checkpoint: Phase 3 gesichert (Restart startet in Burn-Phase)
                    let _ = pt.set_phase(4, 3);
                    state.phase = 3;
                    Some(())
                }
            }
        }
        3 => {
            // Phase 3: Burn läuft — Zeit akkumulieren, Δv aufaddieren, auf Cutoff warten.
            state.t_elapsed += dt;
            // Δv-Akkumulation: a = F/m, dv = a * dt
            if state.mass > 0.0 {
                state.dv_accumulated += (state.thrust / state.mass) * dt;
            }
            if state.t_elapsed >= state.t_burn {
                state.phase = 4;
            }
            Some(())
        }
        4 => {
            // Phase 4: Cutoff — Triebwerk abschalten, Zustand abschließen.
            sps.set_engine(false);
            let _ = pt.set_phase(4, 0);
            state.phase = 0;
            Some(())
        }
        _ => {
            // Phase 0 oder ungültig: Burn ist abgeschlossen.
            None
        }
    }
}

// ---------------------------------------------------------------------------
// P41State — Gimbal Motor Test
// ---------------------------------------------------------------------------

/// Laufzustand von P41 — Gimbal-Motor-Test.
///
/// Testet Pitch- und Yaw-Gimbal je mit einer Sequenz von +2°, −2°, 0°.
/// Nach dem Test werden beide Gimbals auf 0 getrimmt.
///
/// # AGC-Quelle
/// `Comanche055/P40-P47.agc`, P41CSM / S40.6 GIMDTEST-Routine.
#[derive(Copy, Clone, Debug)]
pub struct P41State {
    /// Aktuelle Phase (2 = Pitch-Test, 3 = Yaw-Test, 4 = Trim, 0 = done).
    pub phase: u8,
    /// Aktueller Schritt innerhalb der Phase (0, 1, 2 = +84, −84, 0 LSB).
    pub step: u8,
}

impl P41State {
    /// Erstellt einen neuen P41-Zustand (startet in Phase 2, Schritt 0).
    #[inline]
    pub const fn new() -> Self {
        Self { phase: 2, step: 0 }
    }
}

impl Default for P41State {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// p41_step
// ---------------------------------------------------------------------------

/// Führt einen P41-Schritt aus (Gimbal-Motor-Test).
///
/// Jeder Aufruf führt genau einen Schritt der aktuellen Testphase aus.
/// Die Sequenz lautet: Schritt 0 → +84 LSB, Schritt 1 → −84 LSB,
/// Schritt 2 → 0 LSB. Nach drei Schritten wechselt die Phase.
///
/// # Phasen
/// - **Phase 2**: Pitch-Gimbal (+84 / −84 / 0 LSB)
/// - **Phase 3**: Yaw-Gimbal (+84 / −84 / 0 LSB)
/// - **Phase 4**: Beide Gimbals auf 0 trimmen
/// - **Phase 0**: done → `None`
///
/// # Parameter
/// - `state`:  P41-Laufzustand.
/// - `sps`:    SPS-Hardware-Abstraktion.
/// - `pt`:     Phase-Table.
/// - `alarm`:  Alarm-Zustand (derzeit ungenutzt, Signatur analog P40).
///
/// # Rückgabe
/// `Some(())` bei jedem erfolgreichen Schritt, `None` wenn Phase 0 (done).
///
/// # AGC-Quelle
/// `Comanche055/P40-P47.agc`, S40.6 GIMDTEST-Routine
/// (+2ACTDEG = 84 LSB ≈ +2° bei 85.41 Bogensekunden/LSB).
pub fn p41_step<S: SpsHal>(
    state: &mut P41State,
    sps: &mut S,
    pt: &mut PhaseTable,
    _alarm: &mut AlarmState,
) -> Option<()> {
    /// Sequenz-Ausgabewerte für einen Gimbal-Test [LSB]: +2°, −2°, 0°.
    const SEQ: [i16; 3] = [GIMBAL_TEST_STEP, -GIMBAL_TEST_STEP, 0];

    match state.phase {
        2 => {
            // Pitch-Test
            let cmd = SEQ[state.step as usize];
            sps.set_tvc_pitch(cmd);
            state.step += 1;
            if state.step >= 3 {
                state.step = 0;
                let _ = pt.set_phase(4, 3);
                state.phase = 3;
            }
            Some(())
        }
        3 => {
            // Yaw-Test
            let cmd = SEQ[state.step as usize];
            sps.set_tvc_yaw(cmd);
            state.step += 1;
            if state.step >= 3 {
                state.step = 0;
                let _ = pt.set_phase(4, 4);
                state.phase = 4;
            }
            Some(())
        }
        4 => {
            // Trim: beide Gimbals auf 0
            sps.set_tvc_pitch(0);
            sps.set_tvc_yaw(0);
            let _ = pt.set_phase(4, 0);
            state.phase = 0;
            Some(())
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// P47State — Trim Maneuver
// ---------------------------------------------------------------------------

/// Laufzustand von P47 — Trim-Manöver.
///
/// Berechnet den Δv-Restfehler zwischen dem Soll-Δv (aus dem Burn-Plan)
/// und dem tatsächlich gemessenen Δv (aus der IMU-Integration).
///
/// # Einheiten
/// | Feld            | Einheit | Bedeutung                                       |
/// |-----------------|---------|-------------------------------------------------|
/// | `delta_v_imu`   | m/s     | gemessenes Δv aus IMU-Integration nach dem Burn |
/// | `delta_v_ref`   | m/s     | Soll-Δv aus dem Burn-Plan (DELVREF)             |
///
/// # AGC-Quelle
/// `Comanche055/P40-P47.agc`, P47CSM / CALCN83-Routine.
#[derive(Copy, Clone, Debug)]
pub struct P47State {
    /// Aktuelle Phase (2 = Fehler berechnen, 0 = done).
    pub phase: u8,
    /// Gemessenes Δv aus IMU-Integration [m/s].
    pub delta_v_imu: f64,
    /// Soll-Δv aus dem Burn-Plan [m/s] (entspricht DELVREF im AGC).
    pub delta_v_ref: f64,
}

impl P47State {
    /// Erstellt einen neuen P47-Zustand.
    ///
    /// - `delta_v_imu`: gemessenes Δv aus IMU [m/s].
    /// - `delta_v_ref`: Soll-Δv [m/s].
    #[inline]
    pub const fn new(delta_v_imu: f64, delta_v_ref: f64) -> Self {
        Self {
            phase: 2,
            delta_v_imu,
            delta_v_ref,
        }
    }
}

// ---------------------------------------------------------------------------
// p47_step
// ---------------------------------------------------------------------------

/// Führt einen P47-Schritt aus (Trim-Manöver).
///
/// Berechnet den Δv-Restfehler:
/// ```text
/// dv_trim = delta_v_ref − delta_v_imu
/// ```
/// Ein positives Ergebnis bedeutet, dass mehr Δv benötigt wird.
/// Ein negatives Ergebnis bedeutet, dass zu viel Δv aufgebaut wurde.
///
/// # Parameter
/// - `state`:  P47-Laufzustand.
/// - `pt`:     Phase-Table.
/// - `alarm`:  Alarm-Zustand (reserviert für zukünftige Plausibilitätsprüfung).
///
/// # Rückgabe
/// `Some(dv_trim)` [m/s] bei Erfolg, `None` wenn Phase 0 (done).
///
/// # AGC-Quelle
/// `Comanche055/P40-P47.agc`, P47CSM / CALCN83-Routine
/// (DELVCTL + DELVREF → DV47TEMP → Trim-Δv).
pub fn p47_step(state: &mut P47State, pt: &mut PhaseTable, _alarm: &mut AlarmState) -> Option<f64> {
    match state.phase {
        2 => {
            let dv_trim = state.delta_v_ref - state.delta_v_imu;
            let _ = pt.set_phase(4, 0);
            state.phase = 0;
            Some(dv_trim)
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{
        p40_step, p41_step, p47_step, P40State, P41State, P47State, SPS_ISP, SPS_THRUST_N,
    };
    use crate::alarm::AlarmState;
    use crate::hal::sps::{mock::MockSps, SpsHal};
    use crate::restart::phase_table::PhaseTable;

    // -----------------------------------------------------------------------
    // P40-Tests
    // -----------------------------------------------------------------------

    /// Normaldurchlauf P40: Phase 2 → 3 → 4 → 0.
    ///
    /// Δv = 100 m/s, Masse = 28 840 kg. Nach Phase 2 brennt das Triebwerk.
    /// Nach Phase 3 (t_elapsed ≥ t_burn) wird Cutoff ausgelöst.
    /// Nach Phase 4 ist das Triebwerk aus.
    #[test]
    fn p40_burn_nominal() {
        let mut state = P40State::new(100.0, SPS_ISP, SPS_THRUST_N, 28_840.0);
        let mut sps = MockSps::new();
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        // Phase 2: Ignition
        let r = p40_step(&mut state, &mut sps, &mut pt, &mut alarm, 0.0);
        assert!(r.is_some(), "Phase 2 muss Some liefern");
        assert_eq!(state.phase, 3, "Nach Phase 2: phase = 3");
        assert!(sps.engine_on(), "Triebwerk muss nach Ignition brennen");
        assert!(state.t_burn > 0.0, "t_burn muss berechnet sein");
        assert_eq!(pt.get_phase(4), Some(3), "Phase-Table Gruppe 4 = 3");

        // Phase 3: Burn (Zeit akkumulieren bis Cutoff)
        let t_burn = state.t_burn;
        // Ein Schritt mit dt < t_burn → noch kein Cutoff
        let r = p40_step(&mut state, &mut sps, &mut pt, &mut alarm, 1.0);
        assert!(r.is_some(), "Phase 3 muss Some liefern");
        assert_eq!(state.phase, 3, "Nach erstem dt: noch Phase 3");

        // Schritt mit dt groß genug für Cutoff
        let r = p40_step(&mut state, &mut sps, &mut pt, &mut alarm, t_burn);
        assert!(r.is_some(), "Phase 3 mit Cutoff muss Some liefern");
        assert_eq!(state.phase, 4, "Nach Cutoff: phase = 4");

        // Phase 4: Triebwerk abschalten
        let r = p40_step(&mut state, &mut sps, &mut pt, &mut alarm, 0.0);
        assert!(r.is_some(), "Phase 4 muss Some liefern");
        assert!(!sps.engine_on(), "Triebwerk muss nach Cutoff aus sein");
        assert_eq!(state.phase, 0, "Nach Phase 4: phase = 0");
        assert_eq!(pt.get_phase(4), Some(0), "Phase-Table Gruppe 4 = 0");

        // Phase 0: done
        let r = p40_step(&mut state, &mut sps, &mut pt, &mut alarm, 0.0);
        assert!(r.is_none(), "Phase 0 muss None liefern");
        assert!(alarm.last().is_none(), "Kein Alarm erwartet");
    }

    /// Cutoff-Timing: Der Cutoff tritt genau dann ein, wenn t_elapsed ≥ t_burn.
    #[test]
    fn p40_burn_cutoff_timing() {
        let mut state = P40State::new(100.0, SPS_ISP, SPS_THRUST_N, 28_840.0);
        let mut sps = MockSps::new();
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        // Phase 2 ausführen
        p40_step(&mut state, &mut sps, &mut pt, &mut alarm, 0.0);
        let t_burn = state.t_burn;

        // Phase 3 mit genau t_burn → Cutoff
        p40_step(&mut state, &mut sps, &mut pt, &mut alarm, t_burn);
        assert_eq!(
            state.phase, 4,
            "Cutoff muss bei t_elapsed == t_burn auslösen"
        );
    }

    /// Ungültige Parameter (delta_v = 0) → Alarm 0x0440, None.
    #[test]
    fn p40_invalid_params() {
        let mut state = P40State::new(0.0, SPS_ISP, SPS_THRUST_N, 28_840.0);
        let mut sps = MockSps::new();
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        let r = p40_step(&mut state, &mut sps, &mut pt, &mut alarm, 0.0);
        assert!(r.is_none(), "delta_v=0 muss None liefern");
        assert!(alarm.last().is_some(), "Alarm muss gesetzt sein");
    }

    /// Restart in Phase 3: t_elapsed bereits gesetzt → Burn läuft weiter.
    #[test]
    fn p40_restart_phase3() {
        let mut state = P40State::new(100.0, SPS_ISP, SPS_THRUST_N, 28_840.0);
        let mut sps = MockSps::new();
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        // Phase 2 ausführen (Ignition)
        p40_step(&mut state, &mut sps, &mut pt, &mut alarm, 0.0);
        let t_burn = state.t_burn;

        // Simulierter Restart: Phase-Table auf 3, t_elapsed bereits 5 s
        state.t_elapsed = 5.0;
        pt.set_phase(4, 3).unwrap();

        // Burn läuft weiter mit kleinem dt
        let r = p40_step(&mut state, &mut sps, &mut pt, &mut alarm, 1.0);
        assert!(r.is_some(), "Phase 3 nach Restart muss Some liefern");
        // t_elapsed = 6.0, noch kein Cutoff (t_burn > 6.0 bei nominalen Params)
        if t_burn > 6.0 {
            assert_eq!(state.phase, 3, "Burn noch aktiv");
        }
    }

    // -----------------------------------------------------------------------
    // P41-Tests
    // -----------------------------------------------------------------------

    /// Volle Gimbal-Sequenz: Pitch (3 Schritte) → Yaw (3 Schritte) → Trim.
    #[test]
    fn p41_gimbal_sequence() {
        let mut state = P41State::new();
        let mut sps = MockSps::new();
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        // Pitch-Schritt 1: +84
        p41_step(&mut state, &mut sps, &mut pt, &mut alarm);
        assert_eq!(sps.tvc_pitch, 84, "Pitch Schritt 1: +84");
        assert_eq!(state.phase, 2, "Noch Phase 2");

        // Pitch-Schritt 2: -84
        p41_step(&mut state, &mut sps, &mut pt, &mut alarm);
        assert_eq!(sps.tvc_pitch, -84, "Pitch Schritt 2: -84");

        // Pitch-Schritt 3: 0 → Wechsel auf Phase 3
        p41_step(&mut state, &mut sps, &mut pt, &mut alarm);
        assert_eq!(sps.tvc_pitch, 0, "Pitch Schritt 3: 0");
        assert_eq!(state.phase, 3, "Nach Pitch-Sequenz: Phase 3 (Yaw)");

        // Yaw-Schritt 1: +84
        p41_step(&mut state, &mut sps, &mut pt, &mut alarm);
        assert_eq!(sps.tvc_yaw, 84, "Yaw Schritt 1: +84");

        // Yaw-Schritt 2: -84
        p41_step(&mut state, &mut sps, &mut pt, &mut alarm);
        assert_eq!(sps.tvc_yaw, -84, "Yaw Schritt 2: -84");

        // Yaw-Schritt 3: 0 → Wechsel auf Phase 4
        p41_step(&mut state, &mut sps, &mut pt, &mut alarm);
        assert_eq!(sps.tvc_yaw, 0, "Yaw Schritt 3: 0");
        assert_eq!(state.phase, 4, "Nach Yaw-Sequenz: Phase 4 (Trim)");

        // Trim: beide Gimbals auf 0
        p41_step(&mut state, &mut sps, &mut pt, &mut alarm);
        assert_eq!(sps.tvc_pitch, 0, "Trim: Pitch = 0");
        assert_eq!(sps.tvc_yaw, 0, "Trim: Yaw = 0");
        assert_eq!(state.phase, 0, "Nach Trim: Phase 0 (done)");
        assert_eq!(pt.get_phase(4), Some(0), "Phase-Table Gruppe 4 = 0");

        // done: None
        let r = p41_step(&mut state, &mut sps, &mut pt, &mut alarm);
        assert!(r.is_none(), "Phase 0 muss None liefern");
    }

    // -----------------------------------------------------------------------
    // P47-Tests
    // -----------------------------------------------------------------------

    /// Trim-Restfehler: delta_v_ref − delta_v_imu = 5.0 m/s.
    #[test]
    fn p47_trim_residual() {
        let mut state = P47State::new(95.0, 100.0);
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        let r = p47_step(&mut state, &mut pt, &mut alarm);
        assert!(r.is_some(), "Phase 2 muss Some liefern");
        let dv = r.unwrap();
        assert!(
            (dv - 5.0).abs() < 1e-12,
            "Trim-Restfehler soll 5.0 m/s sein, bekam {dv}"
        );
        assert_eq!(state.phase, 0, "Nach Trim: Phase 0");
        assert_eq!(pt.get_phase(4), Some(0), "Phase-Table Gruppe 4 = 0");

        // done: None
        let r2 = p47_step(&mut state, &mut pt, &mut alarm);
        assert!(r2.is_none(), "Phase 0 muss None liefern");
    }

    /// Kein Restfehler: delta_v_imu == delta_v_ref → dv_trim = 0.0.
    #[test]
    fn p47_no_trim_needed() {
        let mut state = P47State::new(100.0, 100.0);
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        let r = p47_step(&mut state, &mut pt, &mut alarm);
        let dv = r.unwrap();
        assert!(dv.abs() < 1e-12, "Kein Restfehler erwartet, bekam {dv}");
    }
}
