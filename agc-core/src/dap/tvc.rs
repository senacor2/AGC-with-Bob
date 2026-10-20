/// TVC-DAP — Pitch- und Yaw-Digitaler-Autopilot für SPS-Verbrennung.
///
/// Realisiert zwei unabhängige, identisch aufgebaute Regelkreise (Pitch und Yaw),
/// die während SPS-Triebwerkszündungen die Gimbal-Aktuatoren ansteuern.
/// Jeder Kreis enthält:
///   - Lagewinkel-Differenzierung (CDU-Ableitung → Körperachsen-Raten)
///   - Lagefehler-Integration mit Begrenzung
///   - 6th-order vorwärts Filter (vereinfacht als PI-Regler modelliert)
///   - Ausgabebegrenzung auf ±8191 LSB (14-Bit DAC)
///
/// # Einheiten
/// - `attitude_error_rad`: Lagewinkelfehler in Radiant (Pitch bzw. Yaw).
/// - `rate_cmd_rad_s`:     Ratenbefehl aus Cross-Product Steering (S40.8), rad/s.
/// - Ausgabe: `i16` AGC-LSB, 1 LSB ≈ 0.023° → Vollskala ±2,7°.
///
/// # AGC source
/// `Comanche055/TVCDAPS.agc`, PITCHDAP / YAWDAP / FWDFLTR / PRECOMP Routinen.
use crate::hal::SpsHal;

// ---------------------------------------------------------------------------
// Konstanten
// ---------------------------------------------------------------------------

/// DAC-Ausgabebegrenzung: 14-Bit vorzeichenbehaftet (±8191 LSB).
const ACT_LIMIT: i16 = 8191;

/// Integratorlimit für den Lagefehler: ≈ 5° in Radiant.
const ERROR_LIMIT_RAD: f64 = 0.087_266_46; // 5° in rad

/// Proportionalverstärkung des TVC-Reglers (dimensionslos).
/// Entspricht dem AGC-Parameter VARK (nominell ~0.4 für CSM-only, kein LM).
const KP: f64 = 0.4;

/// Integratorverstärkung.
const KI: f64 = 0.05;

/// Konvertierungsfaktor rad → AGC-LSB.
/// 1 LSB = 0.023° = 0.023 * π/180 rad ⇒ 1 rad = 1/(0.023*π/180) ≈ 2491.6 LSB.
const RAD_TO_LSB: f64 = 2491.6;

// ---------------------------------------------------------------------------
// Zustandsspeicher für einen einzelnen TVC-Kanal (Pitch oder Yaw)
// ---------------------------------------------------------------------------

/// Zustandsvariablen für einen TVC-Regelkanal.
///
/// AGC source: `Comanche055/TVCDAPS.agc`, erasable storage PCDU, YCDU etc.
#[derive(Clone, Copy, Debug)]
pub struct TvcChannelState {
    /// Integrierter Lagefehler (rad).
    pub error_integral: f64,
    /// Letzter ausgegebener Aktuatorbefehl (LSB), für Offset-Tracker benötigt.
    pub last_command: i16,
}

impl TvcChannelState {
    pub const fn new() -> Self {
        Self {
            error_integral: 0.0,
            last_command: 0,
        }
    }
}

impl Default for TvcChannelState {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Öffentliche API
// ---------------------------------------------------------------------------

/// Führt einen TVC-DAP-Schritt für **Pitch** durch und gibt den Gimbal-Befehl aus.
///
/// Aufzurufen mit dem Lagefehler um die Pitch-Achse sowie dem Ratenbefehl
/// aus der Cross-Product-Steuerung.  Das Ergebnis wird direkt an [`SpsHal`]
/// übergeben.
///
/// # Parameter
/// - `state`:            veränderlicher Kanalzustand (Integrator etc.).
/// - `attitude_error`:   aktueller Lagewinkel-Fehler Pitch, Radiant.
/// - `rate_cmd`:         Pitch-Ratenbefehl aus S40.8, rad/s.
/// - `dt`:               Zeitschritt (= T5TVCDT), Sekunden.
/// - `sps`:              SPS HAL-Implementierung für die Ausgabe.
///
/// AGC source: `Comanche055/TVCDAPS.agc`, PITCHDAP Routine.
pub fn tvc_pitch_step<S: SpsHal>(
    state: &mut TvcChannelState,
    attitude_error: f64,
    rate_cmd: f64,
    dt: f64,
    sps: &mut S,
) {
    let cmd = tvc_channel_step(state, attitude_error, rate_cmd, dt);
    sps.set_tvc_pitch(cmd);
}

/// Führt einen TVC-DAP-Schritt für **Yaw** durch und gibt den Gimbal-Befehl aus.
///
/// Analog zu [`tvc_pitch_step`] für die Yaw-Achse.
///
/// AGC source: `Comanche055/TVCDAPS.agc`, YAWDAP Routine.
pub fn tvc_yaw_step<S: SpsHal>(
    state: &mut TvcChannelState,
    attitude_error: f64,
    rate_cmd: f64,
    dt: f64,
    sps: &mut S,
) {
    let cmd = tvc_channel_step(state, attitude_error, rate_cmd, dt);
    sps.set_tvc_yaw(cmd);
}

// ---------------------------------------------------------------------------
// Kernberechnung (privat)
// ---------------------------------------------------------------------------

/// Berechnet den Aktuatorbefehl für einen TVC-Kanal (Pitch oder Yaw).
///
/// Entspricht dem AGC-Vorwärtsfilter FWDFLTR mit PI-Regler-Näherung:
///
/// 1. Lagefehler integrieren (mit Begrenzung auf ±ERROR_LIMIT_RAD).
/// 2. Ausgang = KP * (error + rate_cmd * dt / KP) + KI * integral.
/// 3. Skalierung rad → LSB und Clamp auf ±8191.
///
/// AGC source: `Comanche055/TVCDAPS.agc`, FWDFLTR / PRECOMP / ERRORLIM / ACTLIM.
fn tvc_channel_step(
    state: &mut TvcChannelState,
    attitude_error: f64,
    rate_cmd: f64,
    dt: f64,
) -> i16 {
    // Integratorzuwachs = Lagefehler × Zeitschritt
    state.error_integral += attitude_error * dt;

    // Integratorbegrenzung (ERRORLIM)
    state.error_integral = state
        .error_integral
        .clamp(-ERROR_LIMIT_RAD, ERROR_LIMIT_RAD);

    // Reglerausgang
    let output_rad = KP * attitude_error + KI * state.error_integral - rate_cmd * dt;

    // Umrechnung in AGC-LSB und Begrenzung (ACTLIM)
    let output_lsb = libm::round(output_rad * RAD_TO_LSB) as i32;
    let clamped = output_lsb.clamp(-(ACT_LIMIT as i32), ACT_LIMIT as i32) as i16;

    state.last_command = clamped;
    clamped
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hal::sps::mock::MockSps;

    #[test]
    fn zero_error_zero_output() {
        let mut state = TvcChannelState::new();
        let mut sps = MockSps::new();
        tvc_pitch_step(&mut state, 0.0, 0.0, 0.04, &mut sps);
        assert_eq!(sps.tvc_pitch, 0);
    }

    #[test]
    fn positive_pitch_error_gives_positive_command() {
        let mut state = TvcChannelState::new();
        let mut sps = MockSps::new();
        // Kleiner positiver Lagefehler → positiver Gimbal-Befehl
        tvc_pitch_step(&mut state, 0.01, 0.0, 0.04, &mut sps);
        assert!(sps.tvc_pitch > 0, "Pitch-Befehl muss positiv sein");
    }

    #[test]
    fn output_saturates_at_limit() {
        let mut state = TvcChannelState::new();
        let mut sps = MockSps::new();
        // 10 rad Fehler → KP*10*RAD_TO_LSB = 0.4*10*2491.6 ≈ 9966 > ACT_LIMIT
        tvc_yaw_step(&mut state, 10.0, 0.0, 0.04, &mut sps);
        assert_eq!(sps.tvc_yaw, ACT_LIMIT);
    }

    #[test]
    fn negative_error_gives_negative_command() {
        let mut state = TvcChannelState::new();
        let mut sps = MockSps::new();
        tvc_yaw_step(&mut state, -0.05, 0.0, 0.04, &mut sps);
        assert!(sps.tvc_yaw < 0, "Yaw-Befehl muss negativ sein");
    }

    #[test]
    fn integrator_accumulates() {
        let mut state = TvcChannelState::new();
        let mut sps = MockSps::new();
        // Gleicher Fehler für mehrere Schritte → Integral wächst
        for _ in 0..5 {
            tvc_pitch_step(&mut state, 0.001, 0.0, 0.04, &mut sps);
        }
        // Nach 5 Schritten muss der Befehl größer sein als nach einem Schritt
        let cmd_after_5 = sps.tvc_pitch;
        let mut state2 = TvcChannelState::new();
        let mut sps2 = MockSps::new();
        tvc_pitch_step(&mut state2, 0.001, 0.0, 0.04, &mut sps2);
        assert!(
            cmd_after_5 >= sps2.tvc_pitch,
            "Befehl nach 5 Schritten muss ≥ Befehl nach 1 Schritt sein"
        );
    }

    #[test]
    fn integrator_clamped() {
        let mut state = TvcChannelState::new();
        // Nach vielen Schritten mit konstantem Fehler darf ERROR_LIMIT nicht überschritten werden
        for _ in 0..1000 {
            tvc_channel_step(&mut state, 0.1, 0.0, 0.04);
        }
        assert!(
            state.error_integral.abs() <= ERROR_LIMIT_RAD + 1e-12,
            "Integrator überschreitet ERROR_LIMIT_RAD"
        );
    }
}
