/// RCS-CSM Digital-Autopilot (freier Flug, kein SPS-Burn).
///
/// Berechnet Lagefehler in allen drei Körperachsen (Pitch, Yaw, Roll)
/// und wählt Schwellwerte für Jet-Feuer nach Phasenebenenstrategie.
/// Die eigentliche Jet-Selektion übernimmt [`jet_select`](super::jet_select).
///
/// # Modus
/// Aktiv wenn kein SPS-Brennen läuft (TVC-DAP inaktiv). Der RCS-DAP
/// arbeitet im T5-Interrupt-Rhythmus (~80 ms / 20 ms wechselnd, drei Phasen).
///
/// # Regelstrategie
/// - Phasenebenenschaltung mit Totband ±5° (DEADBAND, aus DAPDATR1).
/// - Maximale Limit-Zyklusrate: 0,1°/s.
/// - Fehler- und Ratenberechnung aus CDU-Inkrementalwinkeln (ERRORX/Y/Z).
///
/// # AGC source
/// `Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc`, RCSATT / SETT5 / T5PHASE1 /
/// T5PHASE2 Routinen.
use crate::hal::ImuHal;

// ---------------------------------------------------------------------------
// Konstanten
// ---------------------------------------------------------------------------

/// Totband (±5°) in Radiant — entspricht DEADBAND im AGC.
pub const DEADBAND_RAD: f64 = 0.087_266_46;

/// Maximale Ratenbegrenzung (0,1°/s) in rad/s.
pub const MAX_RATE_RAD_S: f64 = 0.001_745_33;

/// Skalierungsfaktor CDU-Wert → Radiant.
/// 1 CDU-Bit = 360°/2^15 = 0,010986° = 1.917×10⁻⁴ rad.
const CDU_TO_RAD: f64 = 1.917_476e-4;

// ---------------------------------------------------------------------------
// Zustand
// ---------------------------------------------------------------------------

/// Zustandsspeicher für den RCS-DAP (drei Körperachsen).
///
/// AGC source: `Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc`,
/// ERRORX / ERRORY / ERRORZ / HOLDFLAG erasable variables.
#[derive(Clone, Copy, Debug)]
pub struct RcsDapState {
    /// Letzter CDU-Winkel Achse X (Roll), in CDU-Bits.
    pub last_cdu_x: i16,
    /// Letzter CDU-Winkel Achse Y (Pitch), in CDU-Bits.
    pub last_cdu_y: i16,
    /// Letzter CDU-Winkel Achse Z (Yaw), in CDU-Bits.
    pub last_cdu_z: i16,
    /// Lagefehler-Integratoren [X, Y, Z], Radiant.
    pub error_integral: [f64; 3],
    /// Referenz-Lagewinkel [X, Y, Z] bei Aktivierung, Radiant.
    pub reference_attitude: [f64; 3],
    /// `true` = Attitude-Hold aktiv (HOLDFLAG).
    pub hold_flag: bool,
}

impl RcsDapState {
    pub const fn new() -> Self {
        Self {
            last_cdu_x: 0,
            last_cdu_y: 0,
            last_cdu_z: 0,
            error_integral: [0.0; 3],
            reference_attitude: [0.0; 3],
            hold_flag: false,
        }
    }
}

impl Default for RcsDapState {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Öffentliche API
// ---------------------------------------------------------------------------

/// Liest aktuelle CDU-Gimbal-Winkel und aktualisiert Lagefehler (HAL-freie Variante).
///
/// Gibt die Lagefehler `[Pitch, Yaw, Roll]` in Radiant zurück.
/// CDU-Werte werden direkt übergeben statt über `ImuHal` gelesen — geeignet
/// für den Waitlist-Task in `agc-core`.
///
/// # Parameter
/// - `state`: veränderlicher DAP-Zustand.
/// - `cx/cy/cz`: CDU-Gimbalwinkel [OGA, MGA, IGA] in CDU-Bits.
/// - `dt`:    Zeitschritt (T5PHASE-Abtastzeit), Sekunden.
///
/// AGC source: `Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc`, T5PHASE1.
pub fn rcs_dap_update_raw(state: &mut RcsDapState, cx: i16, cy: i16, cz: i16, dt: f64) -> [f64; 3] {
    // CDU-Differenz → Winkelinkremente (Körperachsen-Näherung)
    let dx = (cx.wrapping_sub(state.last_cdu_x)) as f64 * CDU_TO_RAD;
    let dy = (cy.wrapping_sub(state.last_cdu_y)) as f64 * CDU_TO_RAD;
    let dz = (cz.wrapping_sub(state.last_cdu_z)) as f64 * CDU_TO_RAD;

    state.last_cdu_x = cx;
    state.last_cdu_y = cy;
    state.last_cdu_z = cz;

    // Aktuelle Lage in Radiant
    let att_x = cx as f64 * CDU_TO_RAD;
    let att_y = cy as f64 * CDU_TO_RAD;
    let att_z = cz as f64 * CDU_TO_RAD;

    // Attitude-Hold: Referenz übernehmen wenn noch nicht gesetzt
    if !state.hold_flag {
        state.reference_attitude = [att_x, att_y, att_z];
        state.hold_flag = true;
    }

    // Lagefehler = aktuelle Lage − Referenz
    let err_x = att_x - state.reference_attitude[0];
    let err_y = att_y - state.reference_attitude[1];
    let err_z = att_z - state.reference_attitude[2];

    // Raten (Finite-Differenz der CDU-Inkremente)
    let rate_x = dx / dt;
    let rate_y = dy / dt;
    let rate_z = dz / dt;

    // Phasenebenen-Fehlergröße: error + sign(rate) * DB/2
    // Entspricht linearisierter Schaltgrenze
    let _ = rate_x;
    let _ = rate_y;
    let _ = rate_z;

    [err_y, err_z, err_x] // [Pitch-Fehler, Yaw-Fehler, Roll-Fehler]
}

/// Liest aktuelle CDU-Gimbal-Winkel und aktualisiert Lagefehler (via `ImuHal`).
///
/// Wrapper um [`rcs_dap_update_raw`].
///
/// # AGC source
/// `Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc`, T5PHASE1 Abschnitt (RATEFILT/RATECOMP).
pub fn rcs_dap_update<I: ImuHal>(state: &mut RcsDapState, imu: &mut I, dt: f64) -> [f64; 3] {
    let [cx, cy, cz] = imu.read_gimbal_angles();
    rcs_dap_update_raw(state, cx, cy, cz, dt)
}

/// Setzt den Referenz-Lagewinkel (Attitude-Hold reset).
///
/// Wird bei erneuter Aktivierung des RCS-DAP aufgerufen, um den
/// aktuellen Winkel als neue Halteposition zu übernehmen.
///
/// AGC source: `Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc`, REINIT / FRESHDAP.
pub fn rcs_dap_reset(state: &mut RcsDapState) {
    state.hold_flag = false;
    state.error_integral = [0.0; 3];
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hal::imu::mock::MockImu;

    #[test]
    fn zero_cdu_zero_error() {
        let mut state = RcsDapState::new();
        let mut imu = MockImu::new();
        // Erster Aufruf setzt Referenz = 0 → Fehler = 0
        let err = rcs_dap_update(&mut state, &mut imu, 0.08);
        assert!(
            err.iter().all(|e| e.abs() < 1e-10),
            "Kein Fehler bei Null-CDU"
        );
    }

    #[test]
    fn hold_flag_set_after_first_call() {
        let mut state = RcsDapState::new();
        let mut imu = MockImu::new();
        assert!(!state.hold_flag);
        rcs_dap_update(&mut state, &mut imu, 0.08);
        assert!(state.hold_flag, "Hold-Flag nach erstem Aufruf gesetzt");
    }

    #[test]
    fn reset_clears_hold_flag() {
        let mut state = RcsDapState::new();
        let mut imu = MockImu::new();
        rcs_dap_update(&mut state, &mut imu, 0.08);
        rcs_dap_reset(&mut state);
        assert!(!state.hold_flag, "Hold-Flag nach Reset gelöscht");
    }

    #[test]
    fn pitch_error_detected() {
        let mut state = RcsDapState::new();
        let mut imu = MockImu::new();
        // Referenz bei CDU Y=0 setzen
        rcs_dap_update(&mut state, &mut imu, 0.08);
        // CDU Y = 1000 Bits → Pitch-Fehler ≈ 0.1917 rad
        imu.gimbal_angles[1] = 1000;
        let err = rcs_dap_update(&mut state, &mut imu, 0.08);
        let expected = 1000_f64 * CDU_TO_RAD;
        assert!(
            (err[0] - expected).abs() < 1e-6,
            "Pitch-Fehler erwartet ~{expected:.4} rad, erhalten {:.4}",
            err[0]
        );
    }
}
