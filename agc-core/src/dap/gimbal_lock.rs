/// Gimbal-Lock-Erkennung und -Schutz.
///
/// Überwacht den mittleren Gimbal-Winkel (MGA / CDUY) und warnt rechtzeitig
/// vor Gimbal-Lock-Situationen. Im AGC wird dies im Kontext von KALCMANU und
/// dem RCS-DAP geprüft: Übersteigt der MGA ±70° (LOCSKIRT-Grenze), muss der
/// Autopilot gegensteuern.
///
/// # Gimbal-Lock bei der Apollo-IMU
/// Die drei-Achsen-kardanische Lagerung verliert bei MGA ≈ ±90° einen
/// Freiheitsgrad (Gimbal-Lock). Die AGC warnt ab ±70°, damit der Autopilot
/// noch rechtzeitig ausweichen kann.
///
/// | Grenzwert       | CDU-Bits | Grad |
/// |-----------------|----------|------|
/// | Warnung (WCALC) | ±6371    | ±70° |
/// | Kritisch        | ±8192    | ±90° |
///
/// # AGC source
/// `Comanche055/GIMBAL_LOCK_AVOIDANCE.agc`, LOCSKIRT / WCALC Routinen.
use crate::alarm::{self, AlarmState};

// ---------------------------------------------------------------------------
// Konstanten
// ---------------------------------------------------------------------------

/// Warngrenze Gimbal-Lock: 70° in CDU-Bits (15-Bit-Vollkreis = 32768 Bits).
/// 70° = 70/360 * 32768 ≈ 6371 Bits.
///
/// AGC source: `Comanche055/GIMBAL_LOCK_AVOIDANCE.agc`, LOCSKIRT Abschnitt.
pub const GIMBAL_LOCK_WARNING_BITS: i16 = 6371;

/// Kritische Grenze: 85° in CDU-Bits (konservativ vor 90°).
pub const GIMBAL_LOCK_CRITICAL_BITS: i16 = 7736;

/// AGC Alarm-Code für Gimbal-Lock-Warnung.
/// Entspricht dem Program-Alarm in der AGC-Alarm-Infrastruktur.
pub const ALARM_GIMBAL_LOCK: u16 = 0x0214;

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

/// Gimbal-Lock-Zustand.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GimbalLockStatus {
    /// MGA im sicheren Bereich (|MGA| ≤ 70°).
    Safe,
    /// MGA in Warnzone (70° < |MGA| ≤ 85°) — Autopilot sollte ausweichen.
    Warning,
    /// MGA im kritischen Bereich (|MGA| > 85°) — Gimbal-Lock unmittelbar.
    Critical,
}

// ---------------------------------------------------------------------------
// Öffentliche API
// ---------------------------------------------------------------------------

/// Prüft den aktuellen MGA-Winkel auf Gimbal-Lock-Gefahr.
///
/// Bei Warnzone oder kritischem Bereich wird über [`alarm::raise`] ein
/// Program Alarm ausgelöst (kein GOJAM).
///
/// # Parameter
/// - `alarm_state`: Alarm-Zustand für `alarm::raise`.
/// - `mga_cdu`:     Mittlerer Gimbal-Winkel in CDU-Bits (CDUY).
///
/// # Rückgabe
/// [`GimbalLockStatus`] — Safe, Warning oder Critical.
///
/// AGC source: `Comanche055/GIMBAL_LOCK_AVOIDANCE.agc`, WCALC / LOCSKIRT.
pub fn check_gimbal_lock(alarm_state: &mut AlarmState, mga_cdu: i16) -> GimbalLockStatus {
    let abs_mga = mga_cdu.unsigned_abs();

    if abs_mga > GIMBAL_LOCK_CRITICAL_BITS as u16 {
        alarm::raise(alarm_state, ALARM_GIMBAL_LOCK);
        GimbalLockStatus::Critical
    } else if abs_mga > GIMBAL_LOCK_WARNING_BITS as u16 {
        alarm::raise(alarm_state, ALARM_GIMBAL_LOCK);
        GimbalLockStatus::Warning
    } else {
        GimbalLockStatus::Safe
    }
}

/// Gibt `true` zurück wenn der MGA-Winkel zu nahe an Gimbal-Lock ist,
/// um ein Manöver mit der angegebenen Winkelgröße sicher zu starten.
///
/// Wird von KALCMANU aufgerufen bevor ein automatisches Manöver beginnt.
///
/// AGC source: `Comanche055/GIMBAL_LOCK_AVOIDANCE.agc`, SETGO-Logik.
pub fn maneuver_safe(mga_cdu: i16) -> bool {
    mga_cdu.unsigned_abs() <= GIMBAL_LOCK_WARNING_BITS as u16
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::alarm::AlarmState;

    #[test]
    fn safe_at_zero() {
        let mut alarm = AlarmState::new();
        assert_eq!(check_gimbal_lock(&mut alarm, 0), GimbalLockStatus::Safe);
        assert!(alarm.last().is_none(), "Kein Alarm im sicheren Bereich");
    }

    #[test]
    fn safe_below_warning_limit() {
        let mut alarm = AlarmState::new();
        assert_eq!(
            check_gimbal_lock(&mut alarm, GIMBAL_LOCK_WARNING_BITS),
            GimbalLockStatus::Safe
        );
    }

    #[test]
    fn warning_above_70_deg() {
        let mut alarm = AlarmState::new();
        let status = check_gimbal_lock(&mut alarm, GIMBAL_LOCK_WARNING_BITS + 1);
        assert_eq!(status, GimbalLockStatus::Warning);
        assert_eq!(
            alarm.last(),
            Some(ALARM_GIMBAL_LOCK),
            "Alarm 0214 muss ausgelöst werden"
        );
    }

    #[test]
    fn critical_above_85_deg() {
        let mut alarm = AlarmState::new();
        let status = check_gimbal_lock(&mut alarm, GIMBAL_LOCK_CRITICAL_BITS + 1);
        assert_eq!(status, GimbalLockStatus::Critical);
        assert_eq!(alarm.last(), Some(ALARM_GIMBAL_LOCK));
    }

    #[test]
    fn negative_mga_also_detected() {
        let mut alarm = AlarmState::new();
        let status = check_gimbal_lock(&mut alarm, -(GIMBAL_LOCK_WARNING_BITS + 10));
        assert_ne!(
            status,
            GimbalLockStatus::Safe,
            "Negativer Winkel wird erkannt"
        );
    }

    #[test]
    fn maneuver_safe_within_limit() {
        assert!(maneuver_safe(0));
        assert!(maneuver_safe(GIMBAL_LOCK_WARNING_BITS));
        assert!(!maneuver_safe(GIMBAL_LOCK_WARNING_BITS + 1));
    }
}
