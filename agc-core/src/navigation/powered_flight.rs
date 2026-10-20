//! Powered-Flight-Subroutinen für Comanche055.
//!
//! Implementiert Brennzeit-Berechnung (Tsiolkovsky-Gleichung) und CDU-Winkel-Trigonometrie
//! für die SPS- und RCS-Triebwerkssteuerung.
//!
//! # Einheiten und Skalierung
//! - `delta_v` : Geschwindigkeitsänderung [m/s]
//! - `isp`     : spezifischer Impuls [s]
//! - `thrust`  : Schub [N]
//! - `mass`    : Fahrzeugmasse [kg]
//! - `t_burn`  : Brenndauer [s]
//! - `m_final` : Endmasse nach dem Burn [kg]
//!
//! # AGC-Quelle
//! `Comanche055/POWERED_FLIGHT_SUBROUTINES.agc`

use crate::types::CduAngle;

/// Erdbeschleunigung (Standard-Schwerkraft) [m/s²].
///
/// Definition nach ISO 80000-3 / NIST.
const G0: f64 = 9.806_65;

/// Berechnet Brenndauer und Endmasse für einen impulsiven Triebwerksbrand.
///
/// Verwendet die Tsiolkovsky-Raketengleichung:
/// ```text
/// m_final = mass · exp(−Δv / (Isp · g₀))
/// t_burn  = (mass − m_final) · Isp · g₀ / thrust
/// ```
///
/// # Eingabe
/// - `delta_v` : gewünschte Geschwindigkeitsänderung [m/s] — muss > 0
/// - `isp`     : spezifischer Impuls [s] — muss > 0
/// - `thrust`  : Triebwerksschub [N] — muss > 0
/// - `mass`    : Fahrzeugmasse vor dem Burn [kg] — muss > 0
///
/// # Rückgabe
/// `Some((t_burn [s], m_final [kg]))` bei gültigen Eingaben,
/// `None` falls ein Parameter ≤ 0 ist.
///
/// # AGC-Quelle
/// `Comanche055/POWERED_FLIGHT_SUBROUTINES.agc`
pub fn burn_time(delta_v: f64, isp: f64, thrust: f64, mass: f64) -> Option<(f64, f64)> {
    if delta_v <= 0.0 || isp <= 0.0 || thrust <= 0.0 || mass <= 0.0 {
        return None;
    }

    let ve = isp * G0; // effektive Ausströmgeschwindigkeit [m/s]
    let m_final = mass * libm::exp(-delta_v / ve);
    let t_burn = (mass - m_final) * ve / thrust;

    Some((t_burn, m_final))
}

/// Konvertiert einen CDU-Winkel in Sinus- und Kosinus-Werte.
///
/// Liest das CDU-Winkelregister über [`CduAngle::to_radians`] aus und berechnet
/// das Winkelpaar `(sin θ, cos θ)` mit `libm`-Funktionen (kein `std`).
///
/// # Eingabe
/// - `angle`: CDU-Winkelregister als [`CduAngle`] (i16, 1 LSB = 360° / 2¹⁵)
///
/// # Rückgabe
/// `(sin θ, cos θ)` — beide Werte im Bereich `[−1, +1]`.
///
/// # AGC-Quelle
/// `Comanche055/POWERED_FLIGHT_SUBROUTINES.agc`, CDUTRIG-Routine.
pub fn cdu_to_sin_cos(angle: CduAngle) -> (f64, f64) {
    let rad = angle.to_radians();
    (libm::sin(rad), libm::cos(rad))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{burn_time, cdu_to_sin_cos, G0};
    use crate::types::CduAngle;

    /// SPS-Nominalbetrieb:
    /// Isp = 314.5 s, Schub = 97 855 N, Masse = 28 840 kg, Δv = 100 m/s.
    /// Ergebnis wird analytisch gegen die Tsiolkovsky-Gleichung geprüft.
    #[test]
    fn burn_time_sps_nominal() {
        let isp = 314.5_f64;
        let thrust = 97_855.0_f64;
        let mass = 28_840.0_f64;
        let delta_v = 100.0_f64;

        let (t, m_f) =
            burn_time(delta_v, isp, thrust, mass).expect("Nominaleingaben müssen Some liefern");

        // Analytisch:
        let ve = isp * G0;
        let m_final_expected = mass * libm::exp(-delta_v / ve);
        let t_expected = (mass - m_final_expected) * ve / thrust;

        assert!((t - t_expected).abs() < 1e-9, "t_burn: {t} ≠ {t_expected}");
        assert!(
            (m_f - m_final_expected).abs() < 1e-9,
            "m_final: {m_f} ≠ {m_final_expected}"
        );
    }

    /// Ungültige Parameterkombinationen müssen alle None liefern.
    #[test]
    fn burn_time_rejects_invalid() {
        // thrust = 0
        assert!(burn_time(100.0, 314.5, 0.0, 28_840.0).is_none());
        // mass = 0
        assert!(burn_time(100.0, 314.5, 97_855.0, 0.0).is_none());
        // isp = 0
        assert!(burn_time(100.0, 0.0, 97_855.0, 28_840.0).is_none());
        // delta_v = 0
        assert!(burn_time(0.0, 314.5, 97_855.0, 28_840.0).is_none());
    }

    /// CduAngle(0) → Winkel 0 rad → sin ≈ 0, cos ≈ 1.
    #[test]
    fn cdu_to_sin_cos_zero_angle() {
        let (s, c) = cdu_to_sin_cos(CduAngle(0));
        assert!(s.abs() < 1e-15, "sin(0) = {s}");
        assert!((c - 1.0).abs() < 1e-15, "cos(0) = {c}");
    }

    /// CduAngle(0x2000) = 8192 LSB = π/2 → sin ≈ 1, cos ≈ 0.
    ///
    /// Skalierung: 1 LSB = 2π / 32768 rad → 8192 LSB = π/2 (Viertelumdrehung).
    /// 0x4000 = 16384 LSB = π (halbe Umdrehung = 180°), nicht 90°.
    /// Quantisierungsfehler ≤ 0,5 LSB ≈ 9,6e-5 rad.
    #[test]
    fn cdu_to_sin_cos_quarter_rev() {
        let (s, c) = cdu_to_sin_cos(CduAngle(0x2000));
        assert!((s - 1.0).abs() < 1e-4, "sin(π/2) soll ≈ 1 sein, bekam {s}");
        assert!(c.abs() < 1e-4, "cos(π/2) soll ≈ 0 sein, bekam {c}");
    }

    /// Nach dem Burn muss die Endmasse kleiner als die Startmasse und t_burn > 0 sein.
    #[test]
    fn burn_time_fuel_check() {
        let (t, m_f) = burn_time(100.0, 314.5, 97_855.0, 28_840.0)
            .expect("Nominaleingaben müssen Some liefern");

        assert!(m_f < 28_840.0, "Endmasse soll < Startmasse sein: {m_f}");
        assert!(t > 0.0, "Brenndauer soll > 0 sein: {t}");
    }
}
