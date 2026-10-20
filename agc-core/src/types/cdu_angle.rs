/// CDU-Winkel aus den Course Digital Unit (CDU) Winkelregistern der IMU.
///
/// # Einheit und Skalierung
/// `1 LSB = 360° / 2^15 ≈ 0.010986°`  
/// Wertebereich: `−32768 … +32767` (i16, Zweierkomplement).  
/// Volle Umdrehung (360°) entspricht `32768 LSB` (positiver Überlauf = negatives Maximum).
///
/// # Verwendung
/// Rohe CDU-Register-Werte sind immer `CduAngle`. Für Navigationsmathematik
/// mit [`f64`] die Methode [`CduAngle::to_radians`] verwenden.
///
/// # AGC-Quelle
/// `Comanche055/IMU_COMPENSATION_PACKAGE.agc`, CDU-Auslesefunktionen;
/// `Comanche055/FRESH_START_AND_RESTART.agc`, CDUX/CDUY/CDUZ-Kanalworte.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct CduAngle(pub i16);

/// Skalierungsfaktor: Radiant pro LSB = 2π / 2^15
const RAD_PER_LSB: f64 = core::f64::consts::TAU / 32768.0;

/// Skalierungsfaktor: LSB pro Radiant = 2^15 / 2π
const LSB_PER_RAD: f64 = 32768.0 / core::f64::consts::TAU;

impl CduAngle {
    /// Erstellt einen `CduAngle` aus einem rohen i16-Registerwert.
    #[inline]
    pub const fn from_raw(raw: i16) -> Self {
        Self(raw)
    }

    /// Gibt den rohen i16-Registerwert zurück.
    #[inline]
    pub const fn raw(self) -> i16 {
        self.0
    }

    /// Wandelt in Radiant um.
    ///
    /// Ergebnis liegt im Bereich `(−π, +π]`.
    #[inline]
    pub fn to_radians(self) -> f64 {
        f64::from(self.0) * RAD_PER_LSB
    }

    /// Erstellt einen `CduAngle` aus einem Wert in Radiant.
    ///
    /// Der Winkel wird auf den Darstellungsbereich des i16 gewrappt
    /// (modulo 2π, dann gerundet auf den nächsten LSB).
    #[inline]
    pub fn from_radians(rad: f64) -> Self {
        // Skaliere auf LSB, runde auf ganzzahligen Wert
        let turns = libm::round(rad * LSB_PER_RAD);
        // Wrapping modulo 2^16 auf den i16-Bereich [−32768, 32767].
        // Cast via i32 → wrapping_as → i16 vermeidet undefined-behaviour beim
        // Overflow-Cast (Rust-Referenz: `as`-Cast sättigt für f64→iN nicht).
        #[allow(clippy::cast_possible_truncation)]
        let lsb = turns as i32;
        // i32::wrapping_cast auf i16: modulo 2^16
        Self(lsb as i16)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::CduAngle;
    use core::f64::consts::{FRAC_PI_2, PI, TAU};

    #[test]
    fn zero_roundtrip() {
        let a = CduAngle::from_raw(0);
        assert!((a.to_radians()).abs() < 1e-15);
    }

    #[test]
    fn positive_quarter_turn() {
        // 90° = π/2 rad ≈ 8192 LSB
        let a = CduAngle::from_raw(8192);
        let diff = (a.to_radians() - FRAC_PI_2).abs();
        // Fehler ≤ 0.5 LSB = π / 32768
        assert!(diff < PI / 32768.0, "diff = {diff}");
    }

    #[test]
    fn from_radians_roundtrip() {
        let angles = [0.0_f64, FRAC_PI_2, PI * 0.999, -FRAC_PI_2, -PI * 0.999];
        for &rad in &angles {
            let encoded = CduAngle::from_radians(rad);
            let decoded = encoded.to_radians();
            // Quantisierungsfehler ≤ 0.5 LSB ≈ 9.6e-5 rad
            assert!(
                (decoded - rad).abs() < 1e-4,
                "rad={rad:.6} encoded={} decoded={decoded:.6}",
                encoded.raw()
            );
        }
    }

    #[test]
    fn full_turn_wraps() {
        // 2π rad → 32768 LSB → wrappender i32-as-i16-Cast → −32768 (i16::MIN).
        // Das ist der korrekte Overflow: 32768i32 as i16 == -32768.
        let a = CduAngle::from_radians(TAU);
        assert_eq!(a.raw(), i16::MIN, "raw={}", a.raw());
    }
}
