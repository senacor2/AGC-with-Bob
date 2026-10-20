/// Geschwindigkeitsänderung (Delta-V) eines Raketentriebwerks.
///
/// # Einheit und Skalierung
/// Meter pro Sekunde (m/s), als `f64`.
///
/// # Verwendung
/// Guidance-Programme (z. B. P40–P47) berechnen den benötigten ΔV-Betrag.
/// Für die vollständige 3D-Richtungsinformation wird ein [`crate::types::Vec3`]
/// verwendet; `DeltaV` repräsentiert den skalaren Betrag oder eine
/// Einzelachsen-Komponente.
///
/// # AGC-Quelle
/// `Comanche055/P40-P47.agc`, DELTAV-Berechnungen im Burn-Programm;
/// `Comanche055/ORBITAL_INTEGRATION.agc`, Zustandsvektoren-Update nach Verbrennung.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct DeltaV(pub f64);

impl DeltaV {
    /// Erstellt einen `DeltaV`-Wert aus Metern pro Sekunde.
    #[inline]
    pub const fn from_m_per_s(v: f64) -> Self {
        Self(v)
    }

    /// Gibt den Wert in Metern pro Sekunde zurück.
    #[inline]
    pub const fn as_m_per_s(self) -> f64 {
        self.0
    }

    /// Betrag des ΔV-Werts (absoluter Wert).
    #[inline]
    pub fn abs(self) -> Self {
        Self(self.0.abs())
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::DeltaV;

    #[test]
    fn roundtrip() {
        let dv = DeltaV::from_m_per_s(3100.0);
        assert!((dv.as_m_per_s() - 3100.0).abs() < 1e-12);
    }

    #[test]
    fn abs_negative() {
        let dv = DeltaV::from_m_per_s(-50.0);
        assert!((dv.abs().as_m_per_s() - 50.0).abs() < 1e-12);
    }

    #[test]
    fn ordering() {
        let a = DeltaV::from_m_per_s(100.0);
        let b = DeltaV::from_m_per_s(200.0);
        assert!(a < b);
    }
}
