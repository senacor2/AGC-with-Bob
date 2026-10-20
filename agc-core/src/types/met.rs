/// Missionselapsedzeit (Mission Elapsed Time, MET).
///
/// # Einheit und Skalierung
/// Sekunden seit TEPHEM (Time of Ephemeris, der Nullpunkt der AGC-Zeitbasis).  
/// `f64` bietet ausreichend Präzision: Bei einer 14-tägigen Mission und
/// doppelter Fließkommagenauigkeit beträgt der Rundungsfehler < 1 µs.
///
/// # Verwendung
/// Alle zeitabhängigen Navigationsfunktionen nehmen und geben `Met` zurück.
/// Für Differenzen (z. B. Brenndauer) `Met::delta_s` verwenden.
///
/// # AGC-Quelle
/// `Comanche055/TIME_OF_FREE_FALL.agc`, TEPHEM-Referenz;
/// `Comanche055/MEASUREMENT_INCORPORATION.agc`, TIMEDELT-Berechnungen.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Met(pub f64);

impl Met {
    /// Erstellt eine `Met` aus einem Sekundenwert.
    #[inline]
    pub const fn from_seconds(s: f64) -> Self {
        Self(s)
    }

    /// Gibt die Zeit in Sekunden zurück.
    #[inline]
    pub const fn as_seconds(self) -> f64 {
        self.0
    }

    /// Zeitdifferenz zweier `Met`-Werte in Sekunden (`self − other`).
    #[inline]
    pub fn delta_s(self, other: Self) -> f64 {
        self.0 - other.0
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::Met;

    #[test]
    fn roundtrip() {
        let t = Met::from_seconds(86400.0);
        assert!((t.as_seconds() - 86400.0).abs() < 1e-12);
    }

    #[test]
    fn delta() {
        let t1 = Met::from_seconds(1000.0);
        let t2 = Met::from_seconds(1500.0);
        assert!((t2.delta_s(t1) - 500.0).abs() < 1e-12);
    }

    #[test]
    fn ordering() {
        let t1 = Met::from_seconds(100.0);
        let t2 = Met::from_seconds(200.0);
        assert!(t1 < t2);
    }
}
