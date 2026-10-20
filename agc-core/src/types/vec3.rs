use core::ops::{Add, Sub};

/// Dreidimensionaler Vektor mit `f64`-Komponenten.
///
/// Verwendet für Positions- und Geschwindigkeitsvektoren in navigationsmathematischen
/// Routinen. Alle Komponenten in SI-Einheiten (Meter bzw. Meter/Sekunde), sofern
/// nicht anders angegeben.
///
/// # AGC-Quelle
/// Entspricht dem doppelt-präzisen SP-Vektoren im AGC (2 Worte à 15 Bit je Komponente,
/// also 30 Bit Mantisse). In Rust nutzen wir `f64` (52 Bit Mantisse) — höhere Präzision
/// ist zulässig, da keine AGC-Fidelity-Anforderung Genauigkeit begrenzt.
///
/// Quelle: `Comanche055/ORBITAL_INTEGRATION.agc`, Zustandsvektor-Operationen.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    /// Erstellt einen neuen Vektor aus drei Komponenten.
    #[inline]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Nullvektor.
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    /// Betrag (euklidische Norm) des Vektors.
    #[inline]
    pub fn magnitude(self) -> f64 {
        libm::sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
    }

    /// Quadrat des Betrags (ohne Wurzel — effizienter für Vergleiche).
    #[inline]
    pub fn magnitude_sq(self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Skalarmultiplikation.
    #[inline]
    pub fn scale(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }

    /// Skalarprodukt (Dot Product).
    #[inline]
    pub fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    /// Kreuzprodukt (Cross Product).
    ///
    /// AGC source: `Comanche055/VECTOR_DOUBLE_PRECISION_SUBROUTINES.agc`,
    /// VXSC / VXV-Operationen.
    #[inline]
    pub fn cross(self, rhs: Self) -> Self {
        Self::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x,
        )
    }

    /// Normierter Einheitsvektor. Gibt `None` zurück falls der Betrag null ist.
    #[inline]
    pub fn normalize(self) -> Option<Self> {
        let m = self.magnitude();
        if m == 0.0 {
            None
        } else {
            Some(self.scale(1.0 / m))
        }
    }
}

impl Add for Vec3 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Sub for Vec3 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::Vec3;

    #[test]
    fn zero_vector() {
        assert_eq!(Vec3::ZERO.magnitude(), 0.0);
    }

    #[test]
    fn magnitude_unit_x() {
        let v = Vec3::new(1.0, 0.0, 0.0);
        assert!((v.magnitude() - 1.0).abs() < 1e-15);
    }

    #[test]
    fn magnitude_pythagorean() {
        let v = Vec3::new(3.0, 4.0, 0.0);
        assert!((v.magnitude() - 5.0).abs() < 1e-15);
    }

    #[test]
    fn dot_product() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(4.0, 5.0, 6.0);
        assert!((a.dot(b) - 32.0).abs() < 1e-15);
    }

    #[test]
    fn cross_product_unit_axes() {
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        let z = x.cross(y);
        assert!((z.x - 0.0).abs() < 1e-15);
        assert!((z.y - 0.0).abs() < 1e-15);
        assert!((z.z - 1.0).abs() < 1e-15);
    }

    #[test]
    fn normalize_unit_result() {
        let v = Vec3::new(3.0, 4.0, 0.0);
        let n = v.normalize().unwrap();
        assert!((n.magnitude() - 1.0).abs() < 1e-15);
    }

    #[test]
    fn normalize_zero_returns_none() {
        assert!(Vec3::ZERO.normalize().is_none());
    }

    #[test]
    fn add_sub_roundtrip() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(4.0, -1.0, 0.5);
        let result = (a + b) - b;
        assert!((result.x - a.x).abs() < 1e-15);
        assert!((result.y - a.y).abs() < 1e-15);
        assert!((result.z - a.z).abs() < 1e-15);
    }
}
