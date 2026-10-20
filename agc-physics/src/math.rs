//! Minimale Vektormathematik für `no_std`-Umgebungen.
//!
//! Definiert [`Vec3`] — einen dreidimensionalen Vektor mit `f64`-Komponenten,
//! der für Positions- und Geschwindigkeitsvektoren in physikalischen
//! Simulationsroutinen verwendet wird.

use core::ops::{Add, AddAssign, Mul, Neg, Sub};

/// Dreidimensionaler Vektor mit `f64`-Komponenten.
///
/// Verwendet für Positions- (m) und Geschwindigkeitsvektoren (m/s) in
/// `agc-physics`.  Die Komponenten liegen im ECI J2000-Rahmen, sofern nicht
/// anders angegeben.
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

    /// Betrag (euklidische Norm) in denselben Einheiten wie die Komponenten.
    #[inline]
    pub fn magnitude(self) -> f64 {
        libm::sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
    }

    /// Quadrat des Betrags (ohne Wurzel).
    #[inline]
    pub fn magnitude_sq(self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Skalarmultiplikation.
    #[inline]
    pub fn scale(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }

    /// Skalarprodukt.
    #[inline]
    pub fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    /// Kreuzprodukt.
    #[inline]
    pub fn cross(self, rhs: Self) -> Self {
        Self::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x,
        )
    }

    /// Normierter Einheitsvektor.  Gibt `None` zurück, wenn der Betrag null ist.
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

impl AddAssign for Vec3 {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl Sub for Vec3 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Neg for Vec3 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

/// Ermöglicht `scalar * Vec3`.
impl Mul<Vec3> for f64 {
    type Output = Vec3;
    #[inline]
    fn mul(self, rhs: Vec3) -> Vec3 {
        rhs.scale(self)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::Vec3;

    #[test]
    fn magnitude_unit_x() {
        let v = Vec3::new(1.0, 0.0, 0.0);
        assert!((v.magnitude() - 1.0).abs() < 1e-15);
    }

    #[test]
    fn cross_product() {
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        let z = x.cross(y);
        assert!((z.z - 1.0).abs() < 1e-15);
    }

    #[test]
    fn scalar_mul_commutative() {
        let v = Vec3::new(1.0, 2.0, 3.0);
        let a = 2.0 * v;
        let b = v.scale(2.0);
        assert_eq!(a, b);
    }
}
