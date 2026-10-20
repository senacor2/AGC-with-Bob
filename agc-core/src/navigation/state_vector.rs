use crate::types::{Met, Vec3};

/// Bahn-Zustandsvektor: Position, Geschwindigkeit und Zeitstempel.
///
/// # Einheiten
/// | Feld       | Einheit | Skalierung                              |
/// |------------|---------|------------------------------------------|
/// | `position` | Meter   | ECI J2000, Ursprung Erdmittelpunkt       |
/// | `velocity` | m/s     | ECI J2000, inertiales Koordinatensystem  |
/// | `time`     | Sekunden| seit TEPHEM (AGC-Zeitnullpunkt)          |
///
/// # AGC-Kontext
/// Entspricht dem CHIEFTAN-Segment in Comanche055: Die doppelt-präzisen
/// Zustandsvektoren RATT/VATT (Postion/Geschwindigkeit zur aktuellen Zeit)
/// sowie RONE/VONE (letzter Update-Zeitpunkt). In Rust verwenden wir `f64`
/// statt AGC-Skalierung, da der Hardware-Fixpunktfaktor ein 1960er-Artefakt ist.
///
/// AGC source: `Comanche055/ORBITAL_INTEGRATION.agc`, CHIEFTAN-Segment,
/// RATT/VATT/MET-Variablen.
#[derive(Copy, Clone, Debug)]
pub struct StateVector {
    /// Positionsvektor in m (ECI J2000, Ursprung Erdmittelpunkt).
    pub position: Vec3,
    /// Geschwindigkeitsvektor in m/s (ECI J2000, inertial).
    pub velocity: Vec3,
    /// Zeitstempel: Sekunden seit TEPHEM.
    pub time: Met,
}

impl StateVector {
    /// Erstellt einen neuen `StateVector`.
    ///
    /// # Parameter
    /// - `position`: Positionsvektor in m (ECI J2000)
    /// - `velocity`: Geschwindigkeitsvektor in m/s (ECI J2000)
    /// - `time`: Mission Elapsed Time in Sekunden seit TEPHEM
    #[inline]
    pub const fn new(position: Vec3, velocity: Vec3, time: Met) -> Self {
        Self {
            position,
            velocity,
            time,
        }
    }

    /// Null-Zustandsvektor: Position = (0,0,0) m, Geschwindigkeit = (0,0,0) m/s, Zeit = 0 s.
    ///
    /// Nützlich als Standardwert in statischen Tabellen und Phasen-Einträgen.
    pub const ZERO: Self = Self::new(Vec3::ZERO, Vec3::ZERO, Met::from_seconds(0.0));
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::StateVector;
    use crate::types::{Met, Vec3};

    #[test]
    fn zero_has_zero_fields() {
        let sv = StateVector::ZERO;
        assert_eq!(sv.position.x, 0.0);
        assert_eq!(sv.position.y, 0.0);
        assert_eq!(sv.position.z, 0.0);
        assert_eq!(sv.velocity.x, 0.0);
        assert_eq!(sv.velocity.y, 0.0);
        assert_eq!(sv.velocity.z, 0.0);
        assert_eq!(sv.time.as_seconds(), 0.0);
    }

    #[test]
    fn new_stores_fields_correctly() {
        // LEO-Referenzwerte aus tests/reference/generate_nav_refs.py
        // r0 = (6 678 000.0, 0.0, 0.0) m
        // v0 = (0.0, 7725.8394791364, 0.0) m/s
        // T  = 5431.0100015223 s
        let pos = Vec3::new(6_678_000.0, 0.0, 0.0);
        let vel = Vec3::new(0.0, 7725.839_479_136_4, 0.0);
        let t = Met::from_seconds(5431.010_001_522_3);

        let sv = StateVector::new(pos, vel, t);

        assert!((sv.position.x - 6_678_000.0).abs() < 1e-3);
        assert_eq!(sv.position.y, 0.0);
        assert_eq!(sv.position.z, 0.0);
        assert!((sv.velocity.y - 7725.839_479_136_4).abs() < 1e-9);
        assert!((sv.time.as_seconds() - 5431.010_001_522_3).abs() < 1e-9);
    }

    #[test]
    fn copy_clone_independence() {
        let original = StateVector::new(
            Vec3::new(1.0, 2.0, 3.0),
            Vec3::new(4.0, 5.0, 6.0),
            Met::from_seconds(100.0),
        );
        let copied = original;
        // Copy-Semantik: beide Instanzen sind unabhängig lesbar
        assert_eq!(copied.position.x, original.position.x);
        assert_eq!(copied.time.as_seconds(), original.time.as_seconds());
    }
}
