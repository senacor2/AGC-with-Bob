//! Physikalische Konstanten der Himmelskörper.
//!
//! Enthält gravitationsphysikalische Parameter für Erde und Mond,
//! die im Apollo-Missionsprofil relevant sind.
//!
//! # Einheiten
//! Alle Längen in Metern, Massen indirekt über μ = G·M in m³/s².
//!
//! # AGC-Quelle
//! `Comanche055/ORBITAL_INTEGRATION.agc`, Konstanten-Block.

/// Gravitationsparameter der Erde μ_⊕ = G·M_⊕ (m³/s²).
///
/// Quelle: WGS-84, IERS Conventions 2010.
pub const MU_EARTH: f64 = 3.986_004_418e14;

/// Gravitationsparameter des Mondes μ_☽ = G·M_☽ (m³/s²).
///
/// Quelle: DE430, Tabelle 8.
pub const MU_MOON: f64 = 4.904_869_5e12;

/// Äquatorialradius der Erde R_eq,⊕ (m).
///
/// Quelle: WGS-84.
pub const R_EARTH: f64 = 6_378_137.0;

/// Polarradius der Erde R_pol,⊕ (m).
pub const R_EARTH_POLAR: f64 = 6_356_752.3;

/// Äquatorialradius des Mondes R_eq,☽ (m).
pub const R_MOON: f64 = 1_737_400.0;

/// J2-Zonalkoeffizient der Erde (dimensionslos).
///
/// Beschreibt die Abweichung der Erdgravitation von der reinen Kugelform
/// durch die Abplattung.
///
/// Quelle: EGM-96.
pub const J2_EARTH: f64 = 1.082_63e-3;

/// J2-Zonalkoeffizient des Mondes (dimensionslos).
pub const J2_MOON: f64 = 2.023_5e-4;

/// Standardgravitationsbeschleunigung an der Erdoberfläche g₀ (m/s²).
///
/// Verwendet für PIPA-Skalierung und Atmosphärenmodell.
pub const G0: f64 = 9.806_65;

/// Identifikation eines Zentralkörpers.
///
/// Kapselt alle physikalischen Konstanten des jeweiligen Körpers.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Body {
    /// Erde — LEO, GTO, GEO, TLI-Ausgangsbahn.
    Earth,
    /// Mond — LOI, TEI, Mondumkreisungsbahn.
    Moon,
}

impl Body {
    /// Gravitationsparameter μ = G·M (m³/s²).
    #[inline]
    pub const fn mu(self) -> f64 {
        match self {
            Body::Earth => MU_EARTH,
            Body::Moon => MU_MOON,
        }
    }

    /// J2-Zonalkoeffizient (dimensionslos).
    #[inline]
    pub const fn j2(self) -> f64 {
        match self {
            Body::Earth => J2_EARTH,
            Body::Moon => J2_MOON,
        }
    }

    /// Äquatorialradius R_eq (m).
    #[inline]
    pub const fn radius(self) -> f64 {
        match self {
            Body::Earth => R_EARTH,
            Body::Moon => R_MOON,
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn earth_mu_positive() {
        assert!(MU_EARTH > 0.0);
    }

    #[test]
    fn body_enum_consistency() {
        assert_eq!(Body::Earth.mu(), MU_EARTH);
        assert_eq!(Body::Moon.radius(), R_MOON);
    }
}
