//! Atmosphärenmodell: Luftdichte und Schallgeschwindigkeit als Funktion der Höhe.
//!
//! Vereinfachtes exponentielles Mehrschichtmodell, das auf den NRLMSISE-00-Referenzdaten
//! basiert.  Für Apollo-Reentry-Berechnungen (Wiedereintritt aus ca. 120 km) ausreichend.
//!
//! Die Schichten sind als konstante Tabelle implementiert (kein Heap), sodass
//! das Modul vollständig `no_std`-kompatibel ist.
//!
//! # Einheiten
//! - Höhe: Meter über MSL
//! - Dichte: kg/m³
//! - Temperatur: Kelvin
//! - Schallgeschwindigkeit: m/s
//!
//! # Referenz
//! U.S. Standard Atmosphere 1976; NRLMSISE-00 Tabellenwerte bis 120 km.

/// Adiabatenexponent für Luft (dimensionslos).
const GAMMA_AIR: f64 = 1.4;

/// Spezifische Gaskonstante Luft R_spez (J/(kg·K)).
const R_AIR: f64 = 287.058;

/// Anzahl der Atmosphärenschichten in der Tabelle.
const N_LAYERS: usize = 12;

/// Tabelle der Atmosphärenschichten.
///
/// Jede Schicht: `(h_base_m, rho_base_kg_m3, scale_height_m)`.
/// Die Dichte folgt dem Exponentialgesetz:
/// `ρ(h) = ρ_base · exp(−(h − h_base) / H_scale)`.
///
/// Quelle: US Standard Atmosphere 1976 (COESA 1976), Tabelle B.
static LAYERS: [(f64, f64, f64); N_LAYERS] = [
    //  h_base (m)   ρ_base (kg/m³)   H_scale (m)
    (0.0, 1.225_0, 8_500.0),
    (11_000.0, 3.639_3e-1, 8_457.0),
    (20_000.0, 8.803_4e-2, 9_462.0),
    (32_000.0, 1.322_5e-2, 11_361.0),
    (47_000.0, 1.427_4e-3, 15_000.0),
    (52_000.0, 8.617_8e-4, 13_526.0),
    (61_000.0, 6.373_5e-5, 11_966.0),
    (79_000.0, 5.196_8e-6, 11_031.0),
    (90_000.0, 3.170_0e-6, 7_200.0),
    (100_000.0, 5.604_0e-7, 5_877.0),
    (110_000.0, 9.708_0e-8, 8_300.0),
    (120_000.0, 2.222_0e-8, 17_000.0),
];

/// Temperatur-Profil (vereinfacht, US Std Atmosphere 1976).
///
/// Jede Schicht: `(h_base_m, T_base_K, lapse_rate_K_per_m)`.
/// Negative Lapse-Rate = Temperaturzunahme mit Höhe (Stratosphäre/Mesosphäre).
static TEMP_LAYERS: [(f64, f64, f64); 7] = [
    (0.0, 288.15, -6.5e-3),     // Troposphäre
    (11_000.0, 216.65, 0.0),    // untere Stratosphäre (isotherm)
    (20_000.0, 216.65, 1.0e-3), // mittlere Stratosphäre
    (32_000.0, 228.65, 2.8e-3),
    (47_000.0, 270.65, 0.0),     // obere Stratosphäre (isotherm)
    (52_000.0, 270.65, -2.8e-3), // Mesosphäre (abkühlen)
    (61_000.0, 252.65, -2.0e-3),
];

// ---------------------------------------------------------------------------
// Öffentliche API
// ---------------------------------------------------------------------------

/// Luftdichte ρ(h) in kg/m³.
///
/// Verwendet ein exponentielles Mehrschichtmodell auf Basis der
/// US Standard Atmosphere 1976.  Oberhalb von 120 km wird die Dichte
/// der obersten Schicht extrapoliert (Exosphäre — vernachlässigbar gering).
///
/// # Parameter
/// - `altitude_m`: Höhe über MSL in Metern
///
/// # Rückgabe
/// Luftdichte in kg/m³.  Bei negativer Höhe wird der Bodenwert zurückgegeben.
pub fn density(altitude_m: f64) -> f64 {
    if altitude_m <= 0.0 {
        return LAYERS[0].1;
    }

    // Passende Schicht suchen (absteigend, damit wir die höchste h_base ≤ h finden)
    let mut layer_idx = 0usize;
    for (i, layer) in LAYERS.iter().enumerate().skip(1) {
        if layer.0 <= altitude_m {
            layer_idx = i;
        } else {
            break;
        }
    }

    let (h_base, rho_base, h_scale) = LAYERS[layer_idx];
    let dh = altitude_m - h_base;
    rho_base * libm::exp(-dh / h_scale)
}

/// Temperatur T(h) in Kelvin (US Standard Atmosphere 1976).
///
/// # Parameter
/// - `altitude_m`: Höhe über MSL in Metern
pub fn temperature(altitude_m: f64) -> f64 {
    if altitude_m <= 0.0 {
        return TEMP_LAYERS[0].1;
    }

    let mut idx = 0usize;
    for (i, layer) in TEMP_LAYERS.iter().enumerate().skip(1) {
        if layer.0 <= altitude_m {
            idx = i;
        } else {
            break;
        }
    }

    let (h_base, t_base, lapse) = TEMP_LAYERS[idx];
    // Oberhalb der letzten bekannten Schicht: vereinfachte Extrapolation
    let dh = (altitude_m - h_base).min(100_000.0);
    let t = t_base + lapse * dh;
    // Minimalwert physikalisch sinnvoll (kein negativer absoluter Temperatur)
    if t < 100.0 {
        100.0
    } else {
        t
    }
}

/// Machzahl-kompatible Schallgeschwindigkeit c(h) in m/s.
///
/// `c = sqrt(γ · R_spez · T(h))`
///
/// # Parameter
/// - `altitude_m`: Höhe über MSL in Metern
pub fn speed_of_sound(altitude_m: f64) -> f64 {
    libm::sqrt(GAMMA_AIR * R_AIR * temperature(altitude_m))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sea_level_density() {
        let rho = density(0.0);
        // Standardwert: 1.225 kg/m³ ± 1%
        assert!((rho - 1.225).abs() < 0.02, "rho={rho}");
    }

    #[test]
    fn density_decreases_with_altitude() {
        assert!(density(10_000.0) < density(0.0));
        assert!(density(50_000.0) < density(10_000.0));
        assert!(density(100_000.0) < density(50_000.0));
    }

    #[test]
    fn density_at_100km_order_of_magnitude() {
        let rho = density(100_000.0);
        // ≈ 5.6e-7 kg/m³
        assert!(rho > 1e-8 && rho < 1e-5, "rho={rho}");
    }

    #[test]
    fn density_negative_altitude_clamped() {
        assert_eq!(density(-1000.0), density(0.0));
    }

    #[test]
    fn sea_level_sound_speed() {
        let c = speed_of_sound(0.0);
        // ~340 m/s ± 5%
        assert!((c - 340.0).abs() < 20.0, "c={c}");
    }
}
