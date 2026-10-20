//! VHF-Ranging-Simulation: CM-LM-Distanz und Laufzeitmessung.
//!
//! Das Apollo-CM hat kein Rendezvous-Radar — das Radar gehört zum LM
//! (Luminary).  Was Comanche055 als "Radar" bezeichnet, ist eine
//! VHF-Laufzeitmessung: das CM sendet einen Puls, das LM antwortet,
//! das CM misst die Roundtrip-Zeit (RANGERD).
//!
//! Dieses Modul simuliert diesen Prozess:
//! - Berechnet die euklidische Distanz zwischen CM und LM aus ihren
//!   Zustandsvektoren.
//! - Fügt optional ein gaussisches Messrauschen hinzu.
//! - Liefert die Distanz in Metern sowie die entsprechende Signallaufzeit.
//!
//! # AGC-Quelle
//! `Comanche055/RENDEZVOUS_NAVIGATION.agc`, RANGERD-Routine;
//! `Comanche055/RADAR_LEADIN_ROUTINES.agc`.
//!
//! # Koordinatenrahmen
//! Beide Positionen müssen im selben Rahmen (ECI J2000) angegeben werden.
//!
//! # no_std
//! Kein Heap.  Das Rauschen wird mit einem einfachen linearem Kongruenz-
//! generator (LCG) erzeugt — kein `rand`-Crate erforderlich.

use crate::math::Vec3;

/// Lichtgeschwindigkeit (m/s).
const C: f64 = 299_792_458.0;

/// Maximale Reichweite des VHF-Ranging-Systems (m).
///
/// Das AGC ignoriert Reichweiten über 400 NM ≈ 740 km.
/// `Comanche055/RENDEZVOUS_NAVIGATION.agc`, RANGERD.
pub const MAX_RANGE_M: f64 = 740_000.0;

/// Einseitige Standardabweichung des Messrauschens (m).
///
/// Basiert auf dem Systemrauschen (VHF-Transponder + Elektronik):
/// ca. 30 m 1σ (NASA CR-2301, 1973, Abschn. 4.2).
pub const NOISE_STD_M: f64 = 30.0;

// ---------------------------------------------------------------------------
// Einfacher LCG für reproduzierbares Rauschen (no_std)
// ---------------------------------------------------------------------------

/// Linearer Kongruenzgenerator (Park-Miller, 32-Bit).
///
/// Erzeugt ein reproduzierbares Pseudo-Zufallsrauschen ohne `rand`-Abhängigkeit.
/// Nicht kryptographisch sicher; nur für Simulationsrauschen geeignet.
#[derive(Debug)]
struct Lcg32 {
    state: u32,
}

impl Lcg32 {
    const fn new(seed: u32) -> Self {
        Self {
            state: if seed == 0 { 1 } else { seed },
        }
    }

    /// Liefert den nächsten Pseudo-Zufallswert im Bereich [0, 1).
    fn next_f64(&mut self) -> f64 {
        // Knuth-MMIX-Parameter
        self.state = self
            .state
            .wrapping_mul(1_664_525)
            .wrapping_add(1_013_904_223);
        (self.state as f64) / (u32::MAX as f64 + 1.0)
    }

    /// Liefert einen normalverteilten Wert σ=1 (Box-Muller, vereinfacht).
    fn next_normal(&mut self) -> f64 {
        // Box-Muller: braucht zwei gleichverteilte Werte
        let u1 = self.next_f64().max(1e-12);
        let u2 = self.next_f64();
        libm::sqrt(-2.0 * libm::log(u1)) * libm::cos(2.0 * core::f64::consts::PI * u2)
    }
}

// ---------------------------------------------------------------------------
// VhfRanging
// ---------------------------------------------------------------------------

/// VHF-Ranging-Simulator für die CM-LM-Distanz.
///
/// Hält den internen Rausch-Zufallsgenerator und den letzten Messwert.
///
/// # Verwendung
/// ```rust,ignore
/// let mut ranging = VhfRanging::new(42);
/// let meas = ranging.measure(cm_pos, lm_pos);
/// if let Some(m) = meas {
///     // m.range_m an agc-core übergeben
/// }
/// ```
#[derive(Debug)]
pub struct VhfRanging {
    lcg: Lcg32,
    /// `true` wenn Rauschen aktiv ist (false = deterministisch für Tests).
    noise_enabled: bool,
}

impl VhfRanging {
    /// Erstellt einen neuen Ranging-Simulator mit Messrauschen.
    ///
    /// # Parameter
    /// - `seed`: Zufallssaat für den Rausch-Generator.
    pub const fn new(seed: u32) -> Self {
        Self {
            lcg: Lcg32::new(seed),
            noise_enabled: true,
        }
    }

    /// Erstellt einen deterministischen Ranging-Simulator ohne Rauschen.
    ///
    /// Nützlich für unit tests.
    pub const fn noiseless() -> Self {
        Self {
            lcg: Lcg32::new(1),
            noise_enabled: false,
        }
    }

    /// Führt eine VHF-Ranging-Messung durch.
    ///
    /// # Parameter
    /// - `cm_pos`: CM-Position in ECI (m)
    /// - `lm_pos`: LM-Position in ECI (m)
    ///
    /// # Rückgabe
    /// `Some(RangingMeasurement)` wenn die Distanz ≤ [`MAX_RANGE_M`] ist,
    /// `None` wenn außerhalb der Reichweite.
    ///
    /// AGC source: `Comanche055/RENDEZVOUS_NAVIGATION.agc`, RANGERD routine.
    pub fn measure(&mut self, cm_pos: Vec3, lm_pos: Vec3) -> Option<RangingMeasurement> {
        let diff = lm_pos - cm_pos;
        let true_range = diff.magnitude();

        if true_range > MAX_RANGE_M {
            return None;
        }

        let noise = if self.noise_enabled {
            NOISE_STD_M * self.lcg.next_normal()
        } else {
            0.0
        };

        let measured_range = (true_range + noise).max(0.0);
        let roundtrip_time_s = 2.0 * measured_range / C;

        Some(RangingMeasurement {
            range_m: measured_range,
            roundtrip_time_s,
            true_range_m: true_range,
        })
    }
}

/// Ergebnis einer einzelnen VHF-Ranging-Messung.
#[derive(Clone, Copy, Debug)]
pub struct RangingMeasurement {
    /// Gemessene Distanz CM–LM in Metern (mit Rauschen).
    pub range_m: f64,
    /// Roundtrip-Signallaufzeit in Sekunden (aus gemessener Distanz).
    pub roundtrip_time_s: f64,
    /// Wahre euklidische Distanz in Metern (aus Simulation, ohne Rauschen).
    pub true_range_m: f64,
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn noiseless_exact_distance() {
        let cm = Vec3::new(0.0, 0.0, 0.0);
        let lm = Vec3::new(1000.0, 0.0, 0.0);
        let mut r = VhfRanging::noiseless();
        let m = r.measure(cm, lm).expect("innerhalb der Reichweite");
        assert!((m.range_m - 1000.0).abs() < 1e-9, "range={}", m.range_m);
    }

    #[test]
    fn roundtrip_time_consistent() {
        let cm = Vec3::ZERO;
        let lm = Vec3::new(100_000.0, 0.0, 0.0);
        let mut r = VhfRanging::noiseless();
        let m = r.measure(cm, lm).unwrap();
        let expected_t = 2.0 * 100_000.0 / C;
        assert!((m.roundtrip_time_s - expected_t).abs() < 1e-15);
    }

    #[test]
    fn out_of_range_returns_none() {
        let cm = Vec3::ZERO;
        let lm = Vec3::new(MAX_RANGE_M + 1.0, 0.0, 0.0);
        let mut r = VhfRanging::noiseless();
        assert!(r.measure(cm, lm).is_none());
    }

    #[test]
    fn within_range_returns_some() {
        let cm = Vec3::ZERO;
        let lm = Vec3::new(MAX_RANGE_M - 1.0, 0.0, 0.0);
        let mut r = VhfRanging::noiseless();
        assert!(r.measure(cm, lm).is_some());
    }

    #[test]
    fn noisy_measurement_within_bounds() {
        let cm = Vec3::ZERO;
        let lm = Vec3::new(100_000.0, 0.0, 0.0);
        let mut ranging = VhfRanging::new(42);
        // 10 Messungen — alle sollten ≈ 100 km ± 500 m sein
        for _ in 0..10 {
            let m = ranging.measure(cm, lm).unwrap();
            assert!(
                (m.range_m - 100_000.0).abs() < 500.0,
                "Messung zu weit vom Sollwert: {}",
                m.range_m
            );
        }
    }
}
