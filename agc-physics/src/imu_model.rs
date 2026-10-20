//! Simuliertes IMU-Modell: integriert Beschleunigungen und erzeugt PIPA-Pulse.
//!
//! Das Apollo-IMU (Inertial Measurement Unit) enthält drei Pulse Integrating
//! Pendulous Accelerometers (PIPA).  Jede Achse akkumuliert Δv und gibt einen
//! ganzzahligen Puls (`i16`) pro ca. 0.051 m/s Δv aus (Comanche055-Skalierung).
//!
//! Dieses Modul simuliert diesen Prozess für die agc-sim-Schicht:
//! - Nimmt die spezifische Kraft (`specific_force`) in m/s² (Beschleunigung
//!   ohne Gravitation — das, was ein echtes IMU misst) entgegen.
//! - Integriert sie über die Simulationszeit.
//! - Liefert akkumulierte PIPA-Pulse (`i16`) für jede Achse.
//!
//! Der Restbetrag (Sub-LSB) wird zwischen den Schritten beibehalten.
//!
//! # PIPA-Skalierung
//! 1 Puls ≙ 0.051 ft/s ≈ 0.015 544 8 m/s  (Comanche055-Konvention).
//!
//! Quelle: `Comanche055/IMU_COMPENSATION_PACKAGE.agc`, PIPTIME-Routine;
//! *Users Guide to Apollo GNCS*, Kap. 4.3.
//!
//! # Koordinatenrahmen
//! Die spezifische Kraft wird in Stable Member Frame (SMF) erwartet —
//! d.h. der Rahmen, in dem die Gyroskope die Plattform stabil halten.
//! In der Simulation verwenden wir vereinfachend ECI ≈ SMF.
//!
//! # no_std
//! Kein Heap, keine Float-Trap.  `i16`-Overflow wird durch `saturating_add`
//! verhindert; der Aufrufer muss jedoch dafür sorgen, dass die Pulse pro
//! Zeitschritt innerhalb von [`i16::MAX`] bleiben (typisch unkritisch bei dt ≤ 10 s).

use crate::math::Vec3;

/// PIPA-Skalierungsfaktor: Meter pro Sekunde pro Puls (m/s / Puls).
///
/// 1 Puls = 0.051 ft/s = 0.051 × 0.3048 m/s ≈ 0.015 544 8 m/s.
///
/// AGC source: `Comanche055/IMU_COMPENSATION_PACKAGE.agc`, PIPTIME routine.
pub const PIPA_SCALE_MPS: f64 = 0.051 * 0.3048; // ≈ 0.015 545 m/s

/// Achsen-Index für den PIPA-Akkumulator.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    X = 0,
    Y = 1,
    Z = 2,
}

/// Simuliertes IMU-Modell.
///
/// Hält den Δv-Akkumulator für alle drei Achsen.  Der Akkumulator enthält
/// den angesammelten Δv in m/s, der noch nicht als PIPA-Puls ausgegeben wurde.
///
/// # Verwendung
/// ```rust,ignore
/// let mut imu = ImuModel::new();
/// // Pro Simulationsschritt:
/// let pulses = imu.update(specific_force_eci, dt_s);
/// // Pulses an agc-core weiterleiten.
/// ```
#[derive(Debug)]
pub struct ImuModel {
    /// Akkumuliertes Δv in m/s (Subpuls-Reste).
    accum: [f64; 3],
}

impl ImuModel {
    /// Erstellt ein neues, kalibriertes IMU-Modell (Akkumulator = 0).
    pub const fn new() -> Self {
        Self { accum: [0.0; 3] }
    }

    /// Verarbeitet einen Simulationsschritt und gibt PIPA-Pulse zurück.
    ///
    /// # Parameter
    /// - `specific_force`: Spezifische Kraft in m/s² (ECI ≈ SMF, ohne Gravitation)
    /// - `dt_s`:           Simulationsschritt in Sekunden
    ///
    /// # Rückgabe
    /// `[i16; 3]` — PIPA-Pulse für X-, Y-, Z-Achse.
    ///
    /// Ein positiver Wert bedeutet Δv in positiver Achsenrichtung.
    /// Bei Overflow (praktisch unmöglich bei dt ≤ 10 s) wird gesättigt.
    ///
    /// AGC source: `Comanche055/IMU_COMPENSATION_PACKAGE.agc`, PIPTIME routine.
    pub fn update(&mut self, specific_force: Vec3, dt_s: f64) -> [i16; 3] {
        let delta_v = [
            specific_force.x * dt_s,
            specific_force.y * dt_s,
            specific_force.z * dt_s,
        ];

        let mut pulses = [0i16; 3];

        for i in 0..3 {
            self.accum[i] += delta_v[i];

            // Ganzzahlanteil → Pulse (gerundet gegen Null, wie AGC-Hardware)
            let raw = self.accum[i] / PIPA_SCALE_MPS;
            let count = raw as i64; // Abschneiden (Richtung 0)

            // Subpuls-Rest beibehalten
            self.accum[i] -= count as f64 * PIPA_SCALE_MPS;

            // i16 mit Sättigung (verhindert UB, sollte nie auftreten)
            pulses[i] = count.clamp(i16::MIN as i64, i16::MAX as i64) as i16;
        }

        pulses
    }

    /// Gibt den aktuellen Subpuls-Rest (m/s) für eine Achse zurück.
    ///
    /// Nützlich für Integrationstests und Fehleranalyse.
    pub fn residual_mps(&self, axis: Axis) -> f64 {
        self.accum[axis as usize]
    }

    /// Setzt den Akkumulator zurück (z. B. nach GOJAM-Restart).
    pub fn reset(&mut self) {
        self.accum = [0.0; 3];
    }
}

impl Default for ImuModel {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_force_zero_pulses() {
        let mut imu = ImuModel::new();
        let pulses = imu.update(Vec3::ZERO, 1.0);
        assert_eq!(pulses, [0, 0, 0]);
    }

    #[test]
    fn one_pulse_exact() {
        // Exakt 1 PIPA-Puls auf X-Achse in 1 Sekunde
        let force = Vec3::new(PIPA_SCALE_MPS, 0.0, 0.0);
        let mut imu = ImuModel::new();
        let pulses = imu.update(force, 1.0);
        assert_eq!(pulses[0], 1, "X-Puls: {}", pulses[0]);
        // Rest sollte minimal sein
        assert!(imu.residual_mps(Axis::X).abs() < 1e-12);
    }

    #[test]
    fn accumulation_over_multiple_steps() {
        // 0.5 Puls/Schritt → nach 2 Schritten 1 Puls
        let force = Vec3::new(PIPA_SCALE_MPS * 0.5, 0.0, 0.0);
        let mut imu = ImuModel::new();
        let p1 = imu.update(force, 1.0);
        assert_eq!(p1[0], 0, "Erster Schritt noch kein Puls");
        let p2 = imu.update(force, 1.0);
        assert_eq!(p2[0], 1, "Zweiter Schritt: 1 Puls erwartet");
    }

    #[test]
    fn negative_force_negative_pulse() {
        let force = Vec3::new(-PIPA_SCALE_MPS * 2.0, 0.0, 0.0);
        let mut imu = ImuModel::new();
        let pulses = imu.update(force, 1.0);
        assert_eq!(pulses[0], -2);
    }

    #[test]
    fn reset_clears_accumulator() {
        let mut imu = ImuModel::new();
        imu.update(Vec3::new(PIPA_SCALE_MPS * 0.3, 0.0, 0.0), 1.0);
        imu.reset();
        assert_eq!(imu.residual_mps(Axis::X), 0.0);
    }

    #[test]
    fn pipa_scale_correct_value() {
        // Prüfe den Skalierungsfaktor numerisch
        let expected = 0.051 * 0.3048;
        assert!((PIPA_SCALE_MPS - expected).abs() < 1e-10);
    }
}
