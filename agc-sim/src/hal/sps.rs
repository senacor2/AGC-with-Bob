//! Sim-SPS: [`SpsHal`]-Implementierung für den Terminal-Simulator.
//!
//! `SimSps` empfängt TVC-Gimbal-Befehle und Engine-On/Off-Signale von
//! `agc-core` und übersetzt sie in eine Schubvektorbeschleunigung für
//! `agc-physics::Trajectory`.
//!
//! # SPS-Schubmodell
//! Das SPS-Triebwerk erzeugt einen Vakuumschub von 91 188 N (20 500 lbf).
//! Der Schub wirkt in Body-Z-Richtung (nach hinten, d. h. -Z im ECI-Näherungsrahmen).
//! TVC-Pitch- und Yaw-Befehle lenken den Schubvektor entsprechend ab.
//!
//! # AGC-Kanalreferenzen
//! - Kanal **012** (octal): TVC-Pitch-DAC-Befehl.
//! - Kanal **013** (octal): TVC-Yaw-DAC-Befehl.
//! - Kanal **011** Bit 13: Engine-On/Off (ENGONBIT).
//!   Quelle: `Comanche055/TVCEXECUTIVE.agc`, TVCSTEP routine;
//!   `Comanche055/POWERED_FLIGHT_SUBROUTINES.agc`, IGNITION/CUTOFF routine.
//!
//! # TVC-Skalierung
//! 1 LSB ≈ 0.023° (Vollskala ±2,7° bei 14-Bit-Bereich ±8191 LSB).
//! Quelle: AGC ICD, *Users Guide to Apollo GNCS*, Kap. 7.3.

use agc_core::hal::SpsHal;
use agc_physics::math::Vec3;

// ---------------------------------------------------------------------------
// Physikalische Konstanten
// ---------------------------------------------------------------------------

/// Vakuum-Schub des SPS-Triebwerks (N).
///
/// 20 500 lbf × 4.448 222 N/lbf ≈ 91 188 N.
pub const SPS_THRUST_N: f64 = 91_188.0;

/// Apollo-CM+SM-Masse bei SPS-Burn-Beginn (kg) — Annäherung.
///
/// Realistisch: 28 800 kg (CSM voll), variiert mit Propellantverbrauch.
/// Für die Simulation wird eine feste Masse verwendet (Task 13 kann das verfeinern).
pub const CSM_MASS_KG: f64 = 28_800.0;

/// TVC-Winkel-Skalierungsfaktor: Radiant pro LSB.
///
/// Vollskala ±2.7° = ±0.047 124 rad bei 14-Bit (±8191 LSB).
/// 1 LSB ≈ 0.047 124 / 8191 ≈ 5.75 µrad.
const TVC_RAD_PER_LSB: f64 = 0.047_124 / 8_191.0;

// ---------------------------------------------------------------------------
// SimSps
// ---------------------------------------------------------------------------

/// SPS-HAL-Implementierung für den Simulator.
///
/// Hält den Engine-Zustand und die TVC-Gimbal-Befehle.  Der Sim-Loop ruft
/// [`SimSps::thrust_accel`] ab, um die Schubeinleitung in
/// `agc-physics::StepParams::thrust_accel` zu setzen.
pub struct SimSps {
    /// Aktueller TVC-Pitch-Befehl (AGC-LSB, 14-Bit-Format).
    pub tvc_pitch: i16,
    /// Aktueller TVC-Yaw-Befehl (AGC-LSB, 14-Bit-Format).
    pub tvc_yaw: i16,
    /// `true` wenn das Triebwerk zündet.
    pub engine: bool,
}

impl SimSps {
    /// Erstellt ein neues Sim-SPS (Engine aus, Gimbal auf Null).
    pub fn new() -> Self {
        Self {
            tvc_pitch: 0,
            tvc_yaw: 0,
            engine: false,
        }
    }

    /// Berechnet die aktuelle Schubeinleitung (m/s², ECI-Näherung).
    ///
    /// Gibt [`Vec3::ZERO`] zurück wenn die Engine aus ist.
    ///
    /// Der Schubvektor liegt nominell in +Z-Richtung (positiver Vortrieb).
    /// TVC-Pitch lenkt den Vektor in der XZ-Ebene, TVC-Yaw in der YZ-Ebene ab.
    ///
    /// # Einheiten
    /// Rückgabe in m/s² (Nettobeschleunigung = Schub / Masse).
    ///
    /// AGC source: `Comanche055/TVCEXECUTIVE.agc`, TVCSTEP routine.
    pub fn thrust_accel(&self) -> Vec3 {
        if !self.engine {
            return Vec3::ZERO;
        }

        let pitch_rad = self.tvc_pitch as f64 * TVC_RAD_PER_LSB;
        let yaw_rad = self.tvc_yaw as f64 * TVC_RAD_PER_LSB;

        // Kleiner-Winkel-Näherung für TVC-Ablenkung:
        // a_x = a_nom * sin(yaw)   ≈ a_nom * yaw_rad
        // a_y = a_nom * sin(pitch) ≈ a_nom * pitch_rad
        // a_z = a_nom * cos(pitch)*cos(yaw) ≈ a_nom  (dominanter Term)
        let a_nom = SPS_THRUST_N / CSM_MASS_KG;

        Vec3::new(a_nom * yaw_rad, a_nom * pitch_rad, a_nom)
    }

    /// Gibt zurück ob das Triebwerk aktiv ist.
    pub fn is_burning(&self) -> bool {
        self.engine
    }
}

impl Default for SimSps {
    fn default() -> Self {
        Self::new()
    }
}

impl SpsHal for SimSps {
    /// Speichert den TVC-Pitch-Befehl (Kanal 12).
    ///
    /// AGC source: `Comanche055/TVCEXECUTIVE.agc`, TVCSTEP routine.
    fn set_tvc_pitch(&mut self, command: i16) {
        self.tvc_pitch = command;
    }

    /// Speichert den TVC-Yaw-Befehl (Kanal 13).
    ///
    /// AGC source: `Comanche055/TVCEXECUTIVE.agc`, TVCSTEP routine.
    fn set_tvc_yaw(&mut self, command: i16) {
        self.tvc_yaw = command;
    }

    /// Schaltet das SPS-Triebwerk ein oder aus (Kanal-11 Bit 13, ENGONBIT).
    ///
    /// AGC source: `Comanche055/POWERED_FLIGHT_SUBROUTINES.agc`,
    /// IGNITION / CUTOFF routine.
    fn set_engine(&mut self, on: bool) {
        self.engine = on;
    }

    fn engine_on(&self) -> bool {
        self.engine
    }
}
