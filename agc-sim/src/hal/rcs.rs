//! Sim-RCS: [`RcsHal`]-Implementierung für den Terminal-Simulator.
//!
//! `SimRcs` empfängt die Jet-Befehle von `agc-core` und berechnet daraus
//! die resultierende Schubkraft/-moment, die an `agc-physics::Trajectory`
//! weitergegeben werden kann.
//!
//! # AGC-Kanalreferenzen
//! - Kanal **005** (octal): Jet-Befehle Quad A+B (Bits 0–7).
//! - Kanal **006** (octal): Jet-Befehle Quad C+D (Bits 0–7).
//!   Quelle: `Comanche055/JET_SELECTION_LOGIC.agc`, JETSLECT routine.
//!
//! # Schubmodell
//! Jeder der 16 Thruster erzeugt einen Nennschub von 445 N (100 lbf) —
//! entsprechend dem Apollo-SM-RCS-Spezifikation.
//! Die Schubrichtung ist vereinfacht: vier Quads, je vier Thruster,
//! die Nettokraft wird als achsausgerichteter Vektor summiert.
//!
//! Für die Integrationsschicht interessiert nur die akkumulierte Nettokraft
//! (abrufbar über [`SimRcs::net_force_n`]), nicht das Drehmoment.

use agc_core::hal::RcsHal;
use agc_physics::math::Vec3;

// ---------------------------------------------------------------------------
// Thruster-Geometrie
// ---------------------------------------------------------------------------

/// Nennschub eines einzelnen RCS-Thrusters (N).
///
/// Apollo SM RCS: 100 lbf = 444.82 N ≈ 445 N.
const THRUSTER_FORCE_N: f64 = 445.0;

/// Schubrichtungen der 16 RCS-Thruster (in Body-Frame, ECI-näherung).
///
/// Anordnung: 4 Quads (A, B, C, D) × 4 Thruster.
/// Vereinfachte Achsenrichtungen nach AGC ICD:
///
/// Quad A/C: Pitch-Achse (±Y)
/// Quad B/D: Roll/Yaw-Achse (±X, ±Z)
///
/// Mapping: [A1, A2, A3, A4, B1, B2, B3, B4] für Kanal 5,
///          [C1, C2, C3, C4, D1, D2, D3, D4] für Kanal 6.
///
/// Quelle: Apollo Service Module RCS Thruster Arrangement,
/// *Users Guide to Apollo GNCS*, Kap. 7.2.
static THRUST_DIRS_AB: [Vec3; 8] = [
    // Quad A: +Y, -Y, +Y, -Y
    Vec3::new(0.0, 1.0, 0.0),  // A1
    Vec3::new(0.0, -1.0, 0.0), // A2
    Vec3::new(0.0, 1.0, 0.0),  // A3
    Vec3::new(0.0, -1.0, 0.0), // A4
    // Quad B: +X, -X, +Z, -Z
    Vec3::new(1.0, 0.0, 0.0),  // B1
    Vec3::new(-1.0, 0.0, 0.0), // B2
    Vec3::new(0.0, 0.0, 1.0),  // B3
    Vec3::new(0.0, 0.0, -1.0), // B4
];

static THRUST_DIRS_CD: [Vec3; 8] = [
    // Quad C: -Y, +Y, -Y, +Y (gegenüber Quad A)
    Vec3::new(0.0, -1.0, 0.0), // C1
    Vec3::new(0.0, 1.0, 0.0),  // C2
    Vec3::new(0.0, -1.0, 0.0), // C3
    Vec3::new(0.0, 1.0, 0.0),  // C4
    // Quad D: -X, +X, -Z, +Z (gegenüber Quad B)
    Vec3::new(-1.0, 0.0, 0.0), // D1
    Vec3::new(1.0, 0.0, 0.0),  // D2
    Vec3::new(0.0, 0.0, -1.0), // D3
    Vec3::new(0.0, 0.0, 1.0),  // D4
];

// ---------------------------------------------------------------------------
// SimRcs
// ---------------------------------------------------------------------------

/// RCS-HAL-Implementierung für den Simulator.
///
/// Akkumuliert die Nettokraft der aktiven Thruster.  Der Sim-Loop ruft
/// [`SimRcs::net_force_n`] ab, um die Kraft an `agc-physics::Trajectory` zu
/// übergeben.
pub struct SimRcs {
    /// Zuletzt empfangene Jet-Befehle für Quad A+B (Kanal 5).
    pub jets_ab: u8,
    /// Zuletzt empfangene Jet-Befehle für Quad C+D (Kanal 6).
    pub jets_cd: u8,
}

impl SimRcs {
    /// Erstellt ein neues Sim-RCS (alle Jets aus).
    pub fn new() -> Self {
        Self {
            jets_ab: 0,
            jets_cd: 0,
        }
    }

    /// Berechnet die Nettokraft aller aktiven Thruster (N, Body-Frame).
    ///
    /// Der Sim-Loop dividiert diese Kraft durch die Fahrzeugmasse, um die
    /// Schubeinleitung in `agc-physics::StepParams::thrust_accel` zu erhalten.
    ///
    /// AGC source: `Comanche055/JET_SELECTION_LOGIC.agc`, JETSLECT routine.
    pub fn net_force_n(&self) -> Vec3 {
        let mut force = Vec3::ZERO;

        for (i, dir) in THRUST_DIRS_AB.iter().enumerate() {
            if (self.jets_ab >> i) & 1 != 0 {
                force += dir.scale(THRUSTER_FORCE_N);
            }
        }
        for (i, dir) in THRUST_DIRS_CD.iter().enumerate() {
            if (self.jets_cd >> i) & 1 != 0 {
                force += dir.scale(THRUSTER_FORCE_N);
            }
        }

        force
    }

    /// Gibt zurück ob mindestens ein Thruster aktiv ist.
    pub fn any_jet_active(&self) -> bool {
        self.jets_ab != 0 || self.jets_cd != 0
    }
}

impl Default for SimRcs {
    fn default() -> Self {
        Self::new()
    }
}

impl RcsHal for SimRcs {
    /// Empfängt den Jet-Befehl für Quad A+B (Kanal 5).
    ///
    /// AGC source: `Comanche055/JET_SELECTION_LOGIC.agc`, JETSLECT routine.
    fn set_jets_ab(&mut self, bits: u8) {
        self.jets_ab = bits;
    }

    /// Empfängt den Jet-Befehl für Quad C+D (Kanal 6).
    ///
    /// AGC source: `Comanche055/JET_SELECTION_LOGIC.agc`, JETSLECT routine.
    fn set_jets_cd(&mut self, bits: u8) {
        self.jets_cd = bits;
    }
}
