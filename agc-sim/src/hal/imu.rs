//! Sim-IMU: [`ImuHal`]-Implementierung auf Basis von [`agc_physics::imu_model::ImuModel`].
//!
//! Der Simulator ruft [`SimImu::update`] einmal pro Simulationsschritt mit der
//! spezifischen Kraft (m/s², ECI) auf.  Die dabei akkumulierten PIPA-Pulse
//! werden von `agc-core` über [`ImuHal::read_pipa_pulses`] abgeholt.
//!
//! Gimbal-Winkel werden über eine separate CDU-Winkelquelle gesetzt
//! ([`SimImu::set_gimbal_angles`]).  In der aktuellen Simulation werden sie
//! als fest (keine echte Gyro-Drift-Simulation) behandelt.

use agc_core::hal::ImuHal;
use agc_physics::imu_model::ImuModel;
use agc_physics::math::Vec3;

/// Simulator-IMU — verbindet `agc-physics::ImuModel` mit dem `ImuHal`-Trait.
///
/// # Verwendung im Sim-Loop
/// ```rust,ignore
/// let mut imu = SimImu::new();
/// // Pro Sim-Schritt:
/// imu.update(specific_force_eci, dt_s);
/// // agc-core ruft read_pipa_pulses() im T4RUPT auf.
/// ```
pub struct SimImu {
    model: ImuModel,
    /// Akkumulierte PIPA-Pulse seit dem letzten `read_pipa_pulses`-Aufruf.
    pending_pulses: [i16; 3],
    /// Aktuelle CDU-Gimbalwinkel (Outer, Middle, Inner), LSB-skaliert.
    gimbal_angles: [i16; 3],
    /// IMU als betriebsbereit gemeldet.
    operational: bool,
}

impl SimImu {
    /// Erstellt ein kalibriertes, betriebsbereites Sim-IMU.
    pub fn new() -> Self {
        Self {
            model: ImuModel::new(),
            pending_pulses: [0; 3],
            gimbal_angles: [0; 3],
            operational: true,
        }
    }

    /// Simulationsschritt: Integriert `specific_force` über `dt_s` Sekunden
    /// und akkumuliert PIPA-Pulse.
    ///
    /// # Parameter
    /// - `specific_force`: Spezifische Kraft in m/s² (ECI ≈ SMF, ohne Gravitation)
    /// - `dt_s`:           Simulationsschritt in Sekunden
    ///
    /// AGC source: `Comanche055/IMU_COMPENSATION_PACKAGE.agc`, PIPTIME routine.
    pub fn update(&mut self, specific_force: Vec3, dt_s: f64) {
        let new_pulses = self.model.update(specific_force, dt_s);
        for (i, &p) in new_pulses.iter().enumerate() {
            self.pending_pulses[i] = self.pending_pulses[i].saturating_add(p);
        }
    }

    /// Setzt die CDU-Gimbalwinkel (wird z. B. aus dem Attitude-State gespeist).
    ///
    /// `angles`: `[oga, mga, iga]` — jeweils `i16` LSB-skaliert.
    /// 1 LSB = 360° / 2¹⁵ ≈ 0.011°.
    pub fn set_gimbal_angles(&mut self, angles: [i16; 3]) {
        self.gimbal_angles = angles;
    }

    /// Setzt den IMU-Betriebsstatus (z. B. für Fehlerinjektionstests).
    pub fn set_operational(&mut self, op: bool) {
        self.operational = op;
    }
}

impl Default for SimImu {
    fn default() -> Self {
        Self::new()
    }
}

impl ImuHal for SimImu {
    /// Liest die aktuellen CDU-Gimbalwinkel.
    ///
    /// AGC source: `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc`, CDUTRIG routine.
    fn read_gimbal_angles(&self) -> [i16; 3] {
        self.gimbal_angles
    }

    /// Liest und löscht die seit dem letzten Aufruf akkumulierten PIPA-Pulse.
    ///
    /// AGC source: `Comanche055/T4RUPT_PROGRAM.agc`, PIPASR routine.
    fn read_pipa_pulses(&mut self) -> [i16; 3] {
        let p = self.pending_pulses;
        self.pending_pulses = [0; 3];
        p
    }

    fn is_imu_operational(&self) -> bool {
        self.operational
    }
}
