/// IMU Hardware-Abstraction-Layer.
///
/// Deckt die drei Gyroskop-Achsen (X, Y, Z) des Stable-Member sowie die
/// dazugehörigen Coupling-Data-Unit (CDU)-Winkelgeber und Pulse-Integrating
/// Pendulous Accelerometers (PIPAs) ab.
///
/// # AGC-Kanalreferenzen
/// - CDU-Winkel werden über Kanal 12/14 (Gimbal-Winkel), 16 (PIPA) gelesen.
///   Genauer: Kanal **030** (octal) = CDU-Drive-Output; Kanäle **031–033**
///   liefern die drei Gimbal-Winkel (OGA, MGA, IGA) per RHCCTR-Routine.
/// - PIPA-Pulse (Δv-Inkremente) gelesen im T4RUPT-Handler
///   (`Comanche055/T4RUPT_PROGRAM.agc`, PIPASR-Routine).
///
/// # Einheiten
/// - CDU-Winkel: `i16`, 1 LSB = 360° / 2¹⁵ ≈ 0.011°  (Zweier-Komplement).
/// - PIPA-Pulse: `i16`, ein Puls ≈ 0.585 cm/s Δv je Achse.
pub trait ImuHal {
    /// Liest die drei aktuellen CDU-Gimbalwinkel (Outer, Middle, Inner).
    ///
    /// Rückgabe: `[oga, mga, iga]` — jeweils `i16` LSB-skaliert.
    ///
    /// AGC source: `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc`,
    /// CDUTRIG-Routine.
    fn read_gimbal_angles(&self) -> [i16; 3];

    /// Liest und löscht die seit dem letzten Aufruf akkumulierten PIPA-Pulse
    /// auf allen drei Achsen.
    ///
    /// Rückgabe: `[dx, dy, dz]` — Pulse-Zähler, vorzeichenbehaftet.
    ///
    /// AGC source: `Comanche055/T4RUPT_PROGRAM.agc`, PIPASR-Routine.
    fn read_pipa_pulses(&mut self) -> [i16; 3];

    /// Gibt zurück ob das IMU eingeschaltet und kalibriert ist.
    fn is_imu_operational(&self) -> bool;
}

// ---------------------------------------------------------------------------
// Mock-Implementierung für Unit-Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
pub mod mock {
    use super::ImuHal;

    /// Minimaler IMU-Stub — feste Gimbalwinkel, keine PIPA-Pulse.
    pub struct MockImu {
        pub gimbal_angles: [i16; 3],
        pub pipa_pulses: [i16; 3],
        pub operational: bool,
    }

    impl MockImu {
        pub const fn new() -> Self {
            Self {
                gimbal_angles: [0; 3],
                pipa_pulses: [0; 3],
                operational: true,
            }
        }
    }

    impl ImuHal for MockImu {
        fn read_gimbal_angles(&self) -> [i16; 3] {
            self.gimbal_angles
        }

        fn read_pipa_pulses(&mut self) -> [i16; 3] {
            let p = self.pipa_pulses;
            self.pipa_pulses = [0; 3];
            p
        }

        fn is_imu_operational(&self) -> bool {
            self.operational
        }
    }
}
