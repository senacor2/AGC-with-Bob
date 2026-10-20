/// SPS (Service Propulsion System) Hardware-Abstraction-Layer.
///
/// Das SPS ist das Haupttriebwerk des Service Module (SM). Der AGC steuert
/// die Gimbal-Aktuatoren (Pitch/Yaw) über den TVC-DAP sowie das
/// Engine-On/Off-Signal.
///
/// # AGC-Kanalreferenzen
/// - Kanal **012** (octal): TVC-Pitch-DAC-Befehl (DACP), vorzeichenbehaftetes
///   14-Bit-Wort. 1 LSB ≈ 0.023° Pitch-Abweichung.
/// - Kanal **013** (octal): TVC-Yaw-DAC-Befehl (DACY), analoges Format.
///   Quelle: `Comanche055/TVCEXECUTIVE.agc`, TVCSTEP-Routine.
/// - Engine-On/Off: Kanal **011** Bit 13 (ENGONBIT).
///   Quelle: `Comanche055/POWERED_FLIGHT_SUBROUTINES.agc`, IGNITION-Routine.
///
/// # Einheiten
/// - Gimbal-Befehle: `i16`, 14-Bit-Bereich (−8192 … +8191), Einheit LSB.
///   1 LSB ≈ 0.023° (Vollskala ±2,7° nach AGC ICD).
/// - Engine-Zustand: `bool` — `true` = Zündung aktiv.
pub trait SpsHal {
    /// Setzt den TVC-Pitch-Gimbal-Befehl (Kanal 12).
    ///
    /// `command` ist ein vorzeichenbehafteter 14-Bit-Wert im AGC-LSB-Format.
    ///
    /// AGC source: `Comanche055/TVCEXECUTIVE.agc`, TVCSTEP-Routine.
    fn set_tvc_pitch(&mut self, command: i16);

    /// Setzt den TVC-Yaw-Gimbal-Befehl (Kanal 13).
    ///
    /// `command` ist ein vorzeichenbehafteter 14-Bit-Wert im AGC-LSB-Format.
    ///
    /// AGC source: `Comanche055/TVCEXECUTIVE.agc`, TVCSTEP-Routine.
    fn set_tvc_yaw(&mut self, command: i16);

    /// Schaltet das SPS-Triebwerk ein (`true`) oder aus (`false`).
    ///
    /// Setzt/löscht Kanal-11 Bit 13 (ENGONBIT).
    ///
    /// AGC source: `Comanche055/POWERED_FLIGHT_SUBROUTINES.agc`,
    /// IGNITION / CUTOFF-Routine.
    fn set_engine(&mut self, on: bool);

    /// Gibt zurück ob das Triebwerk aktuell als aktiv gemeldet ist.
    fn engine_on(&self) -> bool;
}

// ---------------------------------------------------------------------------
// Mock-Implementierung für Unit-Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
pub mod mock {
    use super::SpsHal;

    /// Minimaler SPS-Stub — zeichnet Gimbal-Befehle und Engine-Zustand auf.
    pub struct MockSps {
        pub tvc_pitch: i16,
        pub tvc_yaw: i16,
        pub engine: bool,
    }

    impl MockSps {
        pub const fn new() -> Self {
            Self {
                tvc_pitch: 0,
                tvc_yaw: 0,
                engine: false,
            }
        }
    }

    impl SpsHal for MockSps {
        fn set_tvc_pitch(&mut self, command: i16) {
            self.tvc_pitch = command;
        }

        fn set_tvc_yaw(&mut self, command: i16) {
            self.tvc_yaw = command;
        }

        fn set_engine(&mut self, on: bool) {
            self.engine = on;
        }

        fn engine_on(&self) -> bool {
            self.engine
        }
    }
}
