/// Hardware-Abstraction-Layer (HAL) für agc-core.
///
/// Dieses Modul definiert ausschließlich Trait-Grenzen — keine konkreten
/// Hardware-Implementierungen. Alle Guidance-Programme, DAP-Code und
/// IMU-Routinen referenzieren diese Traits, niemals konkrete Peripherie.
///
/// Implementierungen leben in `agc-sim` (Simulator-HAL, Task 11) und im
/// späteren Embedded-HAL (STM32 Nucleo-F767ZI).
///
/// # CM-spezifischer Hinweis
/// Das Command Module (CM / Comanche055) besitzt **keinen Rendezvous-Radar**
/// — dieser ist LM-only (Luminary). [`VhfRangingHal`] bildet stattdessen das
/// VHF-Ranging (V87/V88, `RANGERD`-Routine in `R31.agc`) ab und ist per
/// [`VhfRangingHal::is_available`] optional aktivierbar.
///
/// # Traits
/// | Trait              | Peripherie               | Haupt-AGC-Kanäle          |
/// |--------------------|--------------------------|---------------------------|
/// | [`ImuHal`]         | IMU / CDU / PIPA         | Kanal 012–016 (octal)     |
/// | [`DskyHal`]        | DSKY Display + Keyboard  | Kanal 010–011, 015–016    |
/// | [`RcsHal`]         | RCS 16 Thruster          | Kanal 005–006             |
/// | [`SpsHal`]         | SPS Gimbal + Engine      | Kanal 011–013             |
/// | [`VhfRangingHal`]  | VHF Ranging (optional)   | Kanal 034                 |
/// | [`TelemetryHal`]   | Down-/Uplink Telemetrie  | Kanal 034–035, 020        |
pub mod dsky;
pub mod imu;
pub mod rcs;
pub mod sps;
pub mod telemetry;
pub mod vhf_ranging;

pub use dsky::DskyHal;
pub use imu::ImuHal;
pub use rcs::RcsHal;
pub use sps::SpsHal;
pub use telemetry::TelemetryHal;
pub use vhf_ranging::VhfRangingHal;

// ---------------------------------------------------------------------------
// Modul-übergreifende Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::dsky::mock::MockDsky;
    use super::imu::mock::MockImu;
    use super::rcs::mock::MockRcs;
    use super::sps::mock::MockSps;
    use super::telemetry::mock::MockTelemetry;
    use super::vhf_ranging::mock::MockVhfRanging;
    use super::{DskyHal, ImuHal, RcsHal, SpsHal, TelemetryHal, VhfRangingHal};

    // --- ImuHal ---

    #[test]
    fn imu_initial_state() {
        let imu = MockImu::new();
        assert_eq!(imu.read_gimbal_angles(), [0, 0, 0]);
        assert!(imu.is_imu_operational());
    }

    #[test]
    fn imu_pipa_pulses_cleared_after_read() {
        let mut imu = MockImu {
            pipa_pulses: [10, -5, 3],
            ..MockImu::new()
        };
        let first = imu.read_pipa_pulses();
        assert_eq!(first, [10, -5, 3]);
        // Nach dem Lesen müssen Pulse zurückgesetzt sein.
        let second = imu.read_pipa_pulses();
        assert_eq!(second, [0, 0, 0]);
    }

    // --- DskyHal ---

    #[test]
    fn dsky_display_word_stored() {
        let mut dsky = MockDsky::new();
        dsky.write_display_word(0x1234);
        assert_eq!(dsky.last_display_word, 0x1234);
    }

    #[test]
    fn dsky_key_queue_fifo() {
        let mut dsky = MockDsky::new();
        dsky.push_key(16); // PRO
        dsky.push_key(25); // ENTR
        assert_eq!(dsky.read_keycode(), Some(16));
        assert_eq!(dsky.read_keycode(), Some(25));
        assert_eq!(dsky.read_keycode(), None);
    }

    // --- RcsHal ---

    #[test]
    fn rcs_jets_stored() {
        let mut rcs = MockRcs::new();
        rcs.set_jets_ab(0b0000_1010);
        rcs.set_jets_cd(0b1100_0000);
        assert_eq!(rcs.jets_ab, 0b0000_1010);
        assert_eq!(rcs.jets_cd, 0b1100_0000);
    }

    // --- SpsHal ---

    #[test]
    fn sps_engine_toggle() {
        let mut sps = MockSps::new();
        assert!(!sps.engine_on());
        sps.set_engine(true);
        assert!(sps.engine_on());
        sps.set_engine(false);
        assert!(!sps.engine_on());
    }

    #[test]
    fn sps_tvc_commands_stored() {
        let mut sps = MockSps::new();
        sps.set_tvc_pitch(128);
        sps.set_tvc_yaw(-64);
        assert_eq!(sps.tvc_pitch, 128);
        assert_eq!(sps.tvc_yaw, -64);
    }

    // --- VhfRangingHal ---

    #[test]
    fn vhf_ranging_not_available_by_default() {
        let ranging = MockVhfRanging::new();
        assert!(!ranging.is_available());
        assert_eq!(ranging.read_range(), None);
    }

    #[test]
    fn vhf_ranging_returns_configured_value() {
        let ranging = MockVhfRanging {
            available: true,
            range_value: Some(4096),
        };
        assert!(ranging.is_available());
        assert_eq!(ranging.read_range(), Some(4096));
    }

    // --- TelemetryHal ---

    #[test]
    fn telemetry_downlink_count() {
        let mut tm = MockTelemetry::new();
        tm.send_downlink_word(0xABCD);
        tm.send_downlink_word(0x1234);
        assert_eq!(tm.downlink_count, 2);
        assert_eq!(tm.last_downlink, 0x1234);
    }

    #[test]
    fn telemetry_uplink_fifo() {
        let mut tm = MockTelemetry::new();
        tm.push_uplink(0x0001);
        tm.push_uplink(0x0002);
        assert_eq!(tm.read_uplink_word(), Some(0x0001));
        assert_eq!(tm.read_uplink_word(), Some(0x0002));
        assert_eq!(tm.read_uplink_word(), None);
    }
}
