/// Digitale Autopiloten (DAP) — Lage- und Translationsregelung.
///
/// Dieses Modul enthält alle Autopiloten-Algorithmen für das CSM (Comanche055):
///
/// | Untermodul       | Funktion                                             |
/// |------------------|------------------------------------------------------|
/// | [`tvc`]          | TVC-DAP: Pitch- und Yaw-Gimbal-Regelung (SPS-Burn)   |
/// | [`tvc_roll`]     | TVC Roll-DAP: Roll-Regelung via RCS (SPS-Burn)       |
/// | [`rcs_dap`]      | RCS-DAP: Lageregelung im freien Flug                  |
/// | [`jet_select`]   | Jet-Selection-Logic: Achsbefehl → RCS-Bit-Muster     |
/// | [`kalcmanu`]     | KALCMANU: Automatische Lagemanöver-Steuerung         |
/// | [`gimbal_lock`]  | Gimbal-Lock-Erkennung und -Schutz                    |
///
/// # Architektur
/// ```text
///  Guidance (P40/P47)
///       │ rate_cmd (rad/s)
///       ▼
///  ┌──────────┐   gimbal_cmd (i16 LSB)   ┌─────────┐
///  │  tvc     │ ───────────────────────▶ │ SpsHal  │
///  │ (Pitch/  │                          └─────────┘
///  │  Yaw)    │
///  └──────────┘
///
///  ┌──────────┐   jet_bits (u8)           ┌─────────┐
///  │ tvc_roll │ ───────────────────────▶ │ RcsHal  │
///  └──────────┘                          └─────────┘
///
///  ┌──────────┐   attitude_errors         ┌────────────┐   jet_bits
///  │ rcs_dap  │ ──────────────────────▶  │ jet_select │ ────────▶ RcsHal
///  └──────────┘                          └────────────┘
///
///  ┌──────────┐   delta_cdu → RCS-DAP
///  │ kalcmanu │
///  └──────────┘
///
///  ┌─────────────┐   GimbalLockStatus → Alarm
///  │ gimbal_lock │
///  └─────────────┘
/// ```
///
/// # AGC-Quellen
/// - `Comanche055/TVCDAPS.agc`
/// - `Comanche055/TVCROLLDAP.agc`
/// - `Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc`
/// - `Comanche055/JET_SELECTION_LOGIC.agc`
/// - `Comanche055/KALCMANU_STEERING.agc`
/// - `Comanche055/GIMBAL_LOCK_AVOIDANCE.agc`
pub mod gimbal_lock;
pub mod jet_select;
pub mod kalcmanu;
pub mod rcs_dap;
pub mod tvc;
pub mod tvc_roll;
