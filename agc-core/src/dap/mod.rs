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
pub mod task;
pub mod tvc;
pub mod tvc_roll;

// ---------------------------------------------------------------------------
// DapContext — erasable-memory-Brücke zwischen Sim-Loop und DAP-Tasks
// ---------------------------------------------------------------------------

use rcs_dap::RcsDapState;

/// Zentraler DAP-Zustand im `AgcContext`.
///
/// Entspricht dem DAP-Erasable-Memory im echten AGC (DAPDATR, ERRORX/Y/Z,
/// HOLDFLAG, ROLLFIRE usw.).  Der Sim-Loop schreibt die CDU-Winkel vor dem
/// Waitlist-Tick hinein und liest die RCS-Ausgabebits danach heraus.
///
/// # AGC source
/// `Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc`, DAPDATR1 / DAPDATR2 erasable.
#[derive(Debug)]
pub struct DapContext {
    /// RCS-DAP-Zustand (Fehler-Integratoren, Referenzlage, Hold-Flag).
    ///
    /// AGC source: `Comanche055/RCS-CSM_DIGITAL_AUTOPILOT.agc`,
    /// ERRORX / ERRORY / ERRORZ / HOLDFLAG.
    pub rcs: RcsDapState,

    /// Aktuelle CDU-Gimbalwinkel [OGA, MGA, IGA] in CDU-Bits.
    ///
    /// Wird vom Sim-Loop vor jedem DAP-Tick aus `SimImu::read_gimbal_angles()`
    /// übernommen. Ersetzt den direkten IMU-HAL-Zugriff im `no_std`-Task.
    ///
    /// AGC source: `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc`, CDUTRIG.
    pub cdu_angles: [i16; 3],

    /// RCS-Jet-Befehlsbits für Quad A+B (Kanal 5), ausgegeben vom letzten
    /// DAP-Tick.
    ///
    /// Der Sim-Loop liest diesen Wert nach dem Waitlist-Tick und gibt ihn an
    /// `SimRcs::set_jets_ab` weiter.
    ///
    /// AGC source: `Comanche055/JET_SELECTION_LOGIC.agc`, Kanal 005 (octal).
    pub jets_ab: u8,

    /// RCS-Jet-Befehlsbits für Quad C+D (Kanal 6), ausgegeben vom letzten
    /// DAP-Tick.
    ///
    /// AGC source: `Comanche055/JET_SELECTION_LOGIC.agc`, Kanal 006 (octal).
    pub jets_cd: u8,

    /// `true` wenn der RCS-DAP-Waitlist-Task eingetragen ist.
    ///
    /// Verhindert doppeltes Einplanen.
    pub task_scheduled: bool,
}

impl DapContext {
    /// Erstellt einen zurückgesetzten DAP-Zustand (kein Feuer, kein Manöver).
    pub const fn new() -> Self {
        Self {
            rcs: RcsDapState::new(),
            cdu_angles: [0; 3],
            jets_ab: 0,
            jets_cd: 0,
            task_scheduled: false,
        }
    }
}

impl Default for DapContext {
    fn default() -> Self {
        Self::new()
    }
}
