//! Simulator-HAL — Implementierungen aller `agc-core`-HAL-Traits.
//!
//! Dieses Modul enthält die Brücke zwischen `agc-physics` und `agc-core`:
//! Jeder HAL-Trait aus [`agc_core::hal`] wird hier mit echten Simulationsdaten
//! implementiert.
//!
//! # Übersicht
//! | Modul            | Typ               | Verbindung zu               |
//! |------------------|-------------------|-----------------------------|
//! | [`imu`]          | [`SimImu`]        | `agc_physics::ImuModel`     |
//! | [`dsky`]         | [`SimDsky`]       | TUI-State-Puffer (Task 12)  |
//! | [`rcs`]          | [`SimRcs`]        | Nettokraft für Trajektorie  |
//! | [`sps`]          | [`SimSps`]        | Schubvektor für Trajektorie |
//! | [`vhf_ranging`]  | [`SimVhfRanging`] | `agc_physics::VhfRanging`   |
//! | [`telemetry`]    | [`SimTelemetry`]  | Downlink-Ringpuffer         |
//!
//! # Sim-Loop-Integration
//! Der Sim-Loop (Task 12/13) verwendet diese Typen wie folgt:
//! 1. Vor jedem Tick: `imu.update(specific_force, dt)`, `vhf.tick(cm, lm)`.
//! 2. agc-core läuft seinen T4RUPT-Zyklus und ruft HAL-Methoden auf.
//! 3. Nach dem Tick: `sps.thrust_accel()` + `rcs.net_force_n()` summieren
//!    → `StepParams::thrust_accel` für den RK4-Schritt.

pub mod dsky;
pub mod imu;
pub mod rcs;
pub mod sps;
pub mod telemetry;
pub mod vhf_ranging;

pub use dsky::{DskyState, SimDsky};
pub use imu::SimImu;
pub use rcs::SimRcs;
pub use sps::{SimSps, CSM_MASS_KG, SPS_THRUST_N};
pub use telemetry::SimTelemetry;
pub use vhf_ranging::SimVhfRanging;
