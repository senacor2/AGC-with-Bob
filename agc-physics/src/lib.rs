#![cfg_attr(not(feature = "std"), no_std)]
//! AGC Physics — no_std-compatible physical simulation model.
//!
//! Provides trajectory propagation, IMU modelling, VHF ranging simulation,
//! and aerodynamic forces for the Apollo CM reentry.
//!
//! Designed to be reusable for Hardware-in-the-Loop tests on the target
//! platform without requiring `std` or heap allocation.
//!
//! # Note on "radar"
//! The CM has no rendezvous radar — that hardware is LM-only (Luminary).
//! What Comanche055 calls "radar" is VHF ranging: round-trip VHF signal
//! time-of-flight between CM and LM, activated by V87/V88 (`RANGERD`).
