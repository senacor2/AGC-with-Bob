//! AGC Simulator — `std`-Wrapper-Crate um `agc-core` und `agc-physics`.
//!
//! Stellt HAL-Implementierungen, eine Sim-Loop-Infrastruktur und
//! Integrationstests bereit.
//!
//! # Crate-Struktur
//! - [`hal`]: Brücke agc-physics ↔ agc-core (Task 11)
//! - TUI / Sim-Loop: folgen in Task 12
//! - End-to-End Integrationstests: folgen in Task 13

pub mod hal;
