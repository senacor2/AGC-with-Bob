//! Navigationsmathematik für Comanche055.
//!
//! Enthält alle Kern-Navigationsroutinen als `f64`-Rust-Funktionen,
//! entsprechend dem CHIEFTAN-Segment und den zugehörigen AGC-Routinen.
//!
//! # AGC-Quelle
//! `Comanche055/ORBITAL_INTEGRATION.agc`, CHIEFTAN-Segment und verwandte Subroutinen.

pub mod alignment;
pub mod conic;
pub mod ephemeris;
pub mod measurement;
pub mod orbital;
pub mod powered_flight;
pub mod state_vector;
