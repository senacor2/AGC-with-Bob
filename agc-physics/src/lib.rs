#![cfg_attr(not(feature = "std"), no_std)]
//! AGC Physics — no_std-kompatibles physikalisches Simulationsmodell.
//!
//! Stellt bereit:
//! - [`trajectory`][]: RK4-Integrator für Gravitation + Aerodynamik
//! - [`imu_model`][]:  Simuliertes IMU mit PIPA-Puls-Ausgabe
//! - [`vhf_ranging`][]: VHF-Ranging-Simulation (CM-LM-Distanz)
//! - [`gravity`][]:    Gravitationsmodell (Erde J2, Mond Punkt)
//! - [`bodies`][]:     Physikalische Konstanten (μ, J2, R_eq)
//! - [`aero`][]:       Aerodynamikmodell für Apollo-CM-Kapselprofil
//! - [`atmosphere`][]: Atmosphärenmodell (vereinfachtes NRLMSISE-00)
//!
//! # Hinweis: "Radar" vs. VHF-Ranging
//! Das CM hat kein Rendezvous-Radar — das ist LM-Hardware (Luminary).
//! Was Comanche055 als `RANGERD` bezeichnet, ist VHF-Laufzeitmessung.
//! Daher heißt das Modul [`vhf_ranging`], nicht `radar`.

pub mod aero;
pub mod atmosphere;
pub mod bodies;
pub mod gravity;
pub mod imu_model;
pub mod math;
pub mod trajectory;
pub mod vhf_ranging;
