#![no_std]
//! AGC Core — no_std flight-software crate.
//!
//! Implements the navigation, guidance, and control logic of Comanche055
//! (Apollo 11 Command Module AGC) for a Cortex-M7 bare-metal target
//! (STM32 Nucleo-F767ZI, `thumbv7em-none-eabihf`).
//!
//! # Constraints
//! - No heap: `alloc`, `Vec`, `Box`, `String` are forbidden.
//! - No `static mut`: use `cortex_m::interrupt::Mutex<RefCell<T>>`.
//! - No blocking in interrupt handlers or Waitlist tasks.
//! - `panic = "abort"`: every panic triggers GOJAM (hardware restart).

pub mod alarm;
pub mod executive;
pub mod hal;
pub mod navigation;
pub mod restart;
pub mod types;
