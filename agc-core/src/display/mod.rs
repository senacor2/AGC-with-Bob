//! DSKY-Display-Protokoll und Zustandsverwaltung (PINBALL-Subsystem).
//!
//! Dieses Modul übersetzt die PINBALL/DISPLAY-INTERFACE-Konzepte des
//! Comanche055 AGC in reine `no_std`-Rust-Abstraktion — kein TUI,
//! keine Darstellung, nur der Protokollkern.
//!
//! AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`,
//! `Comanche055/DISPLAY_INTERFACE_ROUTINES.agc`.

pub mod dispatcher;
pub mod display_iface;
pub mod noun_resolver;
pub mod verb_noun;

pub use dispatcher::{VerbAction, VerbDispatcher};
pub use display_iface::{DisplayPriority, DisplayQueue, DisplayRequest};
pub use verb_noun::{
    DisplayRegister, DisplayState, InputBuffer, InputMode, KeyCode, KeyEvent, MajorMode, NounCode,
    StatusLights, VerbCode,
};
