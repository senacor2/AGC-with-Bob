//! TUI-Modul — ratatui-basierte DSKY-Oberfläche.
//!
//! # Module
//! - `segment7`: 7-Segment-Zeichensatz (Unicode-Boxzeichen)
//! - `dsky_widget`: DSKY-Panel (Display + Statuslampen + Tastatur)
//! - `state_display`: Zustandsvektor-Panel (Pos, Vel, MET)
//! - `alarm_display`: Alarm-Log-Panel

pub mod alarm_display;
pub mod dsky_widget;
pub mod segment7;
pub mod state_display;
