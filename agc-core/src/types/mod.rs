//! Gemeinsame physikalische Newtypes für AGC-Core.
//!
//! Jeder Typ trägt seine Einheit und Skalierung im Typ-System, um
//! Verwechslungen zwischen z. B. Radiant und CDU-LSB zur Compilezeit
//! auszuschließen.
//!
//! | Typ          | Einheit / Skalierung                                      |
//! |-------------|-----------------------------------------------------------|
//! | [`Vec3`]    | f64 × 3; SI-Einheiten (m, m/s) je nach Kontext           |
//! | [`CduAngle`]| i16; 1 LSB = 360° / 2^15 ≈ 0.011°                        |
//! | [`Met`]     | f64; Sekunden seit TEPHEM                                 |
//! | [`DeltaV`]  | f64; m/s                                                  |

mod cdu_angle;
mod delta_v;
mod met;
mod vec3;

pub use cdu_angle::CduAngle;
pub use delta_v::DeltaV;
pub use met::Met;
pub use vec3::Vec3;
