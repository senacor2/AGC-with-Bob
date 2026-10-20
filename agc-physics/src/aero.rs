//! Aerodynamisches Modell für das Apollo-Kommandomodul (CM) beim Wiedereintritt.
//!
//! Liefert Widerstands- (C_D) und Auftriebskoeffizient (C_L) als Funktion
//! von Machzahl und Anstellwinkel sowie den resultierenden Kraft-Vektor in
//! Körperkoordinaten (entgegen Flugrichtung = Drag, senkrecht = Lift).
//!
//! Das Kapselprofil ist kegelförmig mit einem Halbwinkel von 33°, einem
//! kugeligen Wärmeschild und einem Rück-Konus.  Die Koeffizienten basieren
//! auf NASA-Windkanal- und Flugdaten der Apollo-Missionen (Apollo 6, 11).
//!
//! # Einheiten
//! - Machzahl: dimensionslos
//! - Anstellwinkel: Radiant
//! - Kräfte: Newton (wenn mit ½·ρ·v²·A_ref multipliziert)
//! - Referenzfläche A_ref: π·r_max² (CM-Querschnitt = 11.9 m²)
//!
//! # Referenz
//! NASA CR-101098, *Apollo Command Module Aerodynamics*, 1968.
//! Hillje, *Entry Aerodynamics at Lunar Return Conditions Obtained from the
//! Flight of Apollo 4*, NASA TN D-5399, 1969.

/// Referenzfläche des Apollo-CM (Stirnfläche, m²).
///
/// r_CM ≈ 1.9482 m → A = π·r² ≈ 11.90 m²
pub const A_REF: f64 = 11.90;

/// Anzahl der Einträge in den CD-Tabellen.
const N_MACH: usize = 14;

/// Machzahl-Stützstellen für Drag- und Lift-Tabellen.
static MACH_TABLE: [f64; N_MACH] = [
    0.0, 0.5, 0.8, 1.0, 1.2, 1.5, 2.0, 3.0, 5.0, 8.0, 12.0, 18.0, 25.0, 36.0,
];

/// C_D bei Anstellwinkel 0° (Hitzeschild voran, α = 0).
///
/// Quelle: NASA TN D-5399, Fig. 3, α = 0°-Kurve.
static CD0_TABLE: [f64; N_MACH] = [
    1.30, 1.30, 1.32, 1.50, 1.55, 1.50, 1.45, 1.38, 1.28, 1.25, 1.22, 1.20, 1.18, 1.15,
];

/// dC_D/d(α²) — Änderung von C_D mit dem Quadrat des Anstellwinkels (1/rad²).
///
/// Näherung: C_D(α) ≈ C_D0 + k_d · α²  (kleine Anstellwinkel).
static KD_TABLE: [f64; N_MACH] = [
    0.10, 0.10, 0.12, 0.14, 0.16, 0.15, 0.14, 0.12, 0.10, 0.09, 0.08, 0.07, 0.06, 0.05,
];

/// C_L / α — linearer Liftanstieg (1/rad).
///
/// C_L(α) ≈ C_Lα · α (für |α| < 30°).
/// Quelle: NASA TN D-5399, Fig. 4.
static CLA_TABLE: [f64; N_MACH] = [
    0.0, 0.02, 0.05, 0.10, 0.20, 0.22, 0.25, 0.27, 0.28, 0.28, 0.27, 0.25, 0.23, 0.20,
];

// ---------------------------------------------------------------------------
// Interpolation
// ---------------------------------------------------------------------------

/// Lineare Interpolation in einer Stützstellentabelle.
///
/// Beide Felder müssen gleich lang und `mach_table` muss aufsteigend
/// sortiert sein.  Außerhalb des Bereiches wird extrapoliert.
fn interpolate(mach: f64, mach_table: &[f64], value_table: &[f64]) -> f64 {
    let n = mach_table.len();
    debug_assert!(n >= 2);
    debug_assert_eq!(n, value_table.len());

    if mach <= mach_table[0] {
        return value_table[0];
    }
    if mach >= mach_table[n - 1] {
        return value_table[n - 1];
    }

    // Lineare Suche (Tabelle ist kurz)
    for i in 0..n - 1 {
        if mach < mach_table[i + 1] {
            let t = (mach - mach_table[i]) / (mach_table[i + 1] - mach_table[i]);
            return value_table[i] + t * (value_table[i + 1] - value_table[i]);
        }
    }

    value_table[n - 1]
}

// ---------------------------------------------------------------------------
// Öffentliche API
// ---------------------------------------------------------------------------

/// Widerstandskoeffizient C_D des Apollo-CM.
///
/// # Parameter
/// - `mach`:  Machzahl (dimensionslos)
/// - `alpha`: Anstellwinkel in Radiant
///
/// # Rückgabe
/// C_D (dimensionslos).
pub fn drag_coeff(mach: f64, alpha: f64) -> f64 {
    let cd0 = interpolate(mach, &MACH_TABLE, &CD0_TABLE);
    let kd = interpolate(mach, &MACH_TABLE, &KD_TABLE);
    cd0 + kd * alpha * alpha
}

/// Auftriebskoeffizient C_L des Apollo-CM.
///
/// # Parameter
/// - `mach`:  Machzahl (dimensionslos)
/// - `alpha`: Anstellwinkel in Radiant
///
/// # Rückgabe
/// C_L (dimensionslos, Vorzeichen wie α).
pub fn lift_coeff(mach: f64, alpha: f64) -> f64 {
    let cla = interpolate(mach, &MACH_TABLE, &CLA_TABLE);
    cla * alpha
}

/// Lift-to-Drag-Verhältnis (L/D) des Apollo-CM.
///
/// # Parameter
/// - `mach`:  Machzahl (dimensionslos)
/// - `alpha`: Anstellwinkel in Radiant (|α| > 1e-9 erwartet)
pub fn lift_to_drag(mach: f64, alpha: f64) -> f64 {
    let cd = drag_coeff(mach, alpha);
    if cd == 0.0 {
        return 0.0;
    }
    lift_coeff(mach, alpha) / cd
}

/// Aerodynamische Kraft-Komponenten im Windkoordinatensystem.
///
/// Gibt `(F_drag, F_lift)` in Newton zurück für die gegebenen Flugbedingungen.
///
/// Drag wirkt entgegen der Flugrichtung (−v̂), Lift senkrecht dazu in der
/// Ebene aus Geschwindigkeitsvektor und Körperachse.
///
/// # Parameter
/// - `dynamic_pressure`: Staudruck q = ½·ρ·v² in Pa
/// - `mach`:             Machzahl (dimensionslos)
/// - `alpha`:            Anstellwinkel in Radiant
///
/// # Rückgabe
/// `(F_drag_N, F_lift_N)` in Newton.
pub fn aero_forces(dynamic_pressure: f64, mach: f64, alpha: f64) -> (f64, f64) {
    let cd = drag_coeff(mach, alpha);
    let cl = lift_coeff(mach, alpha);
    let f_drag = cd * A_REF * dynamic_pressure;
    let f_lift = cl * A_REF * dynamic_pressure;
    (f_drag, f_lift)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cd_subsonic_positive() {
        let cd = drag_coeff(0.5, 0.0);
        assert!(cd > 0.0, "cd={cd}");
    }

    #[test]
    fn cd_increases_with_aoa() {
        let cd0 = drag_coeff(5.0, 0.0);
        let cd20 = drag_coeff(5.0, libm::atan(1.0)); // 45°
        assert!(cd20 > cd0, "cd mit AoA sollte größer sein");
    }

    #[test]
    fn lift_zero_at_zero_aoa() {
        let cl = lift_coeff(5.0, 0.0);
        assert_eq!(cl, 0.0);
    }

    #[test]
    fn lift_sign_follows_aoa() {
        assert!(lift_coeff(5.0, 0.1) > 0.0);
        assert!(lift_coeff(5.0, -0.1) < 0.0);
    }

    #[test]
    fn aero_forces_positive_drag() {
        let (f_d, _) = aero_forces(1000.0, 2.0, 0.0);
        assert!(f_d > 0.0);
    }

    #[test]
    fn interpolation_at_boundary() {
        // Exakt an Stützstelle = kein Sprung
        let cd_a = drag_coeff(3.0, 0.0);
        let cd_b = drag_coeff(3.0 + 1e-9, 0.0);
        assert!(
            (cd_a - cd_b).abs() < 1e-6,
            "Sprung an Stützstelle: {cd_a} vs {cd_b}"
        );
    }
}
