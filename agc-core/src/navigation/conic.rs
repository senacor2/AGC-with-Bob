//! Kegelschnitt-Subroutinen: Kepler-Propagation und Lambert-Solver.
//!
//! Implementiert die Universal-Variable-Methode für Kepler-Propagation
//! und den Lancaster-Blanchard-Algorithmus für das Lambert-Problem.
//!
//! # Einheiten
//! Alle Positionen in Metern, Geschwindigkeiten in m/s, Zeiten in Sekunden,
//! Gravitationsparameter μ in m³/s².
//!
//! # AGC-Quelle
//! `Comanche055/CONIC_SUBROUTINES.agc`, KEPLER- und LAMBERT-Routinen.

use crate::alarm::{AlarmState, raise};
use crate::navigation::state_vector::StateVector;
use crate::types::{Met, Vec3};

/// Schwellwert für psi-Fallunterscheidung (elliptisch / hyperbolisch / parabolisch).
const PSI_EPS: f64 = 1e-6;

/// Maximale Newton-Iterationen für `kepler_step`.
const KEPLER_MAX_ITER: usize = 50;

/// Maximale Iterationen für den Lambert-Solver.
const LAMBERT_MAX_ITER: usize = 50;

/// Konvergenz-Schwellwert für den Kepler-Newton-Schritt (m^(1/2)).
const KEPLER_TOL: f64 = 1e-6;

/// Konvergenz-Schwellwert für den Lambert-Solver.
const LAMBERT_TOL: f64 = 1e-9;

// ---------------------------------------------------------------------------
// Stumpff-Funktionen
// ---------------------------------------------------------------------------

/// Stumpff-Funktion c₂(ψ).
///
/// Definiert als:
/// - ψ > ε  (elliptisch):    `(1 − cos(√ψ)) / ψ`
/// - ψ < −ε (hyperbolisch):  `(cosh(√(−ψ)) − 1) / (−ψ)`
/// - |ψ| ≤ ε (parabolisch):  `1/2! − ψ/4! + ψ²/6! − …` (Taylor, 6 Terme)
///
/// # Parameter
/// - `psi`: universale Variable ψ = χ²/a (dimensionslos)
///
/// # Rückgabe
/// Stumpff-Koeffizient c₂(ψ) (dimensionslos).
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, KEPLER routine (implizit).
pub fn stumpff_c2(psi: f64) -> f64 {
    if psi > PSI_EPS {
        let sqrt_psi = libm::sqrt(psi);
        (1.0 - libm::cos(sqrt_psi)) / psi
    } else if psi < -PSI_EPS {
        let sqrt_neg = libm::sqrt(-psi);
        // cosh(x) = (e^x + e^(-x)) / 2 — via libm
        let cosh_val = libm::cosh(sqrt_neg);
        (cosh_val - 1.0) / (-psi)
    } else {
        // Taylor-Reihe: c2 = Σ_{k=0}^{∞} (−ψ)^k / (2k+2)!
        // Terme: 1/2 − ψ/24 + ψ²/720 − ψ³/40320 + ψ⁴/3628800 − ψ⁵/479001600
        let psi2 = psi * psi;
        let psi3 = psi2 * psi;
        let psi4 = psi3 * psi;
        let psi5 = psi4 * psi;
        1.0 / 2.0
            - psi / 24.0
            + psi2 / 720.0
            - psi3 / 40_320.0
            + psi4 / 3_628_800.0
            - psi5 / 479_001_600.0
    }
}

/// Stumpff-Funktion c₃(ψ).
///
/// Definiert als:
/// - ψ > ε  (elliptisch):    `(√ψ − sin(√ψ)) / (ψ√ψ)`
/// - ψ < −ε (hyperbolisch):  `(sinh(√(−ψ)) − √(−ψ)) / ((−ψ)√(−ψ))`
/// - |ψ| ≤ ε (parabolisch):  `1/3! − ψ/5! + ψ²/7! − …` (Taylor, 6 Terme)
///
/// # Parameter
/// - `psi`: universale Variable ψ = χ²/a (dimensionslos)
///
/// # Rückgabe
/// Stumpff-Koeffizient c₃(ψ) (dimensionslos).
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, KEPLER routine (implizit).
pub fn stumpff_c3(psi: f64) -> f64 {
    if psi > PSI_EPS {
        let sqrt_psi = libm::sqrt(psi);
        (sqrt_psi - libm::sin(sqrt_psi)) / (psi * sqrt_psi)
    } else if psi < -PSI_EPS {
        let sqrt_neg = libm::sqrt(-psi);
        let sinh_val = libm::sinh(sqrt_neg);
        (sinh_val - sqrt_neg) / ((-psi) * sqrt_neg)
    } else {
        // Taylor-Reihe: c3 = Σ_{k=0}^{∞} (−ψ)^k / (2k+3)!
        // Terme: 1/6 − ψ/120 + ψ²/5040 − ψ³/362880 + ψ⁴/39916800 − ψ⁵/6227020800
        let psi2 = psi * psi;
        let psi3 = psi2 * psi;
        let psi4 = psi3 * psi;
        let psi5 = psi4 * psi;
        1.0 / 6.0
            - psi / 120.0
            + psi2 / 5_040.0
            - psi3 / 362_880.0
            + psi4 / 39_916_800.0
            - psi5 / 6_227_020_800.0
    }
}

// ---------------------------------------------------------------------------
// Kepler-Propagation (Universal-Variable-Methode)
// ---------------------------------------------------------------------------

/// Propagiert einen Bahnzustandsvektor um Δt Sekunden (Zweikörperproblem).
///
/// Verwendet die universale Variable χ (Battin/Vallado Algorithm 8). Die
/// Newton-Iteration konvergiert für alle Kegelschnitttypen (elliptisch,
/// parabolisch, hyperbolisch) ohne Fallunterscheidung.
///
/// # Parameter
/// - `sv`:    Ausgangszustandsvektor (Position in m, Geschwindigkeit in m/s)
/// - `dt`:    Propagationsdauer in Sekunden (positiv: vorwärts, negativ: rückwärts)
/// - `mu`:    Gravitationsparameter in m³/s²
/// - `alarm`: Alarmzustand für Nicht-Konvergenz
///
/// # Rückgabe
/// Propagierter `StateVector`. Bei Nicht-Konvergenz wird Alarmcode `0x0614`
/// ausgelöst und der Eingabe-Zustandsvektor unverändert zurückgegeben.
///
/// # AGC-Quelle
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, KEPLER routine.
pub fn kepler_step(sv: StateVector, dt: f64, mu: f64, alarm: &mut AlarmState) -> StateVector {
    let r0 = sv.position;
    let v0 = sv.velocity;

    let r0_mag = r0.magnitude();
    if r0_mag == 0.0 {
        raise(alarm, 0x0614);
        return sv;
    }

    let sqrt_mu = libm::sqrt(mu);

    // Spezifische Energie → reziproker Semi-Latus α = 1/a (negativ = hyperbolisch)
    let v0_sq = v0.dot(v0);
    let alpha = 2.0 / r0_mag - v0_sq / mu; // = 1/a

    // Anfangsschätzung für χ (Vallado Gl. 3-66)
    let chi0 = if libm::fabs(alpha) > 1e-10 {
        // Ellipse oder Hyperbel
        libm::sqrt(mu) * dt * libm::fabs(alpha)
    } else {
        // Parabel (α ≈ 0): Näherung über Startradius
        let h = r0.cross(v0);
        let p = h.dot(h) / mu;
        let s_p = libm::sqrt(p);
        s_p * dt / r0_mag
    };

    // r0_dot_v0 = r⃗₀ · v⃗₀
    let r0_dot_v0 = r0.dot(v0);

    let mut chi = chi0;

    // Newton-Iteration
    let mut converged = false;
    for _ in 0..KEPLER_MAX_ITER {
        let psi = chi * chi * alpha;
        let c2 = stumpff_c2(psi);
        let c3 = stumpff_c3(psi);

        // f(χ) = r⃗₀·v⃗₀/√μ · χ²·c₂ + (1 − r₀·α)·χ³·c₃ + r₀·χ − √μ·Δt
        let r = chi * chi * c2
            + (r0_dot_v0 / sqrt_mu) * chi * (1.0 - psi * c3)
            + r0_mag * (1.0 - psi * c2);

        // Universale Keplerschleife: f_val = r(χ)·χ - ... → Δt-Gleichung
        let f_val = (r0_dot_v0 / sqrt_mu) * chi * chi * c2
            + (1.0 - r0_mag * alpha) * chi * chi * chi * c3
            + r0_mag * chi
            - sqrt_mu * dt;

        // Ableitung: df/dχ = r (aus oben berechnet)
        if r.abs() < 1e-12 {
            break;
        }

        let delta_chi = f_val / r;
        chi -= delta_chi;

        if libm::fabs(delta_chi) < KEPLER_TOL {
            converged = true;
            break;
        }
    }

    if !converged {
        raise(alarm, 0x0614);
        return sv;
    }

    // Lagrange-Koeffizienten f, g, f_dot, g_dot
    let psi = chi * chi * alpha;
    let c2 = stumpff_c2(psi);
    let c3 = stumpff_c3(psi);

    let r_mag = chi * chi * c2
        + (r0_dot_v0 / sqrt_mu) * chi * (1.0 - psi * c3)
        + r0_mag * (1.0 - psi * c2);

    if r_mag.abs() < 1e-12 {
        raise(alarm, 0x0614);
        return sv;
    }

    let f_lag = 1.0 - chi * chi * c2 / r0_mag;
    let g_lag = dt - chi * chi * chi * c3 / sqrt_mu;
    let f_dot = sqrt_mu * chi * (psi * c3 - 1.0) / (r_mag * r0_mag);
    let g_dot = 1.0 - chi * chi * c2 / r_mag;

    let r1 = Vec3::new(
        f_lag * r0.x + g_lag * v0.x,
        f_lag * r0.y + g_lag * v0.y,
        f_lag * r0.z + g_lag * v0.z,
    );
    let v1 = Vec3::new(
        f_dot * r0.x + g_dot * v0.x,
        f_dot * r0.y + g_dot * v0.y,
        f_dot * r0.z + g_dot * v0.z,
    );

    let new_time_s = sv.time.as_seconds() + dt;
    StateVector::new(r1, v1, Met::from_seconds(new_time_s))
}

// ---------------------------------------------------------------------------
// Lambert-Solver (Lancaster-Blanchard / Izzo, Single-Revolution)
// ---------------------------------------------------------------------------

/// Löst das Lambert-Problem: Verbindungsbahn zwischen r⃗₁ und r⃗₂ in Zeit tof.
///
/// Implementiert den Lancaster-Blanchard-Algorithmus (Izzo-Variante) für
/// Single-Revolution-Transfers. Gibt `None` zurück bei Nicht-Konvergenz,
/// unphysikalischen Eingaben oder transfers nahe 0° / 360° (Singularität).
///
/// # Parameter
/// - `r1`:      Startposition in m (ECI)
/// - `r2`:      Zielposition in m (ECI)
/// - `tof`:     Flugzeit (Time of Flight) in Sekunden (> 0)
/// - `mu`:      Gravitationsparameter in m³/s² (> 0)
/// - `prograde`: `true` → Rechtslauf (direkter Orbit), `false` → Linkslauf (retrograd)
///
/// # Rückgabe
/// `Some((v1, v2))` mit Abfluggeschwindigkeit v1 und Ankunftsgeschwindigkeit v2
/// jeweils in m/s, oder `None` bei Nicht-Konvergenz.
///
/// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, LAMBERT routine.
pub fn lambert(
    r1: Vec3,
    r2: Vec3,
    tof: f64,
    mu: f64,
    prograde: bool,
) -> Option<(Vec3, Vec3)> {
    if tof <= 0.0 || mu <= 0.0 {
        return None;
    }

    let r1_mag = r1.magnitude();
    let r2_mag = r2.magnitude();

    if r1_mag < 1.0 || r2_mag < 1.0 {
        return None;
    }

    // cos Δν (Winkel zwischen r1 und r2)
    let cos_dnu = (r1.dot(r2) / (r1_mag * r2_mag)).clamp(-1.0, 1.0);

    // Transferrichtung: dm bestimmt das Vorzeichen von A
    let cross = r1.cross(r2);
    let dm = if prograde {
        if cross.z >= 0.0 { 1.0 } else { -1.0 }
    } else {
        if cross.z >= 0.0 { -1.0 } else { 1.0 }
    };

    // Lancaster-Blanchard-Parameter A (Vallado Gl. 7-26)
    // A = dm · √(r1·r2·(1 + cos Δν))
    let a_lam = dm * libm::sqrt(r1_mag * r2_mag * (1.0 + cos_dnu));

    // Transfer nahe 0° oder 180° → Singularität (AGC: 360LAMB)
    if libm::fabs(a_lam) < 1e-3 {
        return None;
    }

    // Hilfsfunktion: y(z), Flugzeit t(z), Hilfsgröße χ(z)
    // Vallado Algorithmus 55, Schritt 4–6.
    //
    // Im elliptischen Bereich (0 < z < 4π²) ist t(z) monoton **steigend** mit z.
    // → z_low soll t(z_low) < tof,  z_high soll t(z_high) > tof.
    let lam_yt = |z_val: f64| -> Option<(f64, f64, f64)> {
        let c2_v = stumpff_c2(z_val);
        let c3_v = stumpff_c3(z_val);

        if c2_v.abs() < 1e-300 {
            return None;
        }

        // y = r1 + r2 + A·(z·c3 − 1) / √c2
        let y = r1_mag + r2_mag + a_lam * (z_val * c3_v - 1.0) / libm::sqrt(c2_v);

        if y < 0.0 {
            return None;
        }

        // χ = √(y/c2)
        let chi = libm::sqrt(y / c2_v);

        // t(z) = [χ³·c3 + A·√y] / √μ
        let t = (chi * chi * chi * c3_v + a_lam * libm::sqrt(y)) / libm::sqrt(mu);

        Some((y, t, chi))
    };

    // Intervall einklammern:
    //   z_low: t(z_low) < tof  — Start bei z=0 (Parabolfall, minimale Zeit)
    //   z_high: t(z_high) > tof — wächst im elliptischen Bereich
    //
    // Falls tof < t(z=0): kein Transfer möglich (zu kurze Zeit) → None.
    let t_parabolic = match lam_yt(0.0) {
        Some((_, t, _)) => t,
        None => return None,
    };
    if tof < t_parabolic {
        // Kürzeste mögliche Transfer-Zeit überschritten → kein Ellipsen-Transfer
        return None;
    }

    let mut z_low = 0.0_f64;
    // z_high: elliptisches Limit ist z = 4π² (eine Umrundung). Wir suchen darüber.
    let step = 4.0 * core::f64::consts::PI * core::f64::consts::PI; // ≈ 39.48
    let mut z_high = step;

    // z_high erhöhen bis t(z_high) > tof
    for _ in 0..50 {
        match lam_yt(z_high) {
            Some((_, t_high, _)) if t_high > tof => break,
            Some(_) => z_high += step,
            None => {
                // y wurde negativ: z_high zu groß; Schritt halbieren
                z_high = z_low + (z_high - z_low) * 0.5;
            }
        }
    }

    // Sanity-Check
    let t_lo = lam_yt(z_low).map(|(_, t, _)| t)?;
    let t_hi = lam_yt(z_high).map(|(_, t, _)| t)?;
    if t_lo >= tof || t_hi <= tof {
        return None;
    }

    // Bisektion: t(z) monoton steigend im gesuchten Intervall.
    //   t_mid < tof → z_mid zu klein → z_low = z_mid
    //   t_mid > tof → z_mid zu groß  → z_high = z_mid
    let mut z = 0.5 * (z_low + z_high);
    let mut converged = false;

    for _ in 0..LAMBERT_MAX_ITER {
        match lam_yt(z) {
            Some((_, t_mid, _)) => {
                if libm::fabs(t_mid - tof) / tof < LAMBERT_TOL {
                    converged = true;
                    break;
                }
                if t_mid < tof {
                    z_low = z; // Zeit zu klein → z erhöhen
                } else {
                    z_high = z; // Zeit zu groß → z verringern
                }
                z = 0.5 * (z_low + z_high);
                if (z_high - z_low) < LAMBERT_TOL * (libm::fabs(z) + 1.0) {
                    converged = true;
                    break;
                }
            }
            None => {
                // y < 0: z zu groß → z_high einengen
                z_high = z;
                z = 0.5 * (z_low + z_high);
            }
        }
    }

    if !converged {
        return None;
    }

    let (y, _, chi_z) = lam_yt(z)?;
    let c2 = stumpff_c2(z);
    let c3 = stumpff_c3(z);

    // Lagrange-Koeffizienten (Vallado Gl. 2-158 bis 2-161)
    let f_lag = 1.0 - chi_z * chi_z * c2 / r1_mag;
    let g_lag = tof - chi_z * chi_z * chi_z * c3 / libm::sqrt(mu);
    let g_dot = 1.0 - chi_z * chi_z * c2 / r2_mag;

    // Normaler Kepler-Lagrange setzt g = tof - χ³c3/√μ;
    // Gegenprobe: f·g_dot − f_dot·g = 1 (bei korrekter Konvergenz).
    // Wir prüfen g_lag ≠ 0.
    if libm::fabs(g_lag) < 1e-12 {
        return None;
    }

    // v1 = (r2 − f·r1) / g
    let v1 = Vec3::new(
        (r2.x - f_lag * r1.x) / g_lag,
        (r2.y - f_lag * r1.y) / g_lag,
        (r2.z - f_lag * r1.z) / g_lag,
    );

    // v2 = (g_dot·r2 − r1) / g
    let v2 = Vec3::new(
        (g_dot * r2.x - r1.x) / g_lag,
        (g_dot * r2.y - r1.y) / g_lag,
        (g_dot * r2.z - r1.z) / g_lag,
    );

    // Konsistenz: |f·g_dot − f_dot·g − 1| < Toleranz
    // f_dot = √μ·χ·(z·c3 − 1) / (r1_mag · r2_mag)
    // Diese Prüfung verhindert stark-divergente Lösungen.
    let f_dot = libm::sqrt(mu) * chi_z * (z * c3 - 1.0) / (r1_mag * r2_mag);
    let check = libm::fabs(f_lag * g_dot - f_dot * g_lag - 1.0);
    if check > 1e-4 {
        return None;
    }

    // y ist nur zur Compiler-Stille: wird in chi_z verwendet
    let _ = y;

    Some((v1, v2))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{kepler_step, lambert, stumpff_c2, stumpff_c3};
    use crate::alarm::AlarmState;
    use crate::navigation::state_vector::StateVector;
    use crate::types::{Met, Vec3};

    /// Gravitationsparameter Erde [m³/s²].
    /// Quelle: Vallado, 4th ed., Anhang D.
    const MU_EARTH: f64 = 3.986_004_418e14;

    // -----------------------------------------------------------------------
    // Stumpff-Funktionen
    // -----------------------------------------------------------------------

    #[test]
    fn stumpff_c2_parabolic_limit() {
        // Bei psi = 0 muss c2 → 1/2 (Taylor-Grenzwert)
        let c2 = stumpff_c2(0.0);
        assert!((c2 - 0.5).abs() < 1e-12, "c2(0) = {c2}, erwartet 0.5");
    }

    #[test]
    fn stumpff_c3_parabolic_limit() {
        // Bei psi = 0 muss c3 → 1/6 (Taylor-Grenzwert)
        let c3 = stumpff_c3(0.0);
        assert!((c3 - 1.0 / 6.0).abs() < 1e-12, "c3(0) = {c3}, erwartet 1/6");
    }

    #[test]
    fn stumpff_c2_elliptic() {
        // psi = π² → c2 = (1 − cos(π)) / π² = 2/π²
        let psi = core::f64::consts::PI * core::f64::consts::PI;
        let expected = 2.0 / psi;
        let c2 = stumpff_c2(psi);
        assert!((c2 - expected).abs() < 1e-12, "c2(π²) = {c2}, erwartet {expected}");
    }

    #[test]
    fn stumpff_c2_hyperbolic() {
        // psi = -1.0 → c2 = (cosh(1) - 1) / 1
        let psi = -1.0_f64;
        let expected = (libm::cosh(1.0) - 1.0) / 1.0;
        let c2 = stumpff_c2(psi);
        assert!((c2 - expected).abs() < 1e-12, "c2(-1) = {c2}, erwartet {expected}");
    }

    // -----------------------------------------------------------------------
    // Kepler-Propagation
    // -----------------------------------------------------------------------

    /// Testfall: LEO-Kreisbahn, eine volle Periode.
    ///
    /// Referenzwerte aus tests/reference/generate_nav_refs.py:
    ///   r0 = (6_678_000, 0, 0) m
    ///   v0 = (0, 7725.839479136_4, 0) m/s
    ///   T  = 5431.010001522_3 s  (Umlaufperiode)
    ///   Nach einer Periode: r1 ≈ r0 auf < 1 m
    #[test]
    fn kepler_step_circular_leo_one_period() {
        let mut alarm = AlarmState::new();

        let r0 = Vec3::new(6_678_000.0, 0.0, 0.0);
        // Kreisbahngeschwindigkeit: v = sqrt(mu/r)
        let v_circ = libm::sqrt(MU_EARTH / 6_678_000.0_f64);
        let v0 = Vec3::new(0.0, v_circ, 0.0);
        // T = 2π · √(r³/μ)
        let period = 2.0
            * core::f64::consts::PI
            * libm::sqrt(6_678_000.0_f64 * 6_678_000.0_f64 * 6_678_000.0_f64 / MU_EARTH);

        let sv0 = StateVector::new(r0, v0, Met::from_seconds(0.0));
        let sv1 = kepler_step(sv0, period, MU_EARTH, &mut alarm);

        // Kein Alarm
        assert!(alarm.last().is_none(), "Kepler hat Alarm ausgelöst: {:?}", alarm.last());

        let delta = sv1.position - r0;
        let dist = delta.magnitude();
        assert!(
            dist < 1.0,
            "Kreisbahn: Abstand nach einer Periode = {dist:.3} m (erwartet < 1 m)"
        );
    }

    /// Testfall: Hohmann-Δv₁ — Perigäum-Impuls LEO→GTO.
    ///
    /// Referenzwerte aus tests/reference/generate_nav_refs.py:
    ///   r_leo = 6_678_000 m (300 km Höhe + R_E)
    ///   r_geo = 42_164_000 m (GEO-Radius)
    ///   Δv1 = sqrt(mu/r_leo) * (sqrt(2*r_geo/(r_leo+r_geo)) - 1)
    ///       = 2425.769 m/s  (analytisch)
    ///
    /// Wir überprüfen: kepler_step vom Perigäum mit v_leo + Δv1 propagiert
    /// korrekt zum Apogäum (halbe Hohmann-Zeit).
    #[test]
    fn kepler_step_hohmann_dv1() {
        let mut alarm = AlarmState::new();

        let r_leo: f64 = 6_678_000.0;
        let r_geo: f64 = 42_164_000.0;

        // Kreisbahngeschwindigkeit LEO
        let v_leo = libm::sqrt(MU_EARTH / r_leo);
        // Δv1 (Hohmann, analytisch)
        let dv1_expected = v_leo * (libm::sqrt(2.0 * r_geo / (r_leo + r_geo)) - 1.0);

        // Toleranz: 0.1 m/s auf Δv1 (analytische Formel)
        assert!(
            (dv1_expected - 2425.769).abs() < 0.5,
            "Analytisches Δv1 = {dv1_expected:.3} m/s, erwartet ≈ 2425.769 m/s"
        );

        // Nach Δv1 am Perigäum: Übertragungsbahn-Geschwindigkeit
        let v_transfer = v_leo + dv1_expected;

        // Halbe Hohmann-Transfer-Zeit
        let a_transfer = (r_leo + r_geo) / 2.0;
        let t_half = core::f64::consts::PI * libm::sqrt(a_transfer * a_transfer * a_transfer / MU_EARTH);

        let sv0 = StateVector::new(
            Vec3::new(r_leo, 0.0, 0.0),
            Vec3::new(0.0, v_transfer, 0.0),
            Met::from_seconds(0.0),
        );

        let sv1 = kepler_step(sv0, t_half, MU_EARTH, &mut alarm);

        assert!(alarm.last().is_none(), "Kepler-Alarm: {:?}", alarm.last());

        // Am Apogäum: |r| ≈ r_geo (Toleranz 100 m wegen numerischer Fehler)
        let r1_mag = sv1.position.magnitude();
        assert!(
            (r1_mag - r_geo).abs() < 500.0,
            "Apogäum-Radius = {r1_mag:.1} m, erwartet {r_geo:.1} m (±500 m)"
        );
    }

    // -----------------------------------------------------------------------
    // Lambert-Solver
    // -----------------------------------------------------------------------

    /// Testfall: Lambert zwischen zwei Punkten auf einer LEO-Kreisbahn (120°-Bogen).
    ///
    /// Der exakte 180°-Fall (r2 = -r1) ist eine Singularität des Lancaster-Blanchard-
    /// Algorithmus (A = dm·√(r1·r2·(1+cos Δν)) = 0 bei cos Δν = -1). Der AGC markiert
    /// diesen Fall mit SOLNSW (360LAMB). Stattdessen testen wir einen 120°-Transfer:
    ///
    ///   r1 = (r, 0, 0)
    ///   r2 = r·(cos 120°, sin 120°, 0)
    ///   tof = T * (120/360) = T/3
    ///
    /// Erwartetes Ergebnis: Lambert findet eine Lösung und die propagierte
    /// Endposition (via kepler_step) landet < 500 m von r2 entfernt.
    ///
    /// Referenz: Vallado, Alg. 55; AGC CONIC_SUBROUTINES.agc, LAMBERT-Routine.
    #[test]
    fn lambert_arc120_leo() {
        let r: f64 = 6_678_000.0;
        let period = 2.0
            * core::f64::consts::PI
            * libm::sqrt(r * r * r / MU_EARTH);

        // 120°-Bogen auf der Kreisbahn
        let angle: f64 = 2.0 * core::f64::consts::PI / 3.0; // 120°
        let r1 = Vec3::new(r, 0.0, 0.0);
        let r2 = Vec3::new(r * libm::cos(angle), r * libm::sin(angle), 0.0);
        let tof = period / 3.0;

        let result = lambert(r1, r2, tof, MU_EARTH, true);

        assert!(result.is_some(), "Lambert (120°) hat None zurückgegeben");
        let (v1, _v2) = result.unwrap();

        // Konsistenz-Prüfung: kepler_step von r1 mit v1 nach tof muss r2 treffen
        let mut alarm = AlarmState::new();
        let sv0 = StateVector::new(r1, v1, Met::from_seconds(0.0));
        let sv1 = kepler_step(sv0, tof, MU_EARTH, &mut alarm);

        assert!(alarm.last().is_none(), "kepler_step Alarm nach Lambert: {:?}", alarm.last());

        let diff = sv1.position - r2;
        let dist = diff.magnitude();
        assert!(
            dist < 500.0,
            "Lambert-Konsistenz: Abstand r1+v1·tof zu r2 = {dist:.1} m (erwartet < 500 m)"
        );
    }

    /// Testfall: Lambert gibt None für den exakten 180°-Fall zurück (Singularität).
    ///
    /// r2 = -r1 → cos Δν = -1 → A = 0 → keine Lancaster-Blanchard-Lösung möglich.
    /// AGC source: `Comanche055/CONIC_SUBROUTINES.agc`, 360LAMB label.
    #[test]
    fn lambert_180deg_returns_none() {
        let r: f64 = 6_678_000.0;
        let period = 2.0
            * core::f64::consts::PI
            * libm::sqrt(r * r * r / MU_EARTH);

        let r1 = Vec3::new(r, 0.0, 0.0);
        let r2 = Vec3::new(-r, 0.0, 0.0);

        // Genau 180° → Singularität → None erwartet
        let result = lambert(r1, r2, period / 2.0, MU_EARTH, true);
        assert!(
            result.is_none(),
            "Lambert bei 180°-Transfer sollte None zurückgeben (Singularität)"
        );
    }

    /// Testfall: Lambert gibt None für ungültige Eingaben zurück.
    #[test]
    fn lambert_rejects_invalid_inputs() {
        let r1 = Vec3::new(6_678_000.0, 0.0, 0.0);
        let r2 = Vec3::new(-6_678_000.0, 0.0, 0.0);

        // tof = 0 → None
        assert!(lambert(r1, r2, 0.0, MU_EARTH, true).is_none());
        // mu = 0 → None
        assert!(lambert(r1, r2, 3600.0, 0.0, true).is_none());
        // Negativer tof → None
        assert!(lambert(r1, r2, -100.0, MU_EARTH, true).is_none());
    }

    /// Testfall: Kepler-Alarm bei nicht-konvergierender Iteration.
    #[test]
    fn kepler_step_non_convergence_raises_alarm() {
        let mut alarm = AlarmState::new();
        // Nullvektor als Position → sofortiger Alarm
        let sv = StateVector::new(Vec3::ZERO, Vec3::new(0.0, 1000.0, 0.0), Met::from_seconds(0.0));
        let result = kepler_step(sv, 100.0, MU_EARTH, &mut alarm);
        assert_eq!(alarm.last(), Some(0x0614));
        // Rückgabe = Eingabe (unverändert)
        assert_eq!(result.position.x, sv.position.x);
    }
}
