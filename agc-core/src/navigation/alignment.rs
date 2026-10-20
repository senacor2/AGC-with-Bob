//! IMU-Ausrichtungsroutinen für Comanche055.
//!
//! Implementiert den TRIAD-Algorithmus zur Bestimmung der Rotationsmatrix zwischen
//! Body-Frame und Referenz-Frame aus zwei Vektorbeobachtungen.
//!
//! # Einheiten und Skalierung
//! - Eingabevektoren: beliebige Länge (werden intern normiert)
//! - Ausgabe: dimensionslose 3×3-Rotationsmatrix (Orthogonalmatrix)
//!
//! # AGC-Quelle
//! `Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc`, INFLY-Routine.

use crate::types::Vec3;

// ---------------------------------------------------------------------------
// Konstanten für star_angle_valid
// ---------------------------------------------------------------------------

/// cos(40°) — untere Winkelgrenze des PICAPAR-Kriteriums (Kosinus ist monoton fallend).
const COS_40_DEG: f64 = 0.766_044_443_118_978;
/// cos(66°) — obere Winkelgrenze des PICAPAR-Kriteriums.
const COS_66_DEG: f64 = 0.406_736_643_075_800;

/// CDU-LSB-Auflösung in Radiant: 2π / 32768.
const CDU_LSB_RAD: f64 = core::f64::consts::TAU / 32768.0;

// ---------------------------------------------------------------------------
// CDU-Hilfsfunktionen
// ---------------------------------------------------------------------------

/// Konvertiert einen CDU-Rohwert (i16, Zweierkomplement) in Radiant.
///
/// # Einheiten und Skalierung
/// - Eingabe: CDU-LSB (Least Significant Bit), Zweierkomplement, 1 LSB = 360° / 2¹⁵
/// - Ausgabe: Radiant (f64), Bereich ≈ (−π, +π]
/// - Formel: `cdu as f64 * (2π / 32768)`
///
/// # AGC-Quelle
/// `Comanche055/IMU_CALIBRATION_AND_ALIGNMENT.agc`, CDUTRIG routine.
pub fn cdu_to_rad(cdu: i16) -> f64 {
    cdu as f64 * CDU_LSB_RAD
}

// ---------------------------------------------------------------------------
// Gimbal-Vektor-Konversion
// ---------------------------------------------------------------------------

/// Berechnet den Einheitsvektor der IMU-Stable-Member-Z-Achse im Body-Frame
/// aus den drei CDU-Gimbal-Winkeln (Outer/Middle/Inner).
///
/// # Euler-Konvention
/// Die IMU verwendet die Z–X–Z Euler-Konvention (Outer → Middle → Inner).
/// Die Z-Achse des Stable Member nach der vollen Rotation ergibt sich durch
/// Anwenden der Kugelkoordinaten-Projektion:
///
/// ```text
/// x = sin(mga_r) * cos(oga_r)
/// y = sin(mga_r) * sin(oga_r)
/// z = cos(mga_r)
/// ```
///
/// Der Inner-Gimbal-Winkel (IGA) rotiert die Stable Member um die Sichtlinie (Z-Achse)
/// und beeinflusst daher die Z-Achsen-Projektion nicht. Er wird für diese Berechnung
/// nicht benötigt.
///
/// # Einheiten und Skalierung
/// - Eingabe: CDU-LSB (i16, Zweierkomplement) via [`cdu_to_rad`]
/// - Ausgabe: dimensionsloser Einheitsvektor im Body-Frame
///
/// # AGC-Quelle
/// `Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc`, CALCSMSC routine.
pub fn gimbal_to_vec(oga: i16, mga: i16, _iga: i16) -> Vec3 {
    let oga_r = cdu_to_rad(oga);
    let mga_r = cdu_to_rad(mga);
    let sin_mga = libm::sin(mga_r);
    let cos_mga = libm::cos(mga_r);
    let sin_oga = libm::sin(oga_r);
    let cos_oga = libm::cos(oga_r);
    Vec3::new(sin_mga * cos_oga, sin_mga * sin_oga, cos_mga)
}

// ---------------------------------------------------------------------------
// Sternwinkel-Validierung
// ---------------------------------------------------------------------------

/// Prüft, ob der Winkel zwischen zwei Sternvektoren im gültigen PICAPAR-Bereich liegt.
///
/// P51/P52 akzeptieren nur Sternpaare, deren gegenseitiger Winkel im Bereich
/// [40°, 66°] liegt. Zu kleine Winkel führen zu schlechter Geometrie; zu große
/// Winkel sind optisch schwer messbar.
///
/// # Algorithmus
/// ```text
/// cos θ = (v1̂ · v2̂).clamp(−1, 1)
/// gültig ↔ cos(66°) ≤ cos θ ≤ cos(40°)
/// ```
///
/// Bei Null-Vektoren (nicht normierbar) wird `false` zurückgegeben.
///
/// # AGC-Quelle
/// `Comanche055/P51-P53.agc`, PICAPAR routine (R56, 40°–66° Winkelbereich).
pub fn star_angle_valid(v1: Vec3, v2: Vec3) -> bool {
    let v1n = match v1.normalize() {
        Some(v) => v,
        None => return false,
    };
    let v2n = match v2.normalize() {
        Some(v) => v,
        None => return false,
    };
    let cos_theta = v1n.dot(v2n).clamp(-1.0, 1.0);
    (COS_66_DEG..=COS_40_DEG).contains(&cos_theta)
}

/// Berechnet die Rotationsmatrix vom Body-Frame in den Referenz-Frame mittels TRIAD-Algorithmus.
///
/// Der TRIAD-Algorithmus (Shuster & Oh, 1981) liefert aus zwei Vektorpaaren eine
/// deterministische 3×3-Rotationsmatrix `C_body_to_ref`. Die erste Vektorbeobachtung
/// wird bevorzugt behandelt (höheres Gewicht), die zweite vervollständigt die Basis.
///
/// # Algorithmus
/// ```text
/// t1 = normalize(v1)
/// t2 = normalize(cross(v1, v2))   // Normalenvektor der Beobachtungsebene
/// t3 = cross(t1, t2)              // vervollständigt das rechthändige Triad
/// C = M_ref * M_body^T            // mit M = [t1 | t2 | t3] (Spaltendarstellung)
/// ```
///
/// # Eingabe
/// - `v1_body`: primäre Vektorbeobachtung im Body-Frame (z. B. Sternrichtung)
/// - `v2_body`: sekundäre Vektorbeobachtung im Body-Frame
/// - `v1_ref` : primäre Vektorbeobachtung im Referenz-Frame
/// - `v2_ref` : sekundäre Vektorbeobachtung im Referenz-Frame
///
/// Alle Vektoren dürfen beliebige Länge haben; Null-Vektoren werden abgewiesen.
///
/// # Rückgabe
/// `Some([[f64; 3]; 3])` — Rotationsmatrix (Zeilenweise: Zeile i, Spalte j).
/// `None` falls ein Eingabevektor Null-Länge hat oder v1 und v2 (in einem Frame)
/// parallel sind (keine eindeutige Ebene spannbar).
///
/// # AGC-Quelle
/// `Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc`, INFLY-Routine.
pub fn triad_align(
    v1_body: Vec3,
    v2_body: Vec3,
    v1_ref: Vec3,
    v2_ref: Vec3,
) -> Option<[[f64; 3]; 3]> {
    // --- Referenz-Triad aufbauen ---
    let t1_ref = v1_ref.normalize()?;
    let cross_ref = v1_ref.cross(v2_ref);
    let t2_ref = cross_ref.normalize()?; // None falls v1_ref ∥ v2_ref oder Null
    let t3_ref = t1_ref.cross(t2_ref);

    // --- Body-Triad aufbauen ---
    let t1_body = v1_body.normalize()?;
    let cross_body = v1_body.cross(v2_body);
    let t2_body = cross_body.normalize()?; // None falls v1_body ∥ v2_body oder Null
    let t3_body = t1_body.cross(t2_body);

    // --- C = M_ref * M_body^T aufstellen ---
    // M_ref  = [t1_ref | t2_ref | t3_ref]  (Spaltenvektoren)
    // M_body = [t1_body | t2_body | t3_body]
    // C[i][j] = dot(Zeile i von M_ref, Zeile i von M_body)
    //         = M_ref[i][k] * M_body[j][k]
    //
    // Kompakt: C[i][j] = ref_col_i . body_col_j
    // Da M als Spaltenmatrix: M[zeil][k] entspricht t_k[zeile].
    // C = sum_k  t_ref_k ⊗ t_body_k  (äußeres Produkt, Summation über k=1,2,3)

    let c = outer_add(
        outer_add(outer(t1_ref, t1_body), outer(t2_ref, t2_body)),
        outer(t3_ref, t3_body),
    );

    Some(c)
}

/// Berechnet das äußere Produkt zweier Vektoren: `a ⊗ b` → 3×3-Matrix `M[i][j] = a[i]*b[j]`.
#[inline]
fn outer(a: Vec3, b: Vec3) -> [[f64; 3]; 3] {
    let a = [a.x, a.y, a.z];
    let b = [b.x, b.y, b.z];
    let mut m = [[0.0_f64; 3]; 3];
    let mut i = 0;
    while i < 3 {
        let mut j = 0;
        while j < 3 {
            m[i][j] = a[i] * b[j];
            j += 1;
        }
        i += 1;
    }
    m
}

/// Addiert zwei 3×3-Matrizen komponentenweise.
#[inline]
fn outer_add(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut m = [[0.0_f64; 3]; 3];
    let mut i = 0;
    while i < 3 {
        let mut j = 0;
        while j < 3 {
            m[i][j] = a[i][j] + b[i][j];
            j += 1;
        }
        i += 1;
    }
    m
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{cdu_to_rad, gimbal_to_vec, star_angle_valid, triad_align};
    use crate::types::Vec3;
    use core::f64::consts::{FRAC_PI_2, PI};

    // -----------------------------------------------------------------------
    // cdu_to_rad
    // -----------------------------------------------------------------------

    #[test]
    fn cdu_to_rad_zero() {
        assert_eq!(cdu_to_rad(0), 0.0);
    }

    #[test]
    fn cdu_to_rad_quarter_turn() {
        // 8192 LSB = 2π * 8192/32768 = π/2
        let result = cdu_to_rad(8192);
        assert!(
            (result - FRAC_PI_2).abs() < 1e-9,
            "erwartet π/2, got {result}"
        );
    }

    #[test]
    fn cdu_to_rad_negative_quarter_turn() {
        // -8192 LSB = -π/2
        let result = cdu_to_rad(-8192);
        assert!(
            (result + FRAC_PI_2).abs() < 1e-9,
            "erwartet -π/2, got {result}"
        );
    }

    #[test]
    fn cdu_to_rad_roundtrip() {
        // 1 LSB * 32768 / (2π) soll ≈ 1.0 ergeben
        let result = cdu_to_rad(1) * 32768.0 / (2.0 * PI);
        assert!(
            (result - 1.0).abs() < 1e-12,
            "Roundtrip fehlgeschlagen: {result}"
        );
    }

    // -----------------------------------------------------------------------
    // gimbal_to_vec
    // -----------------------------------------------------------------------

    #[test]
    fn gimbal_to_vec_all_zero() {
        // mga=0 → sin(0)=0, cos(0)=1 → (0, 0, 1)
        let v = gimbal_to_vec(0, 0, 0);
        assert!((v.x).abs() < 1e-12, "x={}", v.x);
        assert!((v.y).abs() < 1e-12, "y={}", v.y);
        assert!((v.z - 1.0).abs() < 1e-12, "z={}", v.z);
    }

    #[test]
    fn gimbal_to_vec_mga_90deg() {
        // oga=0, mga=8192 (≈90°), iga=0 → (sin90·cos0, sin90·sin0, cos90) = (1, 0, 0)
        let v = gimbal_to_vec(0, 8192, 0);
        assert!((v.x - 1.0).abs() < 1e-15, "x={}", v.x);
        assert!((v.y).abs() < 1e-15, "y={}", v.y);
        assert!((v.z).abs() < 1e-15, "z={}", v.z);
    }

    #[test]
    fn gimbal_to_vec_oga_90deg_mga_90deg() {
        // oga=8192 (≈90°), mga=8192 (≈90°) → (sin90·cos90, sin90·sin90, cos90) = (0, 1, 0)
        let v = gimbal_to_vec(8192, 8192, 0);
        assert!((v.x).abs() < 1e-15, "x={}", v.x);
        assert!((v.y - 1.0).abs() < 1e-15, "y={}", v.y);
        assert!((v.z).abs() < 1e-15, "z={}", v.z);
    }

    // -----------------------------------------------------------------------
    // star_angle_valid
    // -----------------------------------------------------------------------

    #[test]
    fn star_angle_valid_50deg() {
        // cos(50°) ≈ 0.6428 liegt in [cos(66°), cos(40°)] → true
        let angle_50_rad = 50.0_f64 * PI / 180.0;
        let v1 = Vec3::new(1.0, 0.0, 0.0);
        let v2 = Vec3::new(libm::cos(angle_50_rad), libm::sin(angle_50_rad), 0.0);
        assert!(star_angle_valid(v1, v2), "50° sollte gültig sein");
    }

    #[test]
    fn star_angle_valid_30deg_too_small() {
        // 30° < 40° → false
        let angle_30_rad = 30.0_f64 * PI / 180.0;
        let v1 = Vec3::new(1.0, 0.0, 0.0);
        let v2 = Vec3::new(libm::cos(angle_30_rad), libm::sin(angle_30_rad), 0.0);
        assert!(!star_angle_valid(v1, v2), "30° sollte ungültig sein");
    }

    #[test]
    fn star_angle_valid_90deg_too_large() {
        // 90° > 66° → false
        let v1 = Vec3::new(1.0, 0.0, 0.0);
        let v2 = Vec3::new(0.0, 1.0, 0.0);
        assert!(!star_angle_valid(v1, v2), "90° sollte ungültig sein");
    }

    #[test]
    fn star_angle_valid_identical_vectors() {
        // 0° → false
        let v = Vec3::new(1.0, 0.0, 0.0);
        assert!(!star_angle_valid(v, v), "0° sollte ungültig sein");
    }

    #[test]
    fn star_angle_valid_null_vector() {
        // Null-Vektor → false
        let v1 = Vec3::new(0.0, 0.0, 0.0);
        let v2 = Vec3::new(1.0, 0.0, 0.0);
        assert!(
            !star_angle_valid(v1, v2),
            "Null-Vektor sollte false liefern"
        );
    }

    /// Hilfsfunktion: Matrizenmultiplikation C^T * C für Orthogonalitätsprüfung.
    fn mat_mul_ct_c(c: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
        let mut r = [[0.0_f64; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                for k in 0..3 {
                    r[i][j] += c[k][i] * c[k][j]; // C^T[i][k] = C[k][i]
                }
            }
        }
        r
    }

    /// v1_body = v1_ref = [1,0,0], v2_body = v2_ref = [0,1,0] → C = Einheitsmatrix.
    #[test]
    fn identity_rotation() {
        let c = triad_align(
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .expect("triad_align sollte Some liefern");

        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (c[i][j] - expected).abs() < 1e-10,
                    "C[{i}][{j}] = {}, erwartet {expected}",
                    c[i][j]
                );
            }
        }
    }

    /// Body: x→x, y→y; Ref: x→y, y→(−x) → Rotation um 90° um Z-Achse (CCW).
    /// Erwartete Matrix: [[0,-1,0],[1,0,0],[0,0,1]]
    #[test]
    fn known_90deg_rotation() {
        let c = triad_align(
            Vec3::new(1.0, 0.0, 0.0),  // v1_body = x
            Vec3::new(0.0, 1.0, 0.0),  // v2_body = y
            Vec3::new(0.0, 1.0, 0.0),  // v1_ref  = y
            Vec3::new(-1.0, 0.0, 0.0), // v2_ref  = -x
        )
        .expect("triad_align sollte Some liefern");

        // Rotation um +90° um Z: R_z(90°) = [[0,-1,0],[1,0,0],[0,0,1]]
        let expected = [[0.0_f64, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (c[i][j] - expected[i][j]).abs() < 1e-10,
                    "C[{i}][{j}] = {}, erwartet {}",
                    c[i][j],
                    expected[i][j]
                );
            }
        }
    }

    /// C^T * C soll Einheitsmatrix sein (Orthogonalitätsbedingung).
    #[test]
    fn output_is_orthogonal() {
        let c = triad_align(
            Vec3::new(1.0, 0.5, 0.1),
            Vec3::new(0.0, 1.0, 0.2),
            Vec3::new(0.7, 0.3, 0.0),
            Vec3::new(0.1, 0.9, 0.4),
        )
        .expect("triad_align sollte Some liefern");

        let ct_c = mat_mul_ct_c(c);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (ct_c[i][j] - expected).abs() < 1e-10,
                    "C^T·C[{i}][{j}] = {}, erwartet {expected}",
                    ct_c[i][j]
                );
            }
        }
    }

    /// Parallele Vektoren im Body-Frame → kein eindeutiges Triad → None.
    #[test]
    fn parallel_vectors_returns_none() {
        let result = triad_align(
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0), // parallel zu v1_body
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        );
        assert!(result.is_none(), "Parallele Vektoren müssen None liefern");
    }
}
