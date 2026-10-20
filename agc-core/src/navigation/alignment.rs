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
    use super::triad_align;
    use crate::types::Vec3;

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
