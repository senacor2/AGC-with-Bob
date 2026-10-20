//! P51, P52, P53 — IMU-Alignment-Programme.
//!
//! P51 und P53 bestimmen die REFSMMAT-Rotationsmatrix aus zwei Sternbeobachtungen
//! (Initial Platform Alignment). P52 berechnet eine Nominale Orientierung aus dem
//! Positions-/Geschwindigkeitsvektor (Reorientation).
//!
//! # Phasenmodell P51/P53
//! - Phase 2: Erste Sternmessung (Gimbal-Winkel → Body-Vektor)
//! - Phase 3: Zweite Sternmessung + Winkelvalidierung (PICAPAR-Kriterium)
//! - Phase 4: TRIAD-Berechnung → REFSMMAT
//! - Phase 0: Abgeschlossen
//!
//! Phase-Table-Gruppe 5 wird für Restart-Sicherheit verwendet.
//!
//! # AGC-Quelle
//! `Comanche055/P51-P53.agc`

use crate::alarm::{raise, AlarmState};
use crate::hal::imu::ImuHal;
use crate::navigation::alignment::{gimbal_to_vec, star_angle_valid, triad_align};
use crate::navigation::star_tables::get_star;
use crate::restart::phase_table::PhaseTable;
use crate::types::Vec3;

// ---------------------------------------------------------------------------
// Typen
// ---------------------------------------------------------------------------

/// Rotationsmatrix Body→Reference-Frame (REFSMMAT).
///
/// Zeilenweise 3×3-Matrix. `C[i][j]` bedeutet: i-te Zeile, j-te Spalte.
/// Orthogonalmatrix: C^T · C = I.
///
/// # Einheit / Skalierung
/// Dimensionslos. Alle Elemente ∈ [−1, +1].
///
/// # AGC-Quelle
/// `Comanche055/P51-P53.agc`, REFSMMAT-Variable (9 Words, 3×3-Matrix).
pub struct Refsmmat(pub [[f64; 3]; 3]);

/// Zustand des P51-Alignment-Programms.
///
/// Hält die gemessenen Body-Vektoren und die aktuelle Phase für
/// Restart-sichere Fortsetzung.
///
/// # AGC-Quelle
/// `Comanche055/P51-P53.agc`, P51 routine, STARIND/STARAD variables.
pub struct P51State {
    /// Aktuelle Phase (2 = erste Sternmessung, 3 = zweite Sternmessung,
    /// 4 = TRIAD berechnen, 0 = done).
    pub phase: u8,
    /// 1-basierter Index des ersten Sterns im AGC-Sternkatalog.
    pub star1_index: usize,
    /// 1-basierter Index des zweiten Sterns im AGC-Sternkatalog.
    pub star2_index: usize,
    /// Gemessener Body-Vektor für Stern 1 (aus Gimbal-Winkeln).
    pub v1_body: Option<Vec3>,
    /// Gemessener Body-Vektor für Stern 2 (aus Gimbal-Winkeln).
    pub v2_body: Option<Vec3>,
}

impl P51State {
    /// Erstellt einen neuen P51-Zustand für ein Sternpaar.
    ///
    /// - `star1_index`: 1-basierter Sternindex (1–37) für die erste Messung.
    /// - `star2_index`: 1-basierter Sternindex (1–37) für die zweite Messung.
    ///
    /// Startet in Phase 2 (erste Messung ausstehend).
    pub const fn new(star1_index: usize, star2_index: usize) -> Self {
        Self {
            phase: 2,
            star1_index,
            star2_index,
            v1_body: None,
            v2_body: None,
        }
    }
}

// ---------------------------------------------------------------------------
// p51_step
// ---------------------------------------------------------------------------

/// Führt einen Schritt des P51 IMU-Alignment-Programms aus.
///
/// Phasen:
/// - Phase 2: Erste Sternmessung (Gimbal-Winkel → Body-Vektor)
/// - Phase 3: Zweite Sternmessung + Winkelvalidierung (PICAPAR)
/// - Phase 4: TRIAD-Berechnung → REFSMMAT
/// - Phase 0: Done
///
/// Gibt `Some(Refsmmat)` zurück wenn Phase 4 erfolgreich abgeschlossen.
/// Gibt `None` zurück wenn noch nicht fertig oder ein Alarm ausgelöst wurde.
///
/// # Alarme
/// - `0x0350`: IMU nicht verfügbar
/// - `0x0351`: Sternwinkel außerhalb des PICAPAR-Bereichs (40°–66°)
///
/// # AGC-Quelle
/// `Comanche055/P51-P53.agc`, P51 routine.
/// `Comanche055/INFLIGHT_ALIGNMENT_ROUTINES.agc`, AXISGEN/TRIAD routine.
pub fn p51_step<I: ImuHal>(
    state: &mut P51State,
    imu: &I,
    pt: &mut PhaseTable,
    alarm: &mut AlarmState,
) -> Option<Refsmmat> {
    if !imu.is_imu_operational() {
        raise(alarm, 0x0350); // IMU nicht verfügbar
        return None;
    }

    match state.phase {
        2 => {
            pt.set_phase(5, 2);
            let [oga, mga, iga] = imu.read_gimbal_angles();
            let v1 = gimbal_to_vec(oga, mga, iga);
            state.v1_body = Some(v1);
            state.phase = 3;
            None // nächsten Aufruf für Phase 3 abwarten
        }
        3 => {
            pt.set_phase(5, 3);
            let [oga, mga, iga] = imu.read_gimbal_angles();
            let v2 = gimbal_to_vec(oga, mga, iga);
            // Winkelvalidierung (PICAPAR-Kriterium)
            if let Some(v1) = state.v1_body {
                if !star_angle_valid(v1, v2) {
                    raise(alarm, 0x0351); // Sternwinkel ungültig
                    return None;
                }
            }
            state.v2_body = Some(v2);
            state.phase = 4;
            None
        }
        4 => {
            pt.set_phase(5, 4);
            let v1_body = state.v1_body?;
            let v2_body = state.v2_body?;
            // Referenzvektoren aus Sternkatalog
            let v1_ref = get_star(state.star1_index)?;
            let v2_ref = get_star(state.star2_index)?;
            // TRIAD-Algorithmus → REFSMMAT
            let mat = triad_align(v1_body, v2_body, v1_ref, v2_ref)?;
            pt.set_phase(5, 0); // Gruppe inaktiv
            state.phase = 0;
            Some(Refsmmat(mat))
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// p52_reorient
// ---------------------------------------------------------------------------

/// Berechnet die Nominale Orientierung aus dem Positions-/Geschwindigkeitsvektor.
///
/// Nominale Orientierung:
/// - X-Achse = −r̂ (lokale Vertikal, nach unten)
/// - Z-Achse = r̂ × v̂ (Orbit-Normalvektor, ≈ H-Vektor-Richtung)
/// - Y-Achse = Z × X (vervollständigt Rechtssystem)
///
/// # Einheiten und Skalierung
/// - `position`: m (ECI J2000)
/// - `velocity`: m/s (ECI J2000)
/// - Rückgabe: dimensionslose 3×3-Rotationsmatrix als `Refsmmat`
///
/// Gibt `None` zurück bei Null-Vektoren (kein gültiger r̂ oder h-Vektor).
///
/// # AGC-Quelle
/// `Comanche055/P51-P53.agc`, S52.3 routine.
pub fn p52_reorient(
    position: Vec3,
    velocity: Vec3,
    pt: &mut PhaseTable,
    alarm: &mut AlarmState,
) -> Option<Refsmmat> {
    // Alarm-Parameter wird für zukünftige Alarm-Erweiterungen mitgeführt;
    // im aktuellen Scope wird bei Null-Vektoren None zurückgegeben.
    let _ = alarm;

    pt.set_phase(5, 2);

    let r_hat = position.normalize()?;
    let x_axis = Vec3::new(-r_hat.x, -r_hat.y, -r_hat.z); // -r̂

    let h = position.cross(velocity);
    let z_axis = h.normalize()?; // Orbit-Normal

    let y_axis = z_axis.cross(x_axis).normalize()?;

    // Rotationsmatrix aus Spaltenvektoren
    // Zeile i, Spalte j: j-te Basisachse, i-te Komponente
    let mat = [
        [x_axis.x, y_axis.x, z_axis.x],
        [x_axis.y, y_axis.y, z_axis.y],
        [x_axis.z, y_axis.z, z_axis.z],
    ];

    pt.set_phase(5, 0);
    Some(Refsmmat(mat))
}

// ---------------------------------------------------------------------------
// P53 = P51
// ---------------------------------------------------------------------------

/// P53 ist identisch mit P51.
///
/// Im AGC-Quellcode steht `P53 EQUALS P51`; Rust bildet dies als Funktions-Alias ab.
///
/// AGC source: `Comanche055/P51-P53.agc` — `P53 EQUALS P51`.
pub use p51_step as p53_step;

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{p51_step, p52_reorient, P51State, Refsmmat};
    use crate::alarm::AlarmState;
    use crate::hal::imu::mock::MockImu;
    use crate::navigation::star_tables::get_star;
    use crate::restart::phase_table::PhaseTable;
    use crate::types::Vec3;

    /// Hilfsfunktion: C^T * C für Orthogonalitätsprüfung.
    fn mat_ctc(c: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
        let mut r = [[0.0_f64; 3]; 3];
        let mut i = 0;
        while i < 3 {
            let mut j = 0;
            while j < 3 {
                let mut k = 0;
                while k < 3 {
                    r[i][j] += c[k][i] * c[k][j];
                    k += 1;
                }
                j += 1;
            }
            i += 1;
        }
        r
    }

    // -----------------------------------------------------------------------
    // Test 1: identity_alignment
    //
    // Wenn v1_body == v1_ref und v2_body == v2_ref, muss TRIAD die
    // Identitätsmatrix liefern.
    // Sternvektoren aus Katalog (Stern 1 und 3) werden direkt als Body-Vektoren gesetzt.
    // -----------------------------------------------------------------------
    #[test]
    fn identity_alignment() {
        let v1_ref = get_star(1).unwrap();
        let v2_ref = get_star(3).unwrap();

        // State direkt auf Phase 4 setzen, Messungen = Katalogwerte
        let mut state = P51State {
            phase: 4,
            star1_index: 1,
            star2_index: 3,
            v1_body: Some(v1_ref),
            v2_body: Some(v2_ref),
        };

        let mut imu = MockImu::new();
        imu.operational = true;
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        let result = p51_step(&mut state, &mut imu, &mut pt, &mut alarm);
        assert!(result.is_some(), "Phase 4 → REFSMMAT erwartet");

        let Refsmmat(mat) = result.unwrap();
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (mat[i][j] - expected).abs() < 1e-10,
                    "Identität: mat[{i}][{j}] = {:.12}, erwartet {expected}",
                    mat[i][j]
                );
            }
        }
        // Phase-Table Gruppe 5 muss deaktiviert sein
        assert_eq!(pt.get_phase(5), Some(0));
        assert!(alarm.last().is_none());
    }

    // -----------------------------------------------------------------------
    // Test 2: known_rotation_alignment
    //
    // v1_body=[1,0,0], v2_body=[0,1,0] (Identität im Body)
    // v1_ref=[0,1,0],  v2_ref=[-1,0,0]  → 90°-Rotation um Z
    // Erwartete REFSMMAT: [[0,-1,0],[1,0,0],[0,0,1]]
    // -----------------------------------------------------------------------
    #[test]
    fn known_rotation_alignment() {
        let state = P51State {
            phase: 4,
            star1_index: 1,
            star2_index: 2,
            v1_body: Some(Vec3::new(1.0, 0.0, 0.0)),
            v2_body: Some(Vec3::new(0.0, 1.0, 0.0)),
        };

        // Überschreibe die Sternkatalog-Vektoren durch direktes Einsetzen
        // von Ersatz-Sternindizes; da get_star() aufgerufen wird, nutzen wir
        // einen Trick: wir setzen den State auf Phase 4 und rufen triad_align
        // indirekt auf. Da wir v1_ref/v2_ref aus dem Katalog nicht kontrollieren
        // können, nutzen wir hier eine direkte TRIAD-Überprüfung mit bekannten Vektoren.
        //
        // Stattdessen: Funktion direkt mit bekannten Vektoren testen.
        use crate::navigation::alignment::triad_align;
        let mat = triad_align(
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(-1.0, 0.0, 0.0),
        )
        .expect("triad_align soll Some liefern");

        // R_z(+90°): [[0,-1,0],[1,0,0],[0,0,1]]
        let expected = [[0.0_f64, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (mat[i][j] - expected[i][j]).abs() < 1e-10,
                    "90°-Rotation: mat[{i}][{j}] = {:.12}, erwartet {:.1}",
                    mat[i][j],
                    expected[i][j]
                );
            }
        }

        // State nicht weiter gebraucht — Compiler-Warnung vermeiden
        let _ = state;
    }

    // -----------------------------------------------------------------------
    // Test 3: restart_resume_phase3
    //
    // Simuliert Neustart in Phase 3: v1_body bereits gesetzt,
    // MockImu mit oga=0, mga=0, iga=0 → gimbal_to_vec → (0,0,1) ≈ Stern 5.
    // Stern 5 = [0.0131, 0.0078, 0.9999] ≈ Pol, Stern 1 als zweites Paar.
    // Zur Winkelvalidierung müssen v1_body und neues v2 ≥ 40° auseinander liegen.
    // -----------------------------------------------------------------------
    #[test]
    fn restart_resume_phase3() {
        // Stern 1 als v1_body (Katalogwert)
        let v1 = get_star(1).unwrap(); // [0.8749, 0.026, 0.4837]

        // MockImu: mga=8192 → 90° → (1,0,0) — weit entfernt von Stern 1
        // Winkel zwischen [0.8749, 0.026, 0.4837] und [1,0,0]:
        // dot ≈ 0.875 → acos ≈ 29° → zu klein (< 40°)
        //
        // Stattdessen: mga=4096 → 45° → (sin45·cos0, sin45·sin0, cos45) ≈ (0.707, 0, 0.707)
        // dot(v1, v2) = 0.8749*0.707 + 0 + 0.4837*0.707 ≈ 0.618+0.342 = 0.960 → ~16° → zu klein
        //
        // Wähle stattdessen v1 so, dass der Winkel zu (0,0,1) valide ist.
        // gimbal_to_vec(0,0,0) = (0,0,1) (da mga=0 → cos(0)=1, sin(0)=0)
        // Sterne 1 und 3 haben Winkel:
        // dot(star1, star3) = 0.8749*0.4776 + 0.026*0.1166 + 0.4837*0.8708
        //                   ≈ 0.418 + 0.003 + 0.421 ≈ 0.842 → acos ≈ 32.6° → zu klein
        //
        // Wähle Stern 1 als v1_body und gimbal-Vektor für Stern 8:
        // star8 ≈ [0.4106, 0.4988, 0.7633]
        // dot(star1, star8) ≈ 0.8749*0.4106 + 0.026*0.4988 + 0.4837*0.7633
        //                   ≈ 0.3592 + 0.013 + 0.369 = 0.741 → acos ≈ 42.3° → gültig!
        //
        // Setzen wir state.v1_body = star8 (bekannter Vektor)
        // und imu-Gimbalwinkel so, dass gimbal_to_vec → (1,0,0) [Stern ≈ 90°]:
        // dot(star8, [1,0,0]) = 0.4106 → acos ≈ 65.8° → gültig (≤ 66°)!

        let v1_preset = get_star(8).unwrap(); // [0.4106, 0.4988, 0.7633]

        // gimbal_to_vec(0, 8192, 0): oga=0, mga=8192≈90° → (sin90·cos0, sin90·sin0, cos90) = (1,0,0)
        let mut imu = MockImu::new();
        imu.gimbal_angles = [0, 8192, 0]; // → (1.0, 0.0, 0.0)
        imu.operational = true;

        let mut state = P51State {
            phase: 3,
            star1_index: 8,
            star2_index: 1,
            v1_body: Some(v1_preset),
            v2_body: None,
        };

        let mut pt = PhaseTable::new();
        pt.set_phase(5, 3).unwrap();
        let mut alarm = AlarmState::new();

        // Schritt 1: Phase 3 → v2_body setzen, phase = 4
        let r1 = p51_step(&mut state, &mut imu, &mut pt, &mut alarm);
        assert!(r1.is_none(), "Phase 3 gibt None zurück");
        assert_eq!(state.phase, 4, "Phase soll nach Phase 3 auf 4 stehen");
        assert!(state.v2_body.is_some(), "v2_body soll gesetzt sein");
        assert!(alarm.last().is_none(), "Kein Alarm erwartet");

        // Schritt 2: Phase 4 → REFSMMAT
        let r2 = p51_step(&mut state, &mut imu, &mut pt, &mut alarm);
        assert!(r2.is_some(), "Phase 4 → REFSMMAT erwartet");

        let Refsmmat(mat) = r2.unwrap();
        // Orthogonalitätscheck
        let ctc = mat_ctc(&mat);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (ctc[i][j] - expected).abs() < 1e-9,
                    "Orthogonal: C^T·C[{i}][{j}] = {:.12}, erwartet {expected}",
                    ctc[i][j]
                );
            }
        }

        let _ = v1; // nicht verwendet, aber dokumentiert
    }

    // -----------------------------------------------------------------------
    // Test 4: angle_too_small_returns_none
    //
    // v1_body = (0,0,1), imu → gimbal_to_vec(0,0,0) = (0,0,1)
    // Winkel = 0° → star_angle_valid → false → None + Alarm 0x0351
    // -----------------------------------------------------------------------
    #[test]
    fn angle_too_small_returns_none() {
        let mut state = P51State {
            phase: 3,
            star1_index: 5,
            star2_index: 1,
            v1_body: Some(Vec3::new(0.0, 0.0, 1.0)), // identisch mit gimbal_to_vec(0,0,0)
            v2_body: None,
        };

        let mut imu = MockImu::new();
        imu.gimbal_angles = [0, 0, 0]; // → (0.0, 0.0, 1.0)
        imu.operational = true;

        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        let result = p51_step(&mut state, &mut imu, &mut pt, &mut alarm);
        assert!(result.is_none(), "Ungültiger Winkel → None erwartet");
        assert_eq!(alarm.last(), Some(0x0351), "Alarm 0x0351 erwartet");
    }

    // -----------------------------------------------------------------------
    // Test 5: imu_not_operational
    // -----------------------------------------------------------------------
    #[test]
    fn imu_not_operational() {
        let mut state = P51State::new(1, 3);

        let mut imu = MockImu::new();
        imu.operational = false;

        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        let result = p51_step(&mut state, &mut imu, &mut pt, &mut alarm);
        assert!(result.is_none(), "IMU nicht operational → None erwartet");
        assert_eq!(alarm.last(), Some(0x0350), "Alarm 0x0350 erwartet");
    }

    // -----------------------------------------------------------------------
    // Test 6: p52_reorient_produces_orthogonal_matrix
    //
    // Kreisbahn auf x-Achse: r = [6778000, 0, 0], v = [0, 7673, 0]
    // Erwarte: orthogonale Matrix, X-Achse ≈ [-1, 0, 0]
    // -----------------------------------------------------------------------
    #[test]
    fn p52_reorient_produces_orthogonal_matrix() {
        let position = Vec3::new(6_778_000.0, 0.0, 0.0);
        let velocity = Vec3::new(0.0, 7673.0, 0.0);

        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        let result = p52_reorient(position, velocity, &mut pt, &mut alarm);
        assert!(result.is_some(), "p52_reorient soll Some liefern");

        let Refsmmat(mat) = result.unwrap();

        // Orthogonalitätsprüfung: C^T · C = I
        let ctc = mat_ctc(&mat);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (ctc[i][j] - expected).abs() < 1e-10,
                    "Orthogonal: C^T·C[{i}][{j}] = {:.12}, erwartet {expected}",
                    ctc[i][j]
                );
            }
        }

        // X-Achse = erste Spalte der Rotationsmatrix = mat[0][0], mat[1][0], mat[2][0]
        // = [-1, 0, 0]
        assert!(
            (mat[0][0] - (-1.0)).abs() < 1e-10,
            "X-Achse[0] = {:.12}, erwartet -1.0",
            mat[0][0]
        );
        assert!(
            mat[1][0].abs() < 1e-10,
            "X-Achse[1] = {:.12}, erwartet 0.0",
            mat[1][0]
        );
        assert!(
            mat[2][0].abs() < 1e-10,
            "X-Achse[2] = {:.12}, erwartet 0.0",
            mat[2][0]
        );

        // Phase-Table Gruppe 5 muss deaktiviert sein
        assert_eq!(pt.get_phase(5), Some(0));
    }
}
