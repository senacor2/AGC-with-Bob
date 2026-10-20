//! Kalman-Filter Messungs-Inkorporation für Comanche055.
//!
//! Implementiert den skalaren Kalman-Update-Schritt für einen 3D-Positionsfehler-Vektor.
//! Die vollständige 6D/9D-W-Matrix (Kovarianz-Matrix) wird in Task 8 ergänzt;
//! hier wird vereinfacht P = I (Einheitsmatrix) angenommen.
//!
//! # Einheiten und Skalierung
//! - `obs`: Messung in Metern (m)
//! - `h_row`: Mess-Jacobi-Zeile, dimensionslos
//! - `w`: Messrauschen-Gewicht in 1/m² (inverse Varianz)
//! - `delta_x`: aktueller Zustandsfehler in Metern (m)
//! - Rückgabe: aktualisierter Zustandsfehler in Metern (m)
//!
//! # AGC-Quelle
//! `Comanche055/MEASUREMENT_INCORPORATION.agc`, INCORP1-Routine.

use crate::types::Vec3;

/// Führt einen skalaren Kalman-Update-Schritt für einen 3D-Positionsfehler-Vektor durch.
///
/// # Algorithmus
/// ```text
/// innovation  = obs - h_row · delta_x
/// denom       = (h_row · h_row) / w + 1.0   (P = I angenommen)
/// k           = (1/w) / denom                (skalarer Kalman-Gain)
/// delta_x_new = delta_x + k * innovation * h_row
/// ```
///
/// Die vereinfachte Form ergibt sich, weil P = I (Identitätsmatrix) als
/// A-priori-Kovarianz angenommen wird. Die vollständige W-Matrix folgt in Task 8.
///
/// # Einheiten
/// - `obs` : Messung [m]
/// - `h_row`: Mess-Jacobi-Zeile [dimensionslos]
/// - `w`   : Messrauschen-Gewicht [1/m²] — größeres `w` = genauere Messung
/// - `delta_x`: aktueller Zustandsfehler [m]
/// - Rückgabe: aktualisierter Zustandsfehler [m]
///
/// # AGC-Quelle
/// `Comanche055/MEASUREMENT_INCORPORATION.agc`, INCORP1-Routine.
pub fn incorporate_measurement(obs: f64, h_row: Vec3, w: f64, delta_x: Vec3) -> Vec3 {
    let innovation = obs - h_row.dot(delta_x);
    let h_sq = h_row.dot(h_row);
    // denom = h·h / w + 1  (aus P = I → S = H·P·Hᵀ + R = h·h + 1/w)
    let denom = h_sq / w + 1.0;
    // skalarer Kalman-Gain: k = P·Hᵀ / S = 1/w / (h·h/w + 1)
    let k = (1.0 / w) / denom;
    // Zustandsfehler-Update
    delta_x + h_row.scale(k * innovation)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::incorporate_measurement;
    use crate::types::Vec3;

    /// Fünf konsistente Messungen (wahrer Wert = 0, h=[1,0,0], w=0.01)
    /// sollten delta_x.x in Richtung 0 konvergieren lassen.
    #[test]
    fn convergence_test() {
        let h = Vec3::new(1.0, 0.0, 0.0);
        let w = 0.01_f64; // Messrauschen-Gewicht [1/m²]: kleine Varianz → genaue Messung
        let true_obs = 0.0_f64;
        let mut delta_x = Vec3::new(100.0, 0.0, 0.0); // Startwert weit vom wahren Wert

        for _ in 0..5 {
            delta_x = incorporate_measurement(true_obs, h, w, delta_x);
        }

        // Nach 5 Iterationen sollte der Fehler deutlich kleiner als der Startwert sein.
        assert!(
            delta_x.x.abs() < 1.0,
            "Erwarte Konvergenz unter 1 m, bekam {}",
            delta_x.x
        );
        // y- und z-Komponenten bleiben unberührt (h=[1,0,0])
        assert_eq!(delta_x.y, 0.0);
        assert_eq!(delta_x.z, 0.0);
    }

    /// Sehr großes w (w → ∞, d. h. sehr präzise Messung, Rauschvarianz → 0):
    /// denom = h·h/w + 1 → 1,  k = (1/w)/1 → 0  → Update fast null (Messung wird ignoriert).
    ///
    /// Sehr kleines w (w → 0, starkes Rauschen, 1/w → ∞):
    /// denom = h·h/w + 1 → ∞,  k = (1/w)/denom = 1/(h·h + w) → 1/(h·h) = 1
    ///   → Update geht vollständig auf die Messung (Zustand wird komplett durch Messung ersetzt).
    ///
    /// Dieser Test prüft den Grenzfall w→∞ (großes Messrauschen-Gewicht = schlechte Messung):
    /// Bei w=1e10 (quasi Null-Varianz) wird der Gain k ≈ 1/w winzig → Update << obs.
    #[test]
    fn large_weight_gives_tiny_update() {
        let h = Vec3::new(1.0, 0.0, 0.0);
        // w sehr groß ≡ sehr präzise Messung (1/w → 0):
        // k = (1/w) / (h·h/w + 1) → 0/1 = 0
        let w = 1e10_f64;
        let obs = 100.0_f64;
        let delta_x = Vec3::ZERO;

        let updated = incorporate_measurement(obs, h, w, delta_x);

        // Bei sehr großem w ist der Kalman-Gain k ≈ 1/w winzig → Update fast null
        assert!(
            updated.x.abs() < 1e-7,
            "Erwarte minimales Update bei großem w, bekam {}",
            updated.x
        );
    }

    /// Startet bei delta_x = 0, Messung obs = 10 m in x-Richtung.
    /// Das Update muss in positive x-Richtung gehen.
    #[test]
    fn initial_delta_x_zero_update_correct_direction() {
        let h = Vec3::new(1.0, 0.0, 0.0);
        let w = 1.0_f64; // neutrales Gewicht
        let obs = 10.0_f64;
        let delta_x = Vec3::ZERO;

        let updated = incorporate_measurement(obs, h, w, delta_x);

        // Update muss positiv (in Richtung Messung) sein
        assert!(
            updated.x > 0.0,
            "Update muss in positive x-Richtung gehen, bekam {}",
            updated.x
        );
        assert_eq!(updated.y, 0.0);
        assert_eq!(updated.z, 0.0);
    }
}
