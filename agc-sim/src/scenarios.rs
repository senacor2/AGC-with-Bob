//! Scenario Loader — liest Szenario-Dateien (Key=Value) und befüllt den SimState.
//!
//! Szenario-Dateien beschreiben einen physikalischen Startzustand (Position,
//! Geschwindigkeit, Zeit) und optionale Guidance-Felder. Kommentare beginnen
//! mit `#`. Leerzeilen und unbekannte Felder werden ignoriert.
//!
//! # Dateiformat
//! ```text
//! # AGC Scenario File
//! pos_x_m  = 6771000.0
//! pos_y_m  = 0.0
//! pos_z_m  = 0.0
//! vel_x_ms = 0.0
//! vel_y_ms = 7668.3
//! vel_z_ms = 0.0
//! time_s   = 0.0
//! ```
//!
//! Pflichtfelder: `pos_x_m`, `pos_y_m`, `pos_z_m`, `vel_x_ms`, `vel_y_ms`,
//! `vel_z_ms`, `time_s`.
//!
//! Optionale Felder (werden bei Unbekanntheit ignoriert):
//! `thetad_0_deg`, `thetad_1_deg`, `thetad_2_deg`, `mass_kg`.

use agc_physics::math::Vec3;
use agc_physics::trajectory::State;

// ---------------------------------------------------------------------------
// Datentyp
// ---------------------------------------------------------------------------

/// Gelesene Szenario-Daten.
///
/// Alle Werte in SI-Einheiten (m, m/s, s).
#[derive(Debug, Clone, Copy)]
pub struct ScenarioData {
    /// Positionsvektor ECI (m).
    pub position: Vec3,
    /// Geschwindigkeitsvektor ECI (m/s).
    pub velocity: Vec3,
    /// Mission Elapsed Time (s).
    pub time_s: f64,
    /// Optionale Gimbal-Zielwinkel (Grad) für P51/P52 (`thetad_N_deg`).
    pub thetad_deg: Option<[f64; 3]>,
    /// Optionale Fahrzeugmasse [kg] für P40 (`mass_kg`).
    /// Falls nicht angegeben, verwendet der Simulator den Standardwert aus `GuidanceContext`.
    pub mass_kg: Option<f64>,
}

impl ScenarioData {
    /// Konvertiert `ScenarioData` in einen physikalischen [`State`].
    pub fn to_state(self) -> State {
        State::new(self.position, self.velocity, self.time_s)
    }

    /// Optionale Fahrzeugmasse [kg], falls im Szenario angegeben.
    pub fn vehicle_mass_kg(&self) -> Option<f64> {
        self.mass_kg
    }
}

// ---------------------------------------------------------------------------
// Parser
// ---------------------------------------------------------------------------

/// Liest eine Szenario-Datei (Key=Value-Format) und gibt [`ScenarioData`] zurück.
///
/// Fehler: fehlende Pflichtfelder, ungültige Zahlenwerte oder nicht lesbare Datei.
///
/// # Parameter
/// - `path`: Dateipfad relativ zum aktuellen Arbeitsverzeichnis
///
/// # Fehler
/// Gibt `Err(String)` zurück wenn die Datei nicht gelesen werden kann,
/// ein Pflichtfeld fehlt oder ein Wert nicht als `f64` parsbar ist.
pub fn load_scenario_file(path: &str) -> Result<ScenarioData, String> {
    let content = std::fs::read_to_string(path)
        .map_err(|e| format!("Szenario-Datei '{}' nicht lesbar: {}", path, e))?;
    parse_scenario_str(&content)
}

/// Parst Szenario-Inhalt aus einem String.
///
/// Wird intern von `load_scenario_file` verwendet und in Unit-Tests direkt
/// aufgerufen.
pub fn parse_scenario_str(content: &str) -> Result<ScenarioData, String> {
    let mut pos_x: Option<f64> = None;
    let mut pos_y: Option<f64> = None;
    let mut pos_z: Option<f64> = None;
    let mut vel_x: Option<f64> = None;
    let mut vel_y: Option<f64> = None;
    let mut vel_z: Option<f64> = None;
    let mut time_s: Option<f64> = None;
    let mut thetad_0: Option<f64> = None;
    let mut thetad_1: Option<f64> = None;
    let mut thetad_2: Option<f64> = None;
    let mut mass_kg: Option<f64> = None;

    for line in content.lines() {
        // Kommentare und Leerzeilen überspringen
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }

        // Key=Value trennen
        let (key, val) = split_kv(line)?;

        let v: f64 = val
            .parse()
            .map_err(|_| format!("Ungültiger Wert für '{}': '{}'", key, val))?;

        match key {
            "pos_x_m" => pos_x = Some(v),
            "pos_y_m" => pos_y = Some(v),
            "pos_z_m" => pos_z = Some(v),
            "vel_x_ms" => vel_x = Some(v),
            "vel_y_ms" => vel_y = Some(v),
            "vel_z_ms" => vel_z = Some(v),
            "time_s" => time_s = Some(v),
            "thetad_0_deg" => thetad_0 = Some(v),
            "thetad_1_deg" => thetad_1 = Some(v),
            "thetad_2_deg" => thetad_2 = Some(v),
            "mass_kg" => mass_kg = Some(v),
            // Unbekannte Felder werden ignoriert
            _ => {}
        }
    }

    // Pflichtfelder prüfen
    let position = Vec3::new(
        pos_x.ok_or("Pflichtfeld 'pos_x_m' fehlt")?,
        pos_y.ok_or("Pflichtfeld 'pos_y_m' fehlt")?,
        pos_z.ok_or("Pflichtfeld 'pos_z_m' fehlt")?,
    );
    let velocity = Vec3::new(
        vel_x.ok_or("Pflichtfeld 'vel_x_ms' fehlt")?,
        vel_y.ok_or("Pflichtfeld 'vel_y_ms' fehlt")?,
        vel_z.ok_or("Pflichtfeld 'vel_z_ms' fehlt")?,
    );
    let time_s = time_s.ok_or("Pflichtfeld 'time_s' fehlt")?;

    // Optionale thetad-Felder: nur wenn alle drei angegeben
    let thetad_deg = match (thetad_0, thetad_1, thetad_2) {
        (Some(a), Some(b), Some(c)) => Some([a, b, c]),
        _ => None,
    };

    Ok(ScenarioData {
        position,
        velocity,
        time_s,
        thetad_deg,
        mass_kg,
    })
}

/// Trennt eine `key = value`-Zeile in `(key, value)`.
fn split_kv(line: &str) -> Result<(&str, &str), String> {
    let mut parts = line.splitn(2, '=');
    let key = parts
        .next()
        .ok_or_else(|| format!("Ungültige Zeile: '{}'", line))?
        .trim();
    let val = parts
        .next()
        .ok_or_else(|| format!("Kein '='-Zeichen in Zeile: '{}'", line))?
        .trim();
    Ok((key, val))
}

// ---------------------------------------------------------------------------
// Unit-Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const VALID_SCENARIO: &str = "
# AGC Scenario File
pos_x_m  = 6771000.0
pos_y_m  = 0.0
pos_z_m  = 0.0
vel_x_ms = 0.0
vel_y_ms = 7668.3
vel_z_ms = 0.0
time_s   = 0.0
";

    #[test]
    fn parse_valid_scenario() {
        let data = parse_scenario_str(VALID_SCENARIO).unwrap();
        assert!((data.position.x - 6_771_000.0).abs() < 1.0);
        assert!((data.velocity.y - 7_668.3).abs() < 0.1);
        assert_eq!(data.time_s, 0.0);
        assert!(data.thetad_deg.is_none());
    }

    #[test]
    fn missing_field_returns_error() {
        // pos_z_m fehlt
        let bad = "pos_x_m=1.0\npos_y_m=2.0\nvel_x_ms=0.0\nvel_y_ms=0.0\nvel_z_ms=0.0\ntime_s=0.0";
        let result = parse_scenario_str(bad);
        assert!(result.is_err(), "Fehlende Felder sollen Fehler liefern");
        assert!(result.unwrap_err().contains("pos_z_m"));
    }

    #[test]
    fn invalid_value_returns_error() {
        let bad = "pos_x_m=NOTANUMBER\npos_y_m=0.0\npos_z_m=0.0\nvel_x_ms=0.0\nvel_y_ms=0.0\nvel_z_ms=0.0\ntime_s=0.0";
        let result = parse_scenario_str(bad);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("pos_x_m"));
    }

    #[test]
    fn comments_and_blank_lines_ignored() {
        let content = "# Kommentar\n\npos_x_m=1.0\npos_y_m=2.0\npos_z_m=3.0\n# noch ein Kommentar\nvel_x_ms=4.0\nvel_y_ms=5.0\nvel_z_ms=6.0\ntime_s=10.0\n";
        let data = parse_scenario_str(content).unwrap();
        assert_eq!(data.time_s, 10.0);
    }

    #[test]
    fn thetad_optional_fields_parsed() {
        let content = "pos_x_m=1.0\npos_y_m=0.0\npos_z_m=0.0\nvel_x_ms=0.0\nvel_y_ms=0.0\nvel_z_ms=0.0\ntime_s=0.0\nthetad_0_deg=10.0\nthetad_1_deg=20.0\nthetad_2_deg=30.0\n";
        let data = parse_scenario_str(content).unwrap();
        let td = data.thetad_deg.unwrap();
        assert!((td[0] - 10.0).abs() < 1e-9);
        assert!((td[2] - 30.0).abs() < 1e-9);
    }

    #[test]
    fn unknown_fields_ignored() {
        let content = "pos_x_m=1.0\npos_y_m=0.0\npos_z_m=0.0\nvel_x_ms=0.0\nvel_y_ms=0.0\nvel_z_ms=0.0\ntime_s=0.0\nunbekannt=99.9\n";
        let result = parse_scenario_str(content);
        assert!(result.is_ok());
    }
}
