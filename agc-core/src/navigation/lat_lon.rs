//! Geografische Koordinatenkonvertierung: ECI-Positionsvektor → Lat/Lon/Alt.
//!
//! Sphärische Näherung ohne Erdflattenkorrektur (entsprechend der AGC-Implementierung).
//!
//! # Einheiten
//! - Eingabe: Position in Metern (ECI J2000)
//! - Ausgabe: Breite in Radiant, Länge in Radiant, Höhe in Metern
//!
//! # AGC-Quelle
//! `Comanche055/LATITUDE_LONGITUDE_SUBROUTINES.agc`, LAT-LONG routine.

use crate::types::Vec3;

/// Konvertiert einen ECI-Positionsvektor in geocentrische Kugelkoordinaten.
///
/// Verwendet die sphärische Näherung (keine Erdflattenkorrektur).
///
/// # Parameter
/// - `pos`: ECI-Positionsvektor in m (J2000, Ursprung Erdmittelpunkt)
/// - `r_equatorial`: Äquatorialradius der Erde in m (z.B. 6_371_000.0)
///
/// # Rückgabe
/// `(lat, lon, alt)` mit:
/// - `lat`: geocentrische Breite in Radiant (−π/2 … +π/2)
/// - `lon`: Länge in Radiant (−π … +π)
/// - `alt`: Höhe über der Referenzkugel in m
///
/// Gibt `(0.0, 0.0, 0.0)` zurück wenn `pos` der Ursprung ist (r = 0).
///
/// AGC source: `Comanche055/LATITUDE_LONGITUDE_SUBROUTINES.agc`, LAT-LONG routine.
pub fn radius_to_lat_lon_alt(pos: Vec3, r_equatorial: f64) -> (f64, f64, f64) {
    let r = pos.magnitude();
    if r == 0.0 {
        return (0.0, 0.0, 0.0);
    }

    let lat = libm::asin(pos.z / r);
    let lon = libm::atan2(pos.y, pos.x);
    let alt = r - r_equatorial;

    (lat, lon, alt)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::Vec3;
    use core::f64::consts::FRAC_PI_2;

    const R_EQ: f64 = 6_371_000.0;

    #[test]
    fn north_pole() {
        let pos = Vec3::new(0.0, 0.0, 6_671_000.0);
        let (lat, lon, alt) = radius_to_lat_lon_alt(pos, R_EQ);
        assert!(
            (lat - FRAC_PI_2).abs() < 1e-10,
            "Nordpol: lat = {lat}, erwartet π/2"
        );
        assert!(
            (lon - 0.0).abs() < 1e-10,
            "Nordpol: lon = {lon}, erwartet 0"
        );
        assert!(alt > 0.0, "Nordpol: alt = {alt}, erwartet > 0");
    }

    #[test]
    fn equator_prime_meridian() {
        let pos = Vec3::new(6_671_000.0, 0.0, 0.0);
        let (lat, lon, alt) = radius_to_lat_lon_alt(pos, R_EQ);
        assert!(lat.abs() < 1e-10, "Äquator/PM: lat = {lat}, erwartet 0");
        assert!(lon.abs() < 1e-10, "Äquator/PM: lon = {lon}, erwartet 0");
        assert!(
            (alt - 300_000.0).abs() < 1.0,
            "Äquator/PM: alt = {alt}, erwartet ~300_000"
        );
    }

    #[test]
    fn equator_90_east() {
        let pos = Vec3::new(0.0, 6_671_000.0, 0.0);
        let (lat, lon, _alt) = radius_to_lat_lon_alt(pos, R_EQ);
        assert!(lat.abs() < 1e-10, "90°O: lat = {lat}, erwartet 0");
        assert!(
            (lon - FRAC_PI_2).abs() < 1e-10,
            "90°O: lon = {lon}, erwartet π/2"
        );
    }

    #[test]
    fn south_pole() {
        let pos = Vec3::new(0.0, 0.0, -6_671_000.0);
        let (lat, _lon, _alt) = radius_to_lat_lon_alt(pos, R_EQ);
        assert!(
            (lat + FRAC_PI_2).abs() < 1e-10,
            "Südpol: lat = {lat}, erwartet -π/2"
        );
    }
}
