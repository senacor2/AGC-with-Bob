//! AGC-Sternkatalog: 37 Navigationssterne als Unit-Vektoren.
//!
//! Die Vektoren stammen direkt aus der AGC-Quellcode-Datei und sind in
//! J2000-ECI-Koordinaten angegeben (B-1-Skalierung im Original: Werte ∈ (−1, +1)).
//!
//! # Einheiten
//! Einheitsvektoren (dimensionslos, |v| ≈ 1.0).
//!
//! # AGC-Quelle
//! `Comanche055/STAR_TABLES.agc`, STARTAB-Segment.

use crate::types::Vec3;

/// Die 37 AGC-Navigationssterne als J2000-ECI-Einheitsvektoren.
///
/// Index 0 entspricht Stern 1 (AGC-Nummerierung 1-basiert).
/// Verwende [`get_star`] für 1-basierten Zugriff.
///
/// Werte aus `Comanche055/STAR_TABLES.agc` (2DEC-Format, B-1-Skalierung).
/// Die in der AGC-Quelle als `+.XXXXXXXXXX B-1` notierten Dezimalwerte
/// sind direkt die f64-Koordinaten der Einheitsvektoren.
///
/// AGC source: `Comanche055/STAR_TABLES.agc`, STARTAB segment.
pub static STARS: [Vec3; 37] = [
    // STAR 1
    Vec3::new(0.874_865_891_8, 0.026_087_917_4, 0.483_662_167_0),
    // STAR 2
    Vec3::new(0.934_264_040_0, 0.173_507_314_2, -0.311_521_933_9),
    // STAR 3
    Vec3::new(0.477_563_945_0, 0.116_600_434_0, 0.870_825_480_3),
    // STAR 4
    Vec3::new(0.491_767_827_6, 0.220_488_712_5, -0.842_347_393_5),
    // STAR 5  (Polarstern-Nähe)
    Vec3::new(0.013_096_884_0, 0.007_806_279_5, 0.999_883_760_0),
    // STAR 6
    Vec3::new(0.545_010_740_4, 0.531_495_546_6, -0.648_441_035_6),
    // STAR 7
    Vec3::new(0.703_223_546_9, 0.707_584_604_7, 0.069_286_868_5),
    // STAR 8
    Vec3::new(0.410_563_602_0, 0.498_811_000_1, 0.763_298_837_1),
    // STAR 9
    Vec3::new(0.350_731_503_8, 0.892_633_330_7, 0.283_183_949_2),
    // STAR 10
    Vec3::new(0.201_139_958_9, 0.969_033_794_1, -0.143_234_851_2),
    // STAR 11
    Vec3::new(0.137_172_557_5, 0.681_372_106_1, 0.718_968_526_7),
    // STAR 12
    Vec3::new(-0.061_493_723_0, 0.603_156_328_6, -0.795_248_995_7),
    // STAR 13
    Vec3::new(-0.182_075_178_3, 0.940_489_986_9, -0.286_927_192_6),
    // STAR 14
    Vec3::new(-0.411_858_952_4, 0.906_548_536_0, 0.092_422_697_5),
    // STAR 15
    Vec3::new(-0.361_250_853_2, 0.574_727_084_0, -0.734_293_265_5),
    // STAR 16
    Vec3::new(-0.465_794_794_1, 0.477_478_503_3, 0.745_016_435_1),
    // STAR 17
    Vec3::new(-0.774_259_135_6, 0.615_250_419_7, -0.148_289_283_9),
    // STAR 18
    Vec3::new(-0.860_820_521_9, 0.463_621_398_9, 0.209_864_783_5),
    // STAR 19
    Vec3::new(-0.965_660_548_4, 0.052_593_315_6, 0.254_428_080_9),
    // STAR 20
    Vec3::new(-0.952_521_169_5, -0.059_343_479_6, -0.298_633_174_6),
    // STAR 21
    Vec3::new(-0.452_344_020_3, -0.049_371_014_0, -0.890_475_934_6),
    // STAR 22
    Vec3::new(-0.917_009_766_2, -0.350_214_662_8, -0.190_899_917_6),
    // STAR 23
    Vec3::new(-0.581_203_537_6, -0.290_917_129_4, 0.759_980_046_8),
    // STAR 24
    Vec3::new(-0.689_839_323_3, -0.418_233_064_0, -0.590_933_847_4),
    // STAR 25
    Vec3::new(-0.786_176_393_6, -0.521_799_630_5, 0.331_137_167_5),
    // STAR 26
    Vec3::new(-0.532_687_693_0, -0.716_064_455_4, 0.451_104_774_2),
    // STAR 27
    Vec3::new(-0.351_649_960_9, -0.824_075_270_3, -0.444_119_639_0),
    // STAR 28
    Vec3::new(-0.114_623_785_8, -0.339_969_255_7, -0.933_425_033_3),
    // STAR 29
    Vec3::new(-0.112_430_477_3, -0.969_493_420_0, 0.217_811_607_2),
    // STAR 30
    Vec3::new(0.121_729_369_2, -0.770_273_284_7, 0.625_988_041_0),
    // STAR 31
    Vec3::new(0.206_952_578_9, -0.871_988_574_8, -0.443_628_848_6),
    // STAR 32
    Vec3::new(0.453_719_690_8, -0.877_950_880_1, 0.152_776_615_3),
    // STAR 33
    Vec3::new(0.552_018_446_4, -0.793_318_740_0, -0.256_750_874_5),
    // STAR 34
    Vec3::new(0.320_181_737_8, -0.443_602_194_6, -0.837_078_698_6),
    // STAR 35
    Vec3::new(0.454_108_627_0, -0.539_236_819_7, 0.709_231_278_9),
    // STAR 36
    Vec3::new(0.813_983_263_1, -0.555_724_318_9, 0.169_120_455_7),
    // STAR 37
    Vec3::new(0.834_297_140_8, -0.239_248_151_5, -0.496_697_697_5),
];

/// Gibt den Einheitsvektor des AGC-Sterns mit 1-basiertem Index zurück.
///
/// # Parameter
/// - `index`: Sternindex 1–37 (AGC-Konvention)
///
/// # Rückgabe
/// `Some(Vec3)` für gültige Indizes (1–37), `None` für `0` oder `> 37`.
///
/// AGC source: `Comanche055/STAR_TABLES.agc`, STARTAB segment.
pub fn get_star(index: usize) -> Option<Vec3> {
    if index == 0 || index > 37 {
        return None;
    }
    Some(STARS[index - 1])
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_star_vectors_unit_length() {
        for (i, star) in STARS.iter().enumerate() {
            let mag = star.magnitude();
            assert!(
                (mag - 1.0).abs() < 0.001,
                "Stern {}: |v| = {mag:.6}, erwartet ≈ 1.0",
                i + 1
            );
        }
    }

    #[test]
    fn get_star_valid_index() {
        let star = get_star(1);
        assert!(star.is_some(), "get_star(1) soll Some zurückgeben");
        let v = star.unwrap();
        let mag = v.magnitude();
        assert!(
            (mag - 1.0).abs() < 0.001,
            "Stern 1: |v| = {mag:.6}, erwartet ≈ 1.0"
        );
    }

    #[test]
    fn get_star_invalid_index() {
        assert!(get_star(0).is_none(), "get_star(0) soll None zurückgeben");
        assert!(get_star(38).is_none(), "get_star(38) soll None zurückgeben");
    }
}
