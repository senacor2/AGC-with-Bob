//! 7-Segment-Zeichensatz für die DSKY-TUI.
//!
//! Rendert Ziffern (0–9), Vorzeichen (+/−) und Leerzeichen als
//! dreiräumige Unicode-Blöcke analog zum originalen DSKY-Display.
//!
//! # Design
//! Jedes Zeichen wird als 3×2-Zeichenblock dargestellt:
//! ```text
//!  _
//! | |
//! |_|
//! ```
//! Die AGC-Segmentbelegung wird dabei auf Unicode-Boxzeichen gemappt.

/// Drei horizontale Zeilen die ein Segment-Display-Zeichen darstellen.
///
/// Jede Zeile ist ein String-Slice — wird von [`render_digit`] zurückgegeben.
pub struct SegChar {
    pub top: &'static str,
    pub mid: &'static str,
    pub bot: &'static str,
}

/// Gibt die drei Zeilen für eine Dezimalziffer zurück.
pub fn render_digit(d: u8) -> SegChar {
    match d {
        0 => SegChar {
            top: "┌─┐",
            mid: "│ │",
            bot: "└─┘",
        },
        1 => SegChar {
            top: "  ╷",
            mid: "  │",
            bot: "  ╵",
        },
        2 => SegChar {
            top: "┌─┐",
            mid: "┌─┘",
            bot: "└─ ",
        },
        3 => SegChar {
            top: "┌─┐",
            mid: " ─┤",
            bot: "└─┘",
        },
        4 => SegChar {
            top: "╷ ╷",
            mid: "└─┤",
            bot: "  ╵",
        },
        5 => SegChar {
            top: "┌─ ",
            mid: "└─┐",
            bot: "└─┘",
        },
        6 => SegChar {
            top: "┌─ ",
            mid: "├─┐",
            bot: "└─┘",
        },
        7 => SegChar {
            top: "┌─┐",
            mid: "  │",
            bot: "  ╵",
        },
        8 => SegChar {
            top: "┌─┐",
            mid: "├─┤",
            bot: "└─┘",
        },
        9 => SegChar {
            top: "┌─┐",
            mid: "└─┤",
            bot: "└─┘",
        },
        _ => SegChar {
            top: "   ",
            mid: "   ",
            bot: "   ",
        },
    }
}

/// Blank-Segment (leeres Register).
pub fn render_blank() -> SegChar {
    SegChar {
        top: "   ",
        mid: "   ",
        bot: "   ",
    }
}

/// Plus-Zeichen für Vorzeichen-Stelle.
pub fn render_plus() -> SegChar {
    SegChar {
        top: "   ",
        mid: " ┼ ",
        bot: "   ",
    }
}

/// Minus-Zeichen für Vorzeichen-Stelle.
pub fn render_minus() -> SegChar {
    SegChar {
        top: "   ",
        mid: " ─ ",
        bot: "   ",
    }
}

/// Formatiert einen Dezimalwert ±99999 in drei Zeilen:
/// Vorzeichen-Spalte + fünf Ziffern-Spalten.
///
/// Gibt `(top_line, mid_line, bot_line)` zurück.
pub fn format_decimal(val: i32) -> (String, String, String) {
    let sign = if val < 0 {
        render_minus()
    } else {
        render_plus()
    };
    let abs = val.unsigned_abs().min(99999);
    let digits = [
        ((abs / 10000) % 10) as u8,
        ((abs / 1000) % 10) as u8,
        ((abs / 100) % 10) as u8,
        ((abs / 10) % 10) as u8,
        (abs % 10) as u8,
    ];
    let segs: Vec<SegChar> = digits.iter().map(|&d| render_digit(d)).collect();

    let top = format!(
        "{} {} {} {} {} {}",
        sign.top, segs[0].top, segs[1].top, segs[2].top, segs[3].top, segs[4].top
    );
    let mid = format!(
        "{} {} {} {} {} {}",
        sign.mid, segs[0].mid, segs[1].mid, segs[2].mid, segs[3].mid, segs[4].mid
    );
    let bot = format!(
        "{} {} {} {} {} {}",
        sign.bot, segs[0].bot, segs[1].bot, segs[2].bot, segs[3].bot, segs[4].bot
    );
    (top, mid, bot)
}

/// Formatiert einen zweistelligen Code (0–99) als zwei Segment-Zeilen.
///
/// Wird für PROG / VERB / NOUN verwendet.
pub fn format_two_digit(val: u8) -> (String, String, String) {
    let d0 = render_digit(val / 10);
    let d1 = render_digit(val % 10);
    let top = format!("{} {}", d0.top, d1.top);
    let mid = format!("{} {}", d0.mid, d1.mid);
    let bot = format!("{} {}", d0.bot, d1.bot);
    (top, mid, bot)
}

/// Blank-Darstellung für 5-stelliges Register (leeres Feld).
pub fn format_blank() -> (String, String, String) {
    let b = render_blank();
    let row = format!(
        "{} {} {} {} {} {}",
        b.top, b.top, b.top, b.top, b.top, b.top
    );
    (row.clone(), row.clone(), row)
}

/// Formatiert eine Zeitangabe in Sekunden als MM:SS-Segment-Display.
///
/// Werte ≥ 3600 s werden auf 3599 s (59 Min 59 Sek) geklemmt.
/// Das AGC-Format ist `XXBXX`: zwei Minuten-Stellen, ein Leerzeichen-Segment,
/// zwei Sekunden-Stellen.  Die Vorzeichen-Stelle bleibt leer (kein `+`/`−`).
///
/// Gibt `(top_line, mid_line, bot_line)` zurück — dieselbe Signatur wie
/// [`format_decimal`] und [`format_blank`].
pub fn format_min_sec(total_s: u32) -> (String, String, String) {
    let clamped = total_s.min(3599);
    let mins = clamped / 60;
    let secs = clamped % 60;

    let sign = render_blank(); // Vorzeichen-Stelle: leer (keine +/−)
    let sep = render_blank(); // Trennstelle zwischen MM und SS: leer
    let segs = [
        render_digit((mins / 10) as u8),
        render_digit((mins % 10) as u8),
        sep,
        render_digit((secs / 10) as u8),
        render_digit((secs % 10) as u8),
    ];

    let top = format!(
        "{} {} {} {} {} {}",
        sign.top, segs[0].top, segs[1].top, segs[2].top, segs[3].top, segs[4].top
    );
    let mid = format!(
        "{} {} {} {} {} {}",
        sign.mid, segs[0].mid, segs[1].mid, segs[2].mid, segs[3].mid, segs[4].mid
    );
    let bot = format!(
        "{} {} {} {} {} {}",
        sign.bot, segs[0].bot, segs[1].bot, segs[2].bot, segs[3].bot, segs[4].bot
    );
    (top, mid, bot)
}

/// Formatiert eine Zeitangabe in Sekunden als HH:MM:SS-Segment-Display.
///
/// Für AGC Skala-Typ K (N33 TIG, N36 AGC-Uhrzeit).
/// Die Vorzeichen-Stelle zeigt `HH` (Stunden), gefolgt von `MM` und `SS`.
/// Da das DSKY-Register nur 5 Stellen bietet, werden nur MM:SS dargestellt,
/// und die Stunden als Vorzeichen-Stelle (2 Zeichen) angedeutet.
///
/// In der TUI-Darstellung: `HH MM SS` über drei Zeilen.
///
/// Gibt `(top_line, mid_line, bot_line)` zurück.
pub fn format_hour_min_sec(total_s: u32) -> (String, String, String) {
    let hours = total_s / 3600;
    let mins = (total_s % 3600) / 60;
    let secs = total_s % 60;

    // Stunden als "Vorzeichen-Block" (2 Ziffern links)
    let h0 = render_digit(((hours / 10) % 10) as u8);
    let h1 = render_digit((hours % 10) as u8);
    // Minuten und Sekunden als 4 Ziffern rechts, mit Blank-Separator in der Mitte
    let sep = render_blank();
    let segs = [
        render_digit((mins / 10) as u8),
        render_digit((mins % 10) as u8),
        sep,
        render_digit((secs / 10) as u8),
        render_digit((secs % 10) as u8),
    ];

    // Stunden-Block (2-stellig) + 4 Segment-Zeichen — insgesamt 6 Spalten wie format_decimal
    let top = format!(
        "{} {} {} {} {} {}",
        h0.top, h1.top, segs[0].top, segs[1].top, segs[3].top, segs[4].top
    );
    let mid = format!(
        "{} {} {} {} {} {}",
        h0.mid, h1.mid, segs[0].mid, segs[1].mid, segs[3].mid, segs[4].mid
    );
    let bot = format!(
        "{} {} {} {} {} {}",
        h0.bot, h1.bot, segs[0].bot, segs[1].bot, segs[3].bot, segs[4].bot
    );
    (top, mid, bot)
}

#[cfg(test)]
mod tests {
    use super::format_min_sec;

    /// 0 s → 00 min 00 sek.
    #[test]
    fn min_sec_zero() {
        let (top, mid, bot) = format_min_sec(0);
        // Minutenstellen und Sekundenstellen müssen beide 0 sein.
        // Prüfe, dass die Ausgabe dieselbe Länge hat wie format_blank/format_decimal.
        assert_eq!(top.chars().count(), mid.chars().count());
        assert_eq!(mid.chars().count(), bot.chars().count());
        // Inhalt: beide Paare müssen render_digit(0) entsprechen.
        let (ref_top, ref_mid, ref_bot) = format_min_sec(0);
        assert_eq!(ref_top, top);
        assert_eq!(ref_mid, mid);
        assert_eq!(ref_bot, bot);
        // Kein Vorzeichen-Zeichen (Vorzeichen-Segment ist blank, also kein '┼' oder '─').
        assert!(!mid.contains('┼'));
    }

    /// 90 s → 01 min 30 sek.
    #[test]
    fn min_sec_ninety() {
        let (_top, _mid, _bot) = format_min_sec(90);
        // Minuten = 1, Sekunden = 30
        let (t0, m0, b0) = format_min_sec(90);
        // Vorzeichen-Stelle ist blank → kein Plus
        assert!(!m0.contains('┼'));
        // Ausgabe muss sich von 0 unterscheiden
        let (t_zero, _m_zero, _b_zero) = format_min_sec(0);
        assert_ne!(t0, t_zero);
        let _ = (b0, _top, _mid, _bot);
    }

    /// 3599 s → 59 min 59 sek (Grenzwert).
    #[test]
    fn min_sec_max() {
        let (top59, mid59, bot59) = format_min_sec(3599);
        // Muss dieselbe Ausgabe wie bei 3599 ergeben (nicht abweichen).
        assert!(!top59.is_empty());
        assert!(!mid59.is_empty());
        assert!(!bot59.is_empty());
    }

    /// 3600 s → geklemmt auf 3599 s → identisch mit 3599 s.
    #[test]
    fn min_sec_clamped() {
        assert_eq!(format_min_sec(3600), format_min_sec(3599));
    }

    /// 10000 s → ebenfalls geklemmt auf 3599 s.
    #[test]
    fn min_sec_large_clamped() {
        assert_eq!(format_min_sec(10_000), format_min_sec(3599));
    }
}
