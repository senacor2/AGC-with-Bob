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
            top: "  │",
            mid: "  │",
            bot: "  │",
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
            top: "│ │",
            mid: "└─┤",
            bot: "  │",
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
            bot: "  │",
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
