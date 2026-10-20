//! DSKY-Panel-Widget für die ratatui-TUI.
//!
//! Rendert ein optisch originalgetreues DSKY-Display mit:
//! - Statuslampen-Bereich (links oben): UPLINK ACTY, TEMP, NO ATT, GIMBAL LOCK,
//!   STBY, PROG, KEY REL, RESTART, OPR ERR, TRACKER
//! - Display-Bereich (rechts oben): PROG, VERB, NOUN, COMP ACTY, R1/R2/R3
//! - Tastatur-Bereich (unten): VERB, NOUN, +, −, 0–9, CLR, PRO, KEY REL,
//!   ENTR, RSET
//!
//! # Farbpalette
//! Orientiert sich am originalen DSKY:
//! - Gehäuse:        dunkelgrau (`DarkGray`)
//! - Display-Hintergrund: schwarz
//! - Leuchtende Segmente: grün (`Green`)
//! - Aktive Lampen:  grün/gelb je nach Typ
//! - Tasten:         schwarzer Hintergrund, weiße Beschriftung
//! - Warnlampen EIN: helles Grün / Gelb
//! - Warnlampen AUS: dunkelgrün (nicht leuchtend)

use ratatui::{
    buffer::Buffer,
    layout::Rect,
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::Widget,
};

use agc_core::display::verb_noun::{DisplayRegister, DisplayState};

use super::segment7::{format_blank, format_decimal, format_two_digit};

// ---------------------------------------------------------------------------
// Farb-Konstanten (DSKY-Farbpalette)
// ---------------------------------------------------------------------------

/// Hintergrundfarbe des DSKY-Displays (schwarz wie echtes DSKY).
const COL_DISPLAY_BG: Color = Color::Black;
/// Aktive Segment-Farbe (helles Grün).
const COL_SEG_ON: Color = Color::Green;
/// Inaktive Segment-Farbe (dunkelgrün, „nicht leuchtend").
const COL_SEG_OFF: Color = Color::DarkGray;
/// Warnlampe aktiv (helles Gelb/Grün).
const COL_LAMP_ON: Color = Color::LightGreen;
/// Warnlampe inaktiv.
const COL_LAMP_OFF: Color = Color::DarkGray;
/// Tastenfarbe: Hintergrund.
const COL_BTN_BG: Color = Color::Black;
/// Tastenfarbe: Beschriftung.
const COL_BTN_FG: Color = Color::White;
/// COMP ACTY Lampe (gelb, wie am echten DSKY).
const COL_COMP_ACTY: Color = Color::Yellow;
/// Gehäuse-Rahmen-Farbe.
const COL_FRAME: Color = Color::DarkGray;

// ---------------------------------------------------------------------------
// DskyWidget
// ---------------------------------------------------------------------------

/// Ratatui-Widget das das gesamte DSKY-Panel rendert.
///
/// # Usage
/// ```ignore
/// let dsky = DskyWidget::new(&display_state, comp_acty);
/// frame.render_widget(dsky, area);
/// ```
pub struct DskyWidget<'a> {
    state: &'a DisplayState,
    /// `true` wenn der AGC gerade rechnet (COMP ACTY Lampe).
    comp_acty: bool,
}

impl<'a> DskyWidget<'a> {
    /// Erstellt ein neues DSKY-Widget.
    pub fn new(state: &'a DisplayState, comp_acty: bool) -> Self {
        Self { state, comp_acty }
    }
}

impl Widget for DskyWidget<'_> {
    fn render(self, area: Rect, buf: &mut Buffer) {
        render_dsky(self.state, self.comp_acty, area, buf);
    }
}

// ---------------------------------------------------------------------------
// Rendering-Kernlogik
// ---------------------------------------------------------------------------

fn render_dsky(state: &DisplayState, comp_acty: bool, area: Rect, buf: &mut Buffer) {
    // Mindestgröße: 62×34 — Oberteil 21 Zeilen + Trennlinie + Tastatur 12 Zeilen
    if area.width < 62 || area.height < 34 {
        let msg = "[ Terminal zu klein — mind. 62×34 ]";
        buf.set_string(area.x, area.y, msg, Style::default().fg(Color::Red));
        return;
    }

    let x = area.x;
    let y = area.y;

    // --- Obere Hälfte: Statuslampen (links) + Display (rechts) ---
    render_status_lamps(state, comp_acty, x, y, buf);
    render_display_panel(state, x + 28, y, buf);

    // --- Trennlinie: nach 21 Zeilen (R3 endet bei y+20) ---
    let sep_y = y + 21;
    for col in x..x + area.width.min(62) {
        buf.set_string(col, sep_y, "─", Style::default().fg(COL_FRAME));
    }

    // --- Untere Hälfte: Tastatur ---
    render_keyboard(x, sep_y + 1, buf);
}

// ---------------------------------------------------------------------------
// Statuslampen-Panel (links oben, 2×5 Lampen)
// ---------------------------------------------------------------------------

fn lamp_style(on: bool) -> Style {
    if on {
        Style::default()
            .fg(COL_LAMP_ON)
            .add_modifier(Modifier::BOLD)
    } else {
        Style::default().fg(COL_LAMP_OFF)
    }
}

/// Rendert eine einzelne Lampe als beschriftetes Rechteck.
fn render_lamp(label_top: &str, label_bot: &str, on: bool, x: u16, y: u16, buf: &mut Buffer) {
    let sty = lamp_style(on);
    let border_sty = if on {
        Style::default().fg(COL_LAMP_ON)
    } else {
        Style::default().fg(COL_FRAME)
    };

    buf.set_string(x, y, "┌────────┐", border_sty);
    buf.set_string(x, y + 1, "│", border_sty);
    buf.set_string(x + 1, y + 1, format!("{:^8}", label_top), sty);
    buf.set_string(x + 9, y + 1, "│", border_sty);
    buf.set_string(x, y + 2, "│", border_sty);
    buf.set_string(x + 1, y + 2, format!("{:^8}", label_bot), sty);
    buf.set_string(x + 9, y + 2, "│", border_sty);
    buf.set_string(x, y + 3, "└────────┘", border_sty);
}

fn render_status_lamps(state: &DisplayState, comp_acty: bool, x: u16, y: u16, buf: &mut Buffer) {
    let ls = &state.lights;

    // Zwei Spalten à 5 Lampen (je 10 breit, 4 hoch, 1 Zeile Abstand)
    // Spalte 1 (links):  UPLINK ACTY, NO ATT, STBY, KEY REL, OPR ERR + leer
    // Spalte 2 (rechts): TEMP, GIMBAL LOCK, PROG, RESTART, TRACKER + leer

    let lamps_left: &[(&str, &str, bool)] = &[
        ("UPLINK", "ACTY", ls.uplink_acty),
        ("NO", "ATT", ls.no_att),
        ("STBY", "", false), // STBY nicht im agc-core StatusLights — immer aus
        ("KEY", "REL", ls.key_release),
        ("OPR", "ERR", ls.operator_error),
        ("", "", false), // leer
        ("", "", false), // leer
    ];

    let lamps_right: &[(&str, &str, bool)] = &[
        ("TEMP", "", false), // TEMP kein agc-core Feld
        ("GIMBAL", "LOCK", ls.gimbal_lock),
        ("PROG", "", false), // PROG-Lampe (kein eigenes Feld, immer aus)
        ("RE-", "START", false),
        ("TRACK-", "ER", ls.tracker),
        ("", "", false),
        ("", "", false),
    ];

    let col_l = x;
    let col_r = x + 12;

    for (i, (lt, lb, on)) in lamps_left.iter().enumerate() {
        render_lamp(lt, lb, *on, col_l, y + (i as u16) * 2, buf);
    }
    for (i, (lt, lb, on)) in lamps_right.iter().enumerate() {
        let _ = comp_acty; // nur für COMP ACTY unten genutzt
        render_lamp(lt, lb, *on, col_r, y + (i as u16) * 2, buf);
    }
}

// ---------------------------------------------------------------------------
// Display-Panel (rechts oben)
// ---------------------------------------------------------------------------

fn seg_style(on: bool) -> Style {
    if on {
        Style::default().fg(COL_SEG_ON).bg(COL_DISPLAY_BG)
    } else {
        Style::default().fg(COL_SEG_OFF).bg(COL_DISPLAY_BG)
    }
}

/// Rendert einen Label + zweistelligen Segment-Code (PROG / VERB / NOUN).
fn render_label_code(label: &str, val: u8, flash: bool, x: u16, y: u16, buf: &mut Buffer) {
    // Label
    buf.set_string(
        x,
        y,
        label,
        Style::default().fg(Color::White).bg(COL_DISPLAY_BG),
    );

    // Segmente
    let (top, mid, bot) = format_two_digit(val);
    // Wenn Blinken aktiv und VERB/NOUN: Segmente ausblenden (Blink-Halbperiode)
    let visible = !flash;
    let sty = seg_style(visible);
    buf.set_string(x, y + 1, &top, sty);
    buf.set_string(x, y + 2, &mid, sty);
    buf.set_string(x, y + 3, &bot, sty);
}

/// Rendert ein 5-stelliges Register (R1/R2/R3).
fn render_register(reg: &DisplayRegister, label: &str, x: u16, y: u16, buf: &mut Buffer) {
    buf.set_string(
        x,
        y,
        label,
        Style::default().fg(Color::DarkGray).bg(COL_DISPLAY_BG),
    );

    let (top, mid, bot) = match reg {
        DisplayRegister::Blank => format_blank(),
        DisplayRegister::Decimal(v) => format_decimal(*v),
        DisplayRegister::DecimalHighPrec(v, _frac) => format_decimal(*v),
        DisplayRegister::Octal(v) => {
            // Oktal als dezimale Darstellung der Rohbits (vereinfacht)
            format_decimal(*v as i32)
        }
    };

    let sty = seg_style(true);
    buf.set_string(x, y + 1, &top, sty);
    buf.set_string(x, y + 2, &mid, sty);
    buf.set_string(x, y + 3, &bot, sty);
}

fn render_display_panel(state: &DisplayState, x: u16, y: u16, buf: &mut Buffer) {
    // Hintergrund füllen
    for row in y..y + 16 {
        buf.set_string(x, row, " ".repeat(30), Style::default().bg(COL_DISPLAY_BG));
    }

    // COMP ACTY (oben links im Display-Panel)
    let comp_sty = Style::default().fg(COL_COMP_ACTY).bg(COL_DISPLAY_BG);
    buf.set_string(x, y, "COMP", comp_sty);
    buf.set_string(x, y + 1, "ACTY", comp_sty);

    // PROG (oben rechts)
    render_label_code("PROG", state.major_mode.val(), false, x + 16, y, buf);

    // VERB (links) / NOUN (rechts) mit möglichem Blink
    let vn_flash = state.lights.verb_noun_flash;
    render_label_code("VERB", state.verb.val(), vn_flash, x, y + 5, buf);
    render_label_code("NOUN", state.noun.val(), vn_flash, x + 16, y + 5, buf);

    // R1 / R2 / R3 — je 4 Zeilen (Label + 3 Segment-Zeilen), Abstand 1 Zeile
    render_register(&state.r1, "R1", x, y + 9, buf);
    render_register(&state.r2, "R2", x, y + 14, buf);
    render_register(&state.r3, "R3", x, y + 19, buf);
}

// ---------------------------------------------------------------------------
// Tastatur-Panel (unten)
// ---------------------------------------------------------------------------

/// Rendert eine einzelne Taste.
fn render_key(label_top: &str, label_bot: &str, x: u16, y: u16, buf: &mut Buffer) {
    let bg = Style::default().fg(COL_BTN_FG).bg(COL_BTN_BG);
    let border = Style::default().fg(Color::Gray).bg(COL_BTN_BG);

    buf.set_string(x, y, "┌──────┐", border);
    buf.set_string(x, y + 1, "│", border);
    buf.set_string(x + 1, y + 1, format!("{:^6}", label_top), bg);
    buf.set_string(x + 7, y + 1, "│", border);
    buf.set_string(x, y + 2, "│", border);
    buf.set_string(x + 1, y + 2, format!("{:^6}", label_bot), bg);
    buf.set_string(x + 7, y + 2, "│", border);
    buf.set_string(x, y + 3, "└──────┘", border);
}

fn render_keyboard(x: u16, y: u16, buf: &mut Buffer) {
    // Zeilenhöhe pro Taste: 4
    // Layout (angelehnt an DSKY):
    //
    // [VERB]  [+]  [7]  [8]  [9]  [CLR]   [ENTR]
    // [NOUN]  [-]  [4]  [5]  [6]  [PRO]
    //         [0]  [1]  [2]  [3]  [KEY REL][RSET]

    let kw: u16 = 8; // Tastenbreite incl. Abstand

    // Linke Spalte: VERB / NOUN
    render_key("VERB", "", x, y, buf);
    render_key("NOUN", "", x, y + 4, buf);

    // Mittleres Feld 3×5: Zeichen + Ziffern
    let mx = x + kw;

    // Zeile 1: +, 7, 8, 9, CLR
    render_key("+", "", mx, y, buf);
    render_key("7", "", mx + kw, y, buf);
    render_key("8", "", mx + kw * 2, y, buf);
    render_key("9", "", mx + kw * 3, y, buf);
    render_key("CLR", "", mx + kw * 4, y, buf);

    // Zeile 2: −, 4, 5, 6, PRO
    render_key("−", "", mx, y + 4, buf);
    render_key("4", "", mx + kw, y + 4, buf);
    render_key("5", "", mx + kw * 2, y + 4, buf);
    render_key("6", "", mx + kw * 3, y + 4, buf);
    render_key("PRO", "", mx + kw * 4, y + 4, buf);

    // Zeile 3: 0, 1, 2, 3, KEY REL
    render_key("0", "", mx, y + 8, buf);
    render_key("1", "", mx + kw, y + 8, buf);
    render_key("2", "", mx + kw * 2, y + 8, buf);
    render_key("3", "", mx + kw * 3, y + 8, buf);
    render_key("KEY", "REL", mx + kw * 4, y + 8, buf);

    // Rechte Spalte: ENTR / RSET
    let rx = mx + kw * 5;
    render_key("ENTR", "", rx, y, buf);
    render_key("RSET", "", rx, y + 8, buf);

    // Tastatur-Legende (zwei Zeilen)
    let legend1 = Line::from(vec![
        Span::styled("  Tasten: ", Style::default().fg(Color::DarkGray)),
        Span::styled("0–9", Style::default().fg(Color::White)),
        Span::styled("  ", Style::default().fg(Color::DarkGray)),
        Span::styled("v", Style::default().fg(Color::Yellow)),
        Span::styled("=VERB  ", Style::default().fg(Color::DarkGray)),
        Span::styled("n", Style::default().fg(Color::Yellow)),
        Span::styled("=NOUN  ", Style::default().fg(Color::DarkGray)),
        Span::styled("e/Enter", Style::default().fg(Color::Yellow)),
        Span::styled("=ENTR  ", Style::default().fg(Color::DarkGray)),
        Span::styled("c", Style::default().fg(Color::Yellow)),
        Span::styled("=CLR", Style::default().fg(Color::DarkGray)),
    ]);
    let legend2 = Line::from(vec![
        Span::styled("          ", Style::default().fg(Color::DarkGray)),
        Span::styled("p", Style::default().fg(Color::Yellow)),
        Span::styled("=PRO  ", Style::default().fg(Color::DarkGray)),
        Span::styled("r", Style::default().fg(Color::Yellow)),
        Span::styled("=RSET  ", Style::default().fg(Color::DarkGray)),
        Span::styled("k", Style::default().fg(Color::Yellow)),
        Span::styled("=KEYREL  ", Style::default().fg(Color::DarkGray)),
        Span::styled("q", Style::default().fg(Color::Red)),
        Span::styled("=Beenden", Style::default().fg(Color::DarkGray)),
    ]);
    buf.set_line(x, y + 12, &legend1, 80);
    buf.set_line(x, y + 13, &legend2, 80);
}
