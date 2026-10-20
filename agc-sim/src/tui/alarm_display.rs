//! Alarm-Log-Panel-Widget für die TUI.
//!
//! Zeigt die zuletzt aufgetretenen AGC Program-Alarm-Codes aus
//! [`agc_core::alarm::AlarmState`] an.

use ratatui::{
    buffer::Buffer,
    layout::Rect,
    style::{Color, Modifier, Style},
    widgets::Widget,
};

use agc_core::alarm::AlarmState;

/// Ratatui-Widget das Program-Alarm-Codes anzeigt.
pub struct AlarmDisplayWidget<'a> {
    alarms: &'a AlarmState,
}

impl<'a> AlarmDisplayWidget<'a> {
    /// Erstellt ein neues Widget.
    pub fn new(alarms: &'a AlarmState) -> Self {
        Self { alarms }
    }
}

impl Widget for AlarmDisplayWidget<'_> {
    fn render(self, area: Rect, buf: &mut Buffer) {
        render_alarm_panel(self.alarms, area, buf);
    }
}

fn render_alarm_panel(alarms: &AlarmState, area: Rect, buf: &mut Buffer) {
    let header_sty = Style::default()
        .fg(Color::Red)
        .add_modifier(Modifier::BOLD | Modifier::UNDERLINED);
    let ok_sty = Style::default().fg(Color::Green);
    let alarm_sty = Style::default()
        .fg(Color::LightRed)
        .add_modifier(Modifier::BOLD);
    let code_sty = Style::default().fg(Color::Yellow);
    let label_sty = Style::default().fg(Color::DarkGray);

    let x = area.x;
    let mut row = area.y;

    buf.set_string(
        x,
        row,
        "── ALARM-LOG ───────────────────────────",
        header_sty,
    );
    row += 1;

    let codes = alarms.as_slice();

    if codes.is_empty() {
        buf.set_string(x, row, "  ✓ Keine Alarme", ok_sty);
        return;
    }

    for (i, &code) in codes.iter().enumerate() {
        if row >= area.y + area.height {
            break;
        }
        buf.set_string(x, row, format!("  [{}] ", i + 1), label_sty);
        buf.set_string(x + 6, row, format!("P{:04o}", code), alarm_sty);
        buf.set_string(x + 12, row, " — ", label_sty);
        buf.set_string(x + 15, row, alarm_description(code), code_sty);
        row += 1;
    }

    if row < area.y + area.height {
        row += 1;
        buf.set_string(
            x,
            row,
            "  [r] RSET zum Löschen",
            Style::default().fg(Color::DarkGray),
        );
    }
}

/// Liefert eine kurze Beschreibung bekannter AGC-Alarmcodes.
///
/// Codebasis: Comanche055-Alarmcodes (Oktal-Nummern).
fn alarm_description(code: u16) -> &'static str {
    match code {
        0o0110 => "PIPA-Fehler",
        0o0111 => "PIPA-Überlauf",
        0o0114 => "PIPA-Drift",
        0o0200 => "IMU-Ausfall",
        0o0210 => "Gimbal-Lock",
        0o0300 => "RCS-Fehler",
        0o0400 => "SPS-Fehler",
        0o0500 => "VHF-Ranging-Fehler",
        0o1000 => "Executive-Überlauf",
        0o1001 => "Waitlist-Überlauf",
        0o1002 => "Phase-Table-Fehler",
        0o1100 => "Display-Queue-Überlauf",
        0o1200 => "Nav-Fehler",
        0o1300 => "Guidance-Fehler",
        0o1400 => "DAP-Fehler",
        _ => "Unbekannter Alarm",
    }
}
