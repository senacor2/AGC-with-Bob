//! Zustandsvektor-Anzeige-Widget für die TUI.
//!
//! Zeigt Positions-, Geschwindigkeits- und MET-Daten aus dem
//! [`agc_physics::trajectory::State`]-Struct an.

use ratatui::{
    buffer::Buffer,
    layout::Rect,
    style::{Color, Modifier, Style},
    widgets::Widget,
};

use agc_physics::trajectory::State;

/// Ratatui-Widget das Positions-, Geschwindigkeits- und MET-Daten darstellt.
pub struct StateDisplayWidget<'a> {
    state: Option<&'a State>,
}

impl<'a> StateDisplayWidget<'a> {
    /// Erstellt ein neues Widget.  `None` → zeigt Platzhalter an.
    pub fn new(state: Option<&'a State>) -> Self {
        Self { state }
    }
}

impl Widget for StateDisplayWidget<'_> {
    fn render(self, area: Rect, buf: &mut Buffer) {
        render_state_panel(self.state, area, buf);
    }
}

fn render_state_panel(state: Option<&State>, area: Rect, buf: &mut Buffer) {
    let header_sty = Style::default()
        .fg(Color::Cyan)
        .add_modifier(Modifier::BOLD | Modifier::UNDERLINED);
    let label_sty = Style::default().fg(Color::DarkGray);
    let value_sty = Style::default().fg(Color::White);
    let unit_sty = Style::default().fg(Color::DarkGray);

    let x = area.x;
    let mut row = area.y;

    buf.set_string(
        x,
        row,
        "── ZUSTANDSVEKTOR ──────────────────────",
        header_sty,
    );
    row += 1;

    let (pos, vel, time_s) = match state {
        Some(s) => (s.position, s.velocity, s.time_s),
        None => {
            buf.set_string(x, row, "  (keine Simulationsdaten)", label_sty);
            return;
        }
    };

    // MET
    let met_s = time_s as u64;
    let h = met_s / 3600;
    let m = (met_s % 3600) / 60;
    let s = met_s % 60;
    let frac = time_s.fract();
    buf.set_string(x, row, "  MET  ", label_sty);
    buf.set_string(
        x + 7,
        row,
        format!("{:03}:{:02}:{:05.2}", h, m, s as f64 + frac),
        value_sty,
    );
    row += 1;

    // Höhe über Erdoberfläche: Äquatorialradius WGS-84 (6 378.137 km).
    // Konsistent mit agc-physics/bodies.rs R_EARTH und dem entry_pre-Szenario
    // (das ebenfalls r_equatorial = 6 378 137 m verwendet).
    let r_norm = pos.magnitude();
    let alt_km = (r_norm - 6_378_137.0) / 1000.0;
    buf.set_string(x, row, "  ALT  ", label_sty);
    buf.set_string(x + 7, row, format!("{:>12.3}", alt_km), value_sty);
    buf.set_string(x + 20, row, " km", unit_sty);
    row += 1;

    // Geschwindigkeitsbetrag
    let v_norm = vel.magnitude();
    buf.set_string(x, row, "  VEL  ", label_sty);
    buf.set_string(x + 7, row, format!("{:>12.3}", v_norm), value_sty);
    buf.set_string(x + 20, row, " m/s", unit_sty);
    row += 1;

    // Positionskomponenten (ECI)
    buf.set_string(x, row, "  POS  ", label_sty);
    buf.set_string(x + 7, row, "ECI (m)", unit_sty);
    row += 1;

    for (label, v) in [("    X ", pos.x), ("    Y ", pos.y), ("    Z ", pos.z)] {
        if row >= area.y + area.height {
            return;
        }
        buf.set_string(x, row, label, label_sty);
        buf.set_string(x + 7, row, format!("{:>15.1}", v), value_sty);
        row += 1;
    }

    // Geschwindigkeitskomponenten (ECI)
    if row < area.y + area.height {
        buf.set_string(x, row, "  VEL  ", label_sty);
        buf.set_string(x + 7, row, "ECI (m/s)", unit_sty);
        row += 1;
    }
    for (label, v) in [("    X ", vel.x), ("    Y ", vel.y), ("    Z ", vel.z)] {
        if row >= area.y + area.height {
            return;
        }
        buf.set_string(x, row, label, label_sty);
        buf.set_string(x + 7, row, format!("{:>15.3}", v), value_sty);
        row += 1;
    }
    let _ = row;
}
