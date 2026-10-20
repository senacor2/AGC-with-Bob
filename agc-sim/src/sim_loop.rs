//! Simulations-Haupt-Loop für die DSKY-TUI.
//!
//! Verbindet `agc-core`-Logik, `agc-physics`-Physik und die ratatui-TUI.
//!
//! # Schleifenstruktur
//! ```text
//! Initialisierung:
//!   SimState aufbauen (Starttrajektorie LEO 400 km)
//!
//! Haupt-Loop (≈100 Hz / 10 ms pro Tick):
//!   1. Tastatureingaben lesen (crossterm) → AGC-Keycode → SimDsky.push_key()
//!   2. AGC-Core T4RUPT: t4rupt_tick(), write_to_dsky()
//!   3. Tastendruck aus SimDsky-FIFO → process_key()
//!   4. Physikschritt: RK4 dt=10 ms
//!   5. IMU-Update (spezifische Kraft ≈ 0 im freien Fall)
//!   6. TUI rendern
//! ```

use std::time::{Duration, Instant};

use crossterm::event::{self, Event, KeyCode as CrossKey, KeyModifiers};
use ratatui::{
    layout::{Constraint, Direction, Layout},
    Terminal,
};

use agc_core::alarm::AlarmState;
use agc_core::display::verb_noun::{
    process_key, t4rupt_tick, write_to_dsky, DisplayState, InputBuffer, KeyCode as AgcKey, KEY_0,
    KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_9, KEY_CLR, KEY_ENTER, KEY_KEY_REL,
    KEY_MINUS, KEY_NOUN, KEY_PLUS, KEY_PRO, KEY_RSET, KEY_VERB,
};
use agc_core::hal::DskyHal;
use agc_physics::math::Vec3;
use agc_physics::trajectory::{propagate, State, StepParams};

use crate::hal::{SimDsky, SimImu};
use crate::tui::{
    alarm_display::AlarmDisplayWidget, dsky_widget::DskyWidget, state_display::StateDisplayWidget,
};

// ---------------------------------------------------------------------------
// Sim-Tick-Dauer
// ---------------------------------------------------------------------------

/// AGC T4RUPT-Intervall: 10 ms.
const TICK_MS: u64 = 10;
const TICK_DUR: Duration = Duration::from_millis(TICK_MS);

// ---------------------------------------------------------------------------
// Gesamtzustand des Simulators
// ---------------------------------------------------------------------------

/// Gesamtzustand des AGC-Simulators.
pub struct SimState {
    /// AGC Display- und Tastaturzustand.
    pub display: DisplayState,
    /// Tastatureingabe-Puffer (Verb/Noun-Eingabe).
    pub input_buf: InputBuffer,
    /// Program-Alarm-Zustand.
    pub alarms: AlarmState,
    /// Physikalischer Zustandsvektor.
    pub traj: State,
    /// Sim-DSKY-HAL (Tastatur-FIFO + Display-Puffer).
    pub dsky: SimDsky,
    /// Sim-IMU.
    pub imu: SimImu,
    /// `true` wenn der AGC gerade rechnet (COMP ACTY).
    pub comp_acty: bool,
    /// Tick-Zähler (je 10 ms).
    pub tick: u64,
}

impl SimState {
    /// Initialisiert den Simulator auf einer LEO-Kreisbahn (400 km Höhe).
    pub fn new() -> Self {
        // LEO-Kreisbahn: r = R_erde + 400 km, v = √(μ/r)
        let r = 6_371_000.0_f64 + 400_000.0_f64;
        let mu = 3.986_004_418e14_f64; // m³/s² (Erde)
        let v_circ = (mu / r).sqrt();

        let position = Vec3::new(r, 0.0, 0.0);
        let velocity = Vec3::new(0.0, v_circ, 0.0);

        Self {
            display: DisplayState::new(),
            input_buf: InputBuffer::new(),
            alarms: AlarmState::new(),
            traj: State::new(position, velocity, 0.0),
            dsky: SimDsky::new(),
            imu: SimImu::new(),
            comp_acty: false,
            tick: 0,
        }
    }
}

impl Default for SimState {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Einzel-Tick
// ---------------------------------------------------------------------------

/// Führt einen einzelnen Simulations-Tick durch (10 ms).
///
/// 1. T4RUPT: Flash-Zähler dekrementieren, Display-Wort ausgeben.
/// 2. Tastendruck aus SimDsky-FIFO verarbeiten.
/// 3. RK4-Physik-Schritt.
/// 4. IMU aktualisieren.
pub fn sim_tick(state: &mut SimState) {
    let dt = TICK_MS as f64 / 1000.0;

    // --- T4RUPT ---
    t4rupt_tick(&mut state.display);
    write_to_dsky(&state.display, &mut state.dsky);

    // --- Tastendruck aus SimDsky-FIFO verarbeiten ---
    // SimDsky implementiert DskyHal — read_keycode() leert den FIFO.
    while let Some(raw) = state.dsky.read_keycode() {
        if let Some(agc_key) = AgcKey::new(raw) {
            process_key(
                &mut state.display,
                &mut state.input_buf,
                agc_key,
                &mut state.alarms,
            );
        }
    }

    // --- Physik: RK4 ---
    let params = StepParams::earth_ballistic();
    state.traj = propagate(state.traj, dt, 1, &params);

    // --- IMU: spezifische Kraft ≈ 0 im freien Fall (Keplerbahn) ---
    state.imu.update(Vec3::ZERO, dt);

    // COMP ACTY: leuchtet alle 40 Ticks kurz auf (ca. 400 ms Periode, 40 ms EIN)
    state.comp_acty = (state.tick % 40) < 4;
    state.tick += 1;
}

// ---------------------------------------------------------------------------
// TUI-Layout
// ---------------------------------------------------------------------------

/// Rendert die komplette TUI in das angegebene [`Terminal`].
pub fn render<B: ratatui::backend::Backend>(
    terminal: &mut Terminal<B>,
    state: &SimState,
) -> std::io::Result<()> {
    terminal.draw(|frame| {
        let area = frame.area();

        // Haupt-Layout: DSKY links (62 breit), Info rechts
        let chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([Constraint::Length(62), Constraint::Min(32)])
            .split(area);

        // DSKY-Panel
        frame.render_widget(DskyWidget::new(&state.display, state.comp_acty), chunks[0]);

        // Rechte Spalte: Zustandsvektor oben, Alarm-Log unten
        let right_chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Percentage(55), Constraint::Percentage(45)])
            .split(chunks[1]);

        frame.render_widget(StateDisplayWidget::new(Some(&state.traj)), right_chunks[0]);
        frame.render_widget(AlarmDisplayWidget::new(&state.alarms), right_chunks[1]);
    })?;
    Ok(())
}

// ---------------------------------------------------------------------------
// Event → AGC-Keycode-Mapping
// ---------------------------------------------------------------------------

/// Mappt einen crossterm-`KeyCode` auf einen AGC-Rohkeycode (u8).
///
/// Gibt `None` zurück wenn keine Entsprechung definiert ist.
pub fn map_key(key: CrossKey, mods: KeyModifiers) -> Option<u8> {
    match key {
        CrossKey::Char('0') => Some(KEY_0),
        CrossKey::Char('1') => Some(KEY_1),
        CrossKey::Char('2') => Some(KEY_2),
        CrossKey::Char('3') => Some(KEY_3),
        CrossKey::Char('4') => Some(KEY_4),
        CrossKey::Char('5') => Some(KEY_5),
        CrossKey::Char('6') => Some(KEY_6),
        CrossKey::Char('7') => Some(KEY_7),
        CrossKey::Char('8') => Some(KEY_8),
        CrossKey::Char('9') => Some(KEY_9),
        CrossKey::Char('v') | CrossKey::Char('V') => Some(KEY_VERB),
        CrossKey::Char('n') | CrossKey::Char('N') => Some(KEY_NOUN),
        CrossKey::Enter => Some(KEY_ENTER),
        CrossKey::Char('c') | CrossKey::Char('C') if !mods.contains(KeyModifiers::CONTROL) => {
            Some(KEY_CLR)
        }
        CrossKey::Char('p') | CrossKey::Char('P') => Some(KEY_PRO),
        CrossKey::Char('r') | CrossKey::Char('R') => Some(KEY_RSET),
        CrossKey::Char('k') | CrossKey::Char('K') => Some(KEY_KEY_REL),
        CrossKey::Char('+') => Some(KEY_PLUS),
        CrossKey::Char('-') if mods.is_empty() => Some(KEY_MINUS),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Haupt-Einstiegspunkt der TUI
// ---------------------------------------------------------------------------

/// Startet die TUI-Hauptschleife.
///
/// Initialisiert crossterm/ratatui, läuft den Sim-Loop bis der Nutzer
/// `q` oder `Ctrl+C` drückt, und stellt das Terminal danach wieder her.
pub fn run_tui() -> std::io::Result<()> {
    use crossterm::{
        execute,
        terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
    };
    use ratatui::{backend::CrosstermBackend, Terminal};

    enable_raw_mode()?;
    let mut stdout = std::io::stdout();
    execute!(stdout, EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let result = event_loop(&mut terminal);

    // Terminal immer wiederherstellen, auch bei Fehler
    let _ = disable_raw_mode();
    let _ = crossterm::execute!(terminal.backend_mut(), LeaveAlternateScreen);
    let _ = terminal.show_cursor();

    result
}

fn event_loop<B: ratatui::backend::Backend>(terminal: &mut Terminal<B>) -> std::io::Result<()> {
    let mut state = SimState::new();
    let mut last_tick = Instant::now();

    loop {
        // Render
        render(terminal, &state)?;

        // Event-Polling (nicht-blockierend, mit Timeout = Tick-Rest)
        let elapsed = last_tick.elapsed();
        let timeout = TICK_DUR.saturating_sub(elapsed);

        if event::poll(timeout)? {
            if let Event::Key(key) = event::read()? {
                // Beenden mit q oder Ctrl+C
                if key.code == CrossKey::Char('q')
                    || (key.code == CrossKey::Char('c')
                        && key.modifiers.contains(KeyModifiers::CONTROL))
                {
                    return Ok(());
                }

                // Tastendruck an SimDsky weiterleiten
                if let Some(agc_code) = map_key(key.code, key.modifiers) {
                    state.dsky.push_key(agc_code);
                }
            }
        }

        // Tick, wenn Intervall abgelaufen
        if last_tick.elapsed() >= TICK_DUR {
            sim_tick(&mut state);
            last_tick = Instant::now();
        }
    }
}
