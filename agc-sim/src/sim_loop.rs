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
//!   3. Tastendruck aus SimDsky-FIFO → process_key() → VerbDispatcher::dispatch()
//!   4. Monitor-Modus (V16): NounResolver::resolve() → Display aktualisieren
//!   5. Physikschritt: RK4 dt=10 ms
//!   6. IMU-Update (spezifische Kraft ≈ 0 im freien Fall)
//!   7. TUI rendern
//! ```

use std::time::{Duration, Instant};

use crossterm::event::{self, Event, KeyCode as CrossKey, KeyModifiers};
use ratatui::{
    layout::{Constraint, Direction, Layout},
    Terminal,
};

use agc_core::alarm::AlarmState;
use agc_core::context::{AgcContext, AlignmentCommand};
use agc_core::display::dispatcher::VerbDispatcher;
use agc_core::display::noun_resolver;
use agc_core::display::verb_noun::{
    process_key, t4rupt_tick, write_to_dsky, InputBuffer, InputMode, KeyCode as AgcKey, KeyEvent,
    KEY_0, KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_9, KEY_CLR, KEY_ENTER,
    KEY_KEY_REL, KEY_MINUS, KEY_NOUN, KEY_PLUS, KEY_PRO, KEY_RSET, KEY_VERB,
};
use agc_core::hal::DskyHal;
use agc_physics::bodies::Body;
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
    /// Zentraler AGC-Zustand: Display, Executive, PhaseTable, GuidanceContext.
    /// Sub-Task 6: AgcContext ersetzt die separate DisplayState — Display wird
    /// über `ctx.display` zugegriffen.
    pub ctx: AgcContext,
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
    /// Roher Eingabepuffer für R1 (entspricht AGC XREG).
    /// Wird bei V25-ENTER an `accept_load` übergeben, nicht der Anzeige-Wert.
    pub load_r1: i32,
    /// Roher Eingabepuffer für R2 (entspricht AGC YREG).
    pub load_r2: i32,
    /// Roher Eingabepuffer für R3 (entspricht AGC ZREG).
    pub load_r3: i32,
    /// Index des nächsten zu befüllenden Lade-Registers (0=R1, 1=R2, 2=R3).
    /// Wird bei V25-Dateneingabe nach jeder Zahl-ENTER inkrementiert.
    pub load_reg_idx: u8,
    /// Zahlenpuffer für V25-Dateneingabe: akkumulierte Ziffern (Absolutwert).
    ///
    /// Erlaubt Werte ±99999 (5-stellig). Ziffern werden per Keypad eingegeben,
    /// Vorzeichen per KEY_PLUS / KEY_MINUS, ENTER übernimmt den Wert.
    pub load_num_buf: i32,
    /// Vorzeichen des aktuellen Zahlenpuffers: +1 oder −1.
    pub load_num_sign: i32,
    /// Anzahl der bereits eingetippten Ziffern (max. 5).
    pub load_num_digits: u8,
    /// Aktiver Zentralkörper für den RK4-Integrator.
    ///
    /// Wird beim Laden eines Szenarios automatisch aus der Bahnhöhe abgeleitet:
    /// `|r| < 3 000 000 m` → `Body::Moon`, sonst → `Body::Earth`.
    pub step_body: Body,
    /// Eingabemodus für Szenario-Dateiname (`@`-Taste).
    ///
    /// `Some(String)` = TUI zeigt Eingabefeld; `None` = normaler Modus.
    pub scenario_input: Option<String>,
    /// Einzeilige Statusmeldung im TUI-Footer (Fehler, Bestätigungen).
    pub status_msg: Option<String>,
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
            ctx: AgcContext::default(),
            input_buf: InputBuffer::new(),
            alarms: AlarmState::new(),
            traj: State::new(position, velocity, 0.0),
            dsky: SimDsky::new(),
            imu: SimImu::new(),
            comp_acty: false,
            tick: 0,
            load_r1: 0,
            load_r2: 0,
            load_r3: 0,
            load_reg_idx: 0,
            load_num_buf: 0,
            load_num_sign: 1,
            load_num_digits: 0,
            step_body: Body::Earth,
            scenario_input: None,
            status_msg: None,
        }
    }

    /// Lädt ein Szenario in den SimState.
    ///
    /// Überschreibt `traj` mit dem Szenario-Bahnzustand und setzt `ctx` auf
    /// Default zurück. Der Zentralkörper (`step_body`) wird automatisch aus
    /// der Bahnhöhe abgeleitet:
    /// - `|r| < 3 000 000 m` → `Body::Moon` (Mondorbit)
    /// - sonst               → `Body::Earth`
    ///
    /// Optionale `thetad_deg`-Felder im Szenario werden nach Rad konvertiert
    /// und in `ctx.guidance.thetad` gespeichert.
    pub fn load_scenario(&mut self, data: &crate::scenarios::ScenarioData) {
        self.traj = data.to_state();
        self.ctx = AgcContext::default();
        self.input_buf = InputBuffer::new();
        self.alarms = AlarmState::new();
        self.load_r1 = 0;
        self.load_r2 = 0;
        self.load_r3 = 0;
        self.load_reg_idx = 0;
        self.load_num_buf = 0;
        self.load_num_sign = 1;
        self.load_num_digits = 0;

        // Zentralkörper aus Bahnradius ableiten.
        // Schwelle 3 000 000 m: Mondorbit max. ~2 500 km Radius, Erdorbit min. ~6 371 km.
        // Kein Überlappungsrisiko zwischen Mond- und Erdorbits.
        const MOON_THRESHOLD_M: f64 = 3_000_000.0;
        self.step_body = if data.position.magnitude() < MOON_THRESHOLD_M {
            Body::Moon
        } else {
            Body::Earth
        };

        // Optionale Gimbal-Zielwinkel aus Szenario übernehmen (Grad → Rad).
        if let Some(td_deg) = data.thetad_deg {
            const DEG_TO_RAD: f64 = core::f64::consts::PI / 180.0;
            self.ctx.guidance.thetad = [
                td_deg[0] * DEG_TO_RAD,
                td_deg[1] * DEG_TO_RAD,
                td_deg[2] * DEG_TO_RAD,
            ];
        }

        // Optionale Fahrzeugmasse aus Szenario übernehmen.
        if let Some(m) = data.mass_kg {
            self.ctx.guidance.vehicle_mass_kg = m;
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
/// 1. T4RUPT: Flash-Zähler dekrementieren.
/// 2. Monitor-Modus (V16): NounResolver → Display aktualisieren.
/// 3. Display an SimDsky-HAL ausgeben.
/// 4. Tastendruck aus SimDsky-FIFO verarbeiten → VerbDispatcher.
/// 5. RK4-Physik-Schritt.
/// 6. IMU aktualisieren.
pub fn sim_tick(state: &mut SimState) {
    let dt = TICK_MS as f64 / 1000.0;

    // --- Aktuellen Bahnzustandsvektor in ctx übertragen ---
    // Guidance-Programme (P30) lesen ctx.current_sv als TIG-Zustandsvektor.
    // agc_physics::State → agc_core::StateVector konvertieren.
    {
        use agc_core::navigation::state_vector::StateVector;
        use agc_core::types::{Met, Vec3 as CoreVec3};
        state.ctx.current_sv = StateVector::new(
            CoreVec3::new(
                state.traj.position.x,
                state.traj.position.y,
                state.traj.position.z,
            ),
            CoreVec3::new(
                state.traj.velocity.x,
                state.traj.velocity.y,
                state.traj.velocity.z,
            ),
            Met::from_seconds(state.traj.time_s),
        );
    }

    // --- Alarm-Zustand aus sim-eigenem Puffer in ctx synchronisieren ---
    // Guidance-Programme schreiben in ctx.alarms; nach dem Tick schreiben wir
    // ctx.alarms zurück in state.alarms (für TUI und process_key).
    state.ctx.alarms = state.alarms;

    // --- T4RUPT ---
    t4rupt_tick(&mut state.ctx.display);

    // --- Monitor-Modus (V16): NounResolver 1x pro Sekunde aufrufen ---
    // 1 Hz = 100 Ticks à 10 ms, entspricht AGC MONITOR-Zyklus.
    // AGC source: Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, MONITOR label.
    if state.ctx.display.verb.val() == 16 && state.tick.is_multiple_of(100) {
        let noun = state.ctx.display.noun;
        let [r1, r2, r3] = noun_resolver::resolve(noun, &state.ctx);
        state.ctx.display.r1 = r1;
        state.ctx.display.r2 = r2;
        state.ctx.display.r3 = r3;
    }

    write_to_dsky(&state.ctx.display, &mut state.dsky);

    // --- Tastendruck aus SimDsky-FIFO verarbeiten ---
    // SimDsky implementiert DskyHal — read_keycode() leert den FIFO.
    while let Some(raw) = state.dsky.read_keycode() {
        if let Some(agc_key) = AgcKey::new(raw) {
            // V25-Dateneingabe-Modus: Im Flash-Modus mit V25 und einem Noun der
            // keine spezielle Behandlung hat (nicht N22/N93), fangen wir
            // Ziffern, +/−, CLR und ENTER direkt ab — bevor process_key sie
            // als Operator Error oder Noun-Wert (≤99) fehlinterpretiert.
            //
            // Erlaubt Werte ±99999 (5 Ziffern), kompatibel mit AGC XREG/YREG/ZREG.
            //
            // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, ENTPAS / STOREDAT.
            let v25_data_mode = state.ctx.display.flash_state
                && state.ctx.display.verb.val() == 25
                && !matches!(state.ctx.display.noun.val(), 22 | 93);

            if v25_data_mode {
                use agc_core::display::verb_noun::{DisplayRegister, KEY_CLR, KEY_MINUS, KEY_PLUS};
                let k = agc_key.val();
                match k {
                    // Vorzeichen: setzt Vorzeichen, setzt Ziffernpuffer zurück
                    KEY_PLUS => {
                        state.load_num_sign = 1;
                        state.load_num_buf = 0;
                        state.load_num_digits = 0;
                        state.ctx.display.lights.operator_error = false;
                        continue;
                    }
                    KEY_MINUS => {
                        state.load_num_sign = -1;
                        state.load_num_buf = 0;
                        state.load_num_digits = 0;
                        state.ctx.display.lights.operator_error = false;
                        continue;
                    }
                    // Ziffern 0–9: in Zahlenpuffer akkumulieren (max. 5 Stellen)
                    0..=9 => {
                        if state.load_num_digits < 5 {
                            state.load_num_buf = state.load_num_buf * 10 + k as i32;
                            state.load_num_digits += 1;
                            // Live-Anzeige des Puffers im aktuellen Register
                            let live =
                                DisplayRegister::Decimal(state.load_num_sign * state.load_num_buf);
                            match state.load_reg_idx {
                                0 => state.ctx.display.r1 = live,
                                1 => state.ctx.display.r2 = live,
                                _ => state.ctx.display.r3 = live,
                            }
                        }
                        continue;
                    }
                    // CLR: Zahlenpuffer zurücksetzen
                    KEY_CLR => {
                        state.load_num_buf = 0;
                        state.load_num_sign = 1;
                        state.load_num_digits = 0;
                        state.ctx.display.lights.operator_error = false;
                        continue;
                    }
                    // ENTER: aktuellen Zahlenpuffer als Registerwert übernehmen,
                    // dann Index inkrementieren. Bei R3: accept_load() aufrufen.
                    //
                    // Wenn keine Ziffern eingegeben wurden (load_num_digits == 0),
                    // fällt ENTER durch zum normalen process_key-Pfad. Das erlaubt
                    // accept_load via ENTER ohne vorherige Zifferneingabe (Rückwärts-
                    // kompatibilität mit direktem load_r*-Setzen in Tests).
                    KEY_ENTER if state.load_num_digits > 0 => {
                        let val = state.load_num_sign * state.load_num_buf;
                        match state.load_reg_idx {
                            0 => {
                                state.load_r1 = val;
                                state.ctx.display.r1 = DisplayRegister::Decimal(val);
                                state.load_reg_idx = 1;
                            }
                            1 => {
                                state.load_r2 = val;
                                state.ctx.display.r2 = DisplayRegister::Decimal(val);
                                state.load_reg_idx = 2;
                            }
                            _ => {
                                state.load_r3 = val;
                                state.ctx.display.r3 = DisplayRegister::Decimal(val);
                                // Alle drei Register befüllt → accept_load
                                VerbDispatcher::accept_load(
                                    state.load_r1,
                                    state.load_r2,
                                    state.load_r3,
                                    &mut state.ctx,
                                );
                                state.load_reg_idx = 0;
                            }
                        }
                        // Puffer für nächsten Wert zurücksetzen
                        state.load_num_buf = 0;
                        state.load_num_sign = 1;
                        state.load_num_digits = 0;
                        continue;
                    }
                    // Alle anderen Tasten (VERB, NOUN, …) normal verarbeiten
                    _ => {}
                }
            }

            let event = process_key(
                &mut state.ctx.display,
                &mut state.input_buf,
                agc_key,
                &mut state.alarms,
            );

            // Dispatcher aufrufen wenn Verb oder Noun per ENTER übernommen wurde.
            //
            // AGC VERBFAN-Logik: Der Dispatcher wird erst ausgelöst, wenn Noun
            // gesetzt wird (NounSet). Ein reines VerbSet (ohne nachfolgendes Noun)
            // löst keinen Dispatch aus — das entspricht dem echten PINBALL-Automat,
            // der auf ENTER im Idle-Modus oder NounSet reagiert.
            //
            // Ausnahme: V33/V34/V32 (Proceed/Terminate/Recycle) werden auch ohne
            // Noun per VerbSet sofort ausgelöst, da sie keinen Noun benötigen.
            //
            // AGC source: Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, VERBFAN.
            match event {
                KeyEvent::VerbSet(v) => {
                    // Sofort dispatchen wenn Verb keinen Noun braucht:
                    // - V32/V33/V34: Recycle/Proceed/Terminate
                    // - V37: RequestProgram (Flash einschalten)
                    // - V41/V42: CoarseAlign/FineAlign (Prompt selbst schaltet Flash ein,
                    //   kein Noun nötig für den ersten Schritt)
                    // V06/V16/V25 warten weiterhin auf NounSet.
                    //
                    // AGC source: EXTENDED_VERBS.agc, VBCOARK/IMUFINEK — V41/V42 werden
                    // ohne Noun-Anforderung ausgelöst.
                    let v_raw = v.val();
                    // Alignment-Prompt abbrechen: V34 (Terminate) während Flash+V25+N22/N93
                    // aktiv → Flash stoppen, alignment_cmd NICHT setzen.
                    // Muss vor dem allgemeinen Dispatch geprüft werden, da der Dispatcher
                    // display.verb bereits auf 34 gesetzt hat.
                    //
                    // AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, TERMKEY.
                    let was_alignment_prompt = state.ctx.display.flash_state
                        && matches!(state.ctx.display.noun.val(), 22 | 93);
                    if v_raw == 34 && was_alignment_prompt {
                        agc_core::display::verb_noun::stop_flash(&mut state.ctx.display);
                    } else if matches!(v_raw, 32..=34 | 41 | 42) {
                        let verb = state.ctx.display.verb;
                        let noun = state.ctx.display.noun;
                        VerbDispatcher::dispatch(verb, noun, &mut state.ctx);
                    } else if v_raw == 37 {
                        // V37 erster Schritt: immer mit Noun=0 dispatchen (RequestProgram).
                        // Der alte Noun-Wert im Display darf nicht als Programmnummer
                        // missinterpretiert werden. REQMM setzt NOUNREG=0 und wartet auf
                        // die neue Eingabe.
                        // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, REQMM label.
                        let verb = state.ctx.display.verb;
                        #[expect(
                            clippy::unwrap_used,
                            reason = "NounCode(0) ist garantiert gültig (0 ≤ 99)"
                        )]
                        let noun_zero = agc_core::display::verb_noun::NounCode::new(0).unwrap();
                        VerbDispatcher::dispatch(verb, noun_zero, &mut state.ctx);
                    }
                    // V37: Flash aktiv → Eingabemodus auf Noun setzen, damit die
                    // Programmnummer direkt als Ziffernfolge eingegeben werden kann
                    // (ohne NOUN-Taste). Entspricht AGC REQMM, das NOUNREG beschreibt.
                    // AGC source: PINBALL_GAME__BUTTONS_AND_LIGHTS.agc, REQMM label.
                    if v_raw == 37 && state.ctx.display.flash_state {
                        state.input_buf.clear();
                        state.input_buf.mode = InputMode::Noun;
                    }
                    // Flash verlassen → Eingabe-Sequenz abbrechen, Index zurücksetzen.
                    if !state.ctx.display.flash_state {
                        state.load_reg_idx = 0;
                    }
                }
                KeyEvent::NounSet(_) => {
                    let verb = state.ctx.display.verb;
                    let noun = state.ctx.display.noun;

                    // Im Flash-Modus mit V25: Noun-Eingabe als Datenwert in
                    // load_r1/r2/r3 übernehmen, Index weiterschalten.
                    // AGC source: Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc,
                    // ENTR/NOUNP-Zweig bei aktiver LOAD-Sequenz.
                    if state.ctx.display.flash_state && verb.val() == 25 {
                        let raw = noun.val() as i32;
                        match state.load_reg_idx {
                            0 => {
                                state.load_r1 = raw;
                                state.ctx.display.r1 =
                                    agc_core::display::verb_noun::DisplayRegister::Decimal(raw);
                            }
                            1 => {
                                state.load_r2 = raw;
                                state.ctx.display.r2 =
                                    agc_core::display::verb_noun::DisplayRegister::Decimal(raw);
                            }
                            _ => {
                                state.load_r3 = raw;
                                state.ctx.display.r3 =
                                    agc_core::display::verb_noun::DisplayRegister::Decimal(raw);
                            }
                        }
                        if state.load_reg_idx < 2 {
                            state.load_reg_idx += 1;
                        }
                    } else {
                        // Normaler Modus: Dispatcher auslösen (z. B. V06 N60, V16 N67).
                        VerbDispatcher::dispatch(verb, noun, &mut state.ctx);
                    }
                }
                KeyEvent::Enter => {
                    // Enter im Flash-Modus mit V25 N22/N93:
                    // Alignment-Prompt bestätigen → alignment_cmd setzen, Flash stoppen.
                    //
                    // AGC source: `Comanche055/EXTENDED_VERBS.agc`, VBCOARK / IMUFINEK.
                    if state.ctx.display.flash_state && state.ctx.display.verb.val() == 25 {
                        match state.ctx.display.noun.val() {
                            22 => {
                                state.ctx.guidance.alignment_cmd =
                                    Some(AlignmentCommand::CoarseAlign);
                                agc_core::display::verb_noun::stop_flash(&mut state.ctx.display);
                            }
                            93 => {
                                state.ctx.guidance.alignment_cmd =
                                    Some(AlignmentCommand::FineAlign);
                                agc_core::display::verb_noun::stop_flash(&mut state.ctx.display);
                            }
                            _ => {
                                // Enter im Idle-Modus: V25-Lade-Sequenz abschließen (accept_load).
                                // Crew-Eingabe aus dediziertem Eingabepuffer lesen (XREG/YREG/ZREG).
                                // Nicht aus Display-Registern: die enthalten den NounResolver-Wert,
                                // nicht die Tastatureingabe.
                                VerbDispatcher::accept_load(
                                    state.load_r1,
                                    state.load_r2,
                                    state.load_r3,
                                    &mut state.ctx,
                                );
                            }
                        }
                    } else if state.ctx.display.verb.val() == 25 {
                        // V25 aktiv, aber kein Flash (normaler accept_load-Pfad)
                        VerbDispatcher::accept_load(
                            state.load_r1,
                            state.load_r2,
                            state.load_r3,
                            &mut state.ctx,
                        );
                    }
                }
                KeyEvent::Proceed => {
                    // Proceed (PRO / V33) im Alignment-Prompt-Modus:
                    // Flash+V25+N22 → CoarseAlign, Flash+V25+N93 → FineAlign.
                    //
                    // AGC source: `Comanche055/EXTENDED_VERBS.agc`, VBCOARK / IMUFINEK.
                    if state.ctx.display.flash_state && state.ctx.display.verb.val() == 25 {
                        match state.ctx.display.noun.val() {
                            22 => {
                                state.ctx.guidance.alignment_cmd =
                                    Some(AlignmentCommand::CoarseAlign);
                                agc_core::display::verb_noun::stop_flash(&mut state.ctx.display);
                            }
                            93 => {
                                state.ctx.guidance.alignment_cmd =
                                    Some(AlignmentCommand::FineAlign);
                                agc_core::display::verb_noun::stop_flash(&mut state.ctx.display);
                            }
                            _ => {
                                state.ctx.guidance.proceed_flag = true;
                            }
                        }
                    } else {
                        state.ctx.guidance.proceed_flag = true;
                    }
                }
                KeyEvent::Terminate => {
                    // Terminate (V34) im Alignment-Prompt-Modus:
                    // Flash stoppen, alignment_cmd NICHT setzen (Crew bricht ab).
                    //
                    // AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, TERMKEY.
                    if state.ctx.display.flash_state
                        && state.ctx.display.verb.val() == 25
                        && matches!(state.ctx.display.noun.val(), 22 | 93)
                    {
                        agc_core::display::verb_noun::stop_flash(&mut state.ctx.display);
                    } else {
                        state.ctx.guidance.terminate_flag = true;
                    }
                }
                KeyEvent::Recycle => {
                    state.ctx.guidance.recycle_flag = true;
                }
                KeyEvent::Reset => {
                    // RSET: Flags zurücksetzen
                    state.ctx.guidance.proceed_flag = false;
                    state.ctx.guidance.terminate_flag = false;
                    state.ctx.guidance.recycle_flag = false;
                }
                KeyEvent::None | KeyEvent::OperatorError => {}
            }
        }
    }

    // --- Physik: RK4 ---
    // Zentralkörper aus state.step_body (wird beim Laden eines Szenarios gesetzt).
    let params = StepParams {
        body: state.step_body,
        thrust_accel: Vec3::ZERO,
        alpha_rad: 0.0,
        body_radius_m: state.step_body.radius(),
    };
    state.traj = propagate(state.traj, dt, 1, &params);

    // --- IMU: spezifische Kraft ≈ 0 im freien Fall (Keplerbahn) ---
    state.imu.update(Vec3::ZERO, dt);

    // --- Alarm-Zustand zurückschreiben (Guidance-Alarme an TUI sichtbar machen) ---
    state.alarms = state.ctx.alarms;

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
    use ratatui::{
        style::{Color, Style},
        text::Line,
        widgets::{Block, Borders, Clear, Paragraph},
    };

    terminal.draw(|frame| {
        let area = frame.area();

        // Haupt-Layout: oben Hauptinhalt, unten 1 Zeile Footer
        let main_chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Min(1), Constraint::Length(1)])
            .split(area);

        let main_area = main_chunks[0];
        let footer_area = main_chunks[1];

        // Haupt-Layout: DSKY links (62 breit), Info rechts
        let chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([Constraint::Length(62), Constraint::Min(32)])
            .split(main_area);

        // DSKY-Panel
        frame.render_widget(
            DskyWidget::new(&state.ctx.display, state.comp_acty),
            chunks[0],
        );

        // Rechte Spalte: Zustandsvektor oben, Alarm-Log unten
        let right_chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Percentage(55), Constraint::Percentage(45)])
            .split(chunks[1]);

        frame.render_widget(StateDisplayWidget::new(Some(&state.traj)), right_chunks[0]);
        frame.render_widget(AlarmDisplayWidget::new(&state.alarms), right_chunks[1]);

        // Footer: Hilfszeile oder Statusmeldung
        let footer_text = if let Some(msg) = &state.status_msg {
            Line::styled(format!(" {msg}"), Style::default().fg(Color::Yellow))
        } else {
            Line::styled(
                "  q Beenden  @ Szenario laden  V/N/E/P/R/C/K AGC-Tasten",
                Style::default().fg(Color::DarkGray),
            )
        };
        frame.render_widget(Paragraph::new(footer_text), footer_area);

        // Szenario-Eingabefeld (overlay, wenn @-Modus aktiv)
        if let Some(ref input) = state.scenario_input {
            // Eingabefeld zentriert über dem DSKY-Panel
            let popup_w = 58u16.min(area.width.saturating_sub(2));
            let popup_x = area.x + (area.width.saturating_sub(popup_w)) / 2;
            let popup_y = area.y + area.height / 2;
            let popup_area = ratatui::layout::Rect {
                x: popup_x,
                y: popup_y,
                width: popup_w,
                height: 3,
            };
            frame.render_widget(Clear, popup_area);
            let block = Block::default()
                .title(" Szenario laden (ENTER bestätigen, ESC abbrechen) ")
                .borders(Borders::ALL)
                .style(Style::default().fg(Color::Cyan));
            let inner = block.inner(popup_area);
            frame.render_widget(block, popup_area);
            frame.render_widget(
                Paragraph::new(input.as_str()).style(Style::default().fg(Color::White)),
                inner,
            );
        }
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
        // Enter-Taste oder 'e'/'E' — beides löst ENTR aus
        CrossKey::Enter | CrossKey::Char('e') | CrossKey::Char('E') => Some(KEY_ENTER),
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
///
/// # Parameter
/// - `scenario_path`: optionaler Pfad zu einer `.scenario`-Datei, die vor dem
///   ersten Tick geladen wird (entspricht `--scenario <path>` CLI-Flag).
pub fn run_tui(scenario_path: Option<&str>) -> std::io::Result<()> {
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

    let result = event_loop(&mut terminal, scenario_path);

    // Terminal immer wiederherstellen, auch bei Fehler
    let _ = disable_raw_mode();
    let _ = crossterm::execute!(terminal.backend_mut(), LeaveAlternateScreen);
    let _ = terminal.show_cursor();

    result
}

fn event_loop<B: ratatui::backend::Backend>(
    terminal: &mut Terminal<B>,
    scenario_path: Option<&str>,
) -> std::io::Result<()> {
    let mut state = SimState::new();

    // CLI-Flag: Szenario vorladen
    if let Some(path) = scenario_path {
        match crate::scenarios::load_scenario_file(path) {
            Ok(data) => {
                state.load_scenario(&data);
                state.status_msg = Some(format!("Szenario geladen: {path}"));
            }
            Err(e) => {
                state.status_msg = Some(format!("Fehler: {e}"));
            }
        }
    }

    let mut last_tick = Instant::now();

    loop {
        // Render
        render(terminal, &state)?;

        // Event-Polling (nicht-blockierend, mit Timeout = Tick-Rest)
        let elapsed = last_tick.elapsed();
        let timeout = TICK_DUR.saturating_sub(elapsed);

        if event::poll(timeout)? {
            if let Event::Key(key) = event::read()? {
                // Eingabemodus für Szenario-Dateiname (@-Taste)
                if let Some(ref mut input) = state.scenario_input.clone() {
                    match key.code {
                        CrossKey::Esc => {
                            state.scenario_input = None;
                        }
                        CrossKey::Enter => {
                            let path = input.trim().to_string();
                            state.scenario_input = None;
                            match crate::scenarios::load_scenario_file(&path) {
                                Ok(data) => {
                                    state.load_scenario(&data);
                                    state.status_msg = Some(format!("Szenario geladen: {path}"));
                                }
                                Err(e) => {
                                    state.status_msg = Some(format!("Fehler: {e}"));
                                }
                            }
                        }
                        CrossKey::Backspace => {
                            if let Some(ref mut s) = state.scenario_input {
                                s.pop();
                            }
                        }
                        CrossKey::Char(c) => {
                            if let Some(ref mut s) = state.scenario_input {
                                s.push(c);
                            }
                        }
                        _ => {}
                    }
                    continue; // Keine AGC-Tastenverarbeitung im Eingabemodus
                }

                // Beenden mit q oder Ctrl+C
                if key.code == CrossKey::Char('q')
                    || (key.code == CrossKey::Char('c')
                        && key.modifiers.contains(KeyModifiers::CONTROL))
                {
                    return Ok(());
                }

                // @-Taste: Szenario-Eingabemodus öffnen
                if key.code == CrossKey::Char('@') {
                    state.scenario_input = Some(String::new());
                    state.status_msg = None;
                    continue;
                }

                // Statusmeldung bei nächster Taste löschen
                if state.status_msg.is_some() {
                    state.status_msg = None;
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
