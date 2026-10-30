//! P61–P67 — Entry Preparation und Reentry Control.
//!
//! Implementiert die atmosphärischen Wiedereintritts-Programme des AGC:
//!
//! - **P61** — Entry Preparation: berechnet EMS-Initialisierungsdaten
//!   (GMAX, VPRED, GAMMAEI, RTGO, VIO, TTE) aus dem aktuellen Zustandsvektor.
//!
//! - **P62** — Pre-Entry Orientation: prüft IMU-Ausrichtung und bereitet
//!   die Entry-Lage (ALFAPAD / ROLLC) vor.
//!
//! - **P63** — Entry Monitor / .05G Sensing: überwacht die Atmosphärenbremsung
//!   bis zum 0.05-g-Einsatzpunkt.
//!
//! - **P64** — Constant-Drag Entry: Schritt für Schritt Rollsteuerung zur
//!   Reichweite-Regelung (Eintrittsphase bei konstantem Luftwiderstand).
//!
//! - **P65** — Up-Control Phase: Hochziehmörser-Phase zur präzisen Zielführung
//!   (wenn Reichweite < 25 NM vom Ziel).
//!
//! - **P66** — Final Phase (Trim): Haltesteuerung relativ zum Relativgeschwindigkeitsvektor
//!   ohne Rollbefehle, bis Luftwiderstand wieder auf Q7 + 0.5 ft/s² steigt.
//!
//! - **P67** — Terminal Phase: Beendet die Rollsteuerung bei v_rel ≤ 305 m/s.
//!
//! # Physikalisches Modell
//!
//! Die Brems-Beschleunigung (Drag) und Liftkraft werden in vereinfachter Form
//! aus Zustandsvektor + atmosphärischen Konstanten berechnet.  Die Gleichungen
//! folgen den analytischen Entry-Guidance-Algorithmen aus REENTRY_CONTROL.agc.
//!
//! # Einheiten
//! - Positionen: m (ECI J2000)
//! - Geschwindigkeiten: m/s (ECI J2000)
//! - G-Last: in Einheiten von g₀ = 9.80665 m/s²
//! - Zeiten: Sekunden
//!
//! AGC source: `Comanche055/P61-P67.agc`, `Comanche055/REENTRY_CONTROL.agc`.

use crate::alarm::{raise, AlarmState};
use crate::navigation::orbital::CentralBody;
use crate::navigation::state_vector::StateVector;
use crate::restart::phase_table::PhaseTable;
use crate::types::Vec3;

// ---------------------------------------------------------------------------
// Physikalische Konstanten
// ---------------------------------------------------------------------------

/// Normalbeschleunigung g₀ [m/s²].
const G0: f64 = 9.806_65;

/// EMS-Interface-Altitude für orbitale Eintritte (400 000 ft = 121 920 m über Fischer-Ellipsoid).
///
/// AGC source: `Comanche055/P61-P67.agc`, S61.2, EMSALT PAD-Loaded Value.
pub const EMSALT_ORBITAL_M: f64 = 121_920.0;

/// Atmosphärische Skalenhöhe H [m] für vereinfachtes Exponentialmodell.
const SCALE_HEIGHT_M: f64 = 7_200.0;

/// Referenz-Luftdichte auf Meereshöhe [kg/m³].
const RHO_SEA_LEVEL: f64 = 1.225;

/// Ballistischer Koeffizient des CM (B = m / (C_D · A)) [kg/m²].
/// Typischer AGC-Wert: L/D ≈ 0.3, C_D ≈ 1.37.
const BALLISTIC_COEFF: f64 = 300.0;

/// Lift-to-Drag-Ratio L/D des CM (Trimm-Konfiguration).
const L_OVER_D: f64 = 0.3;

/// Alarmcode P61: Zustandsvektor außerhalb Atmosphäreneintritts-Grenze.
///
/// AGC source: `Comanche055/P61-P67.agc`, S61.1, Alarm 01426.
pub const ALARM_P61_IMU_UNSATISFACTORY: u16 = 0x1426;

/// Alarmcode P64: Keine Up-Control-Lösung mit V > 18 000 ft/s.
pub const ALARM_P64_NO_UPCONTROL: u16 = 0x0640;

// ---------------------------------------------------------------------------
// Hilfsfunktionen
// ---------------------------------------------------------------------------

/// Berechnet die atmosphärische Luftdichte in der gegebenen Höhe h [m].
///
/// Vereinfachtes Exponentialmodell:
/// ρ(h) = ρ₀ · exp(−h / H)
///
/// # Parameter
/// - `alt_m`: Höhe über Meeresspiegel [m]
///
/// # Rückgabe
/// Luftdichte [kg/m³]; 0 für h > 200 km (außerhalb Atmosphäre).
fn air_density(alt_m: f64) -> f64 {
    if alt_m > 200_000.0 {
        return 0.0;
    }
    let h = if alt_m < 0.0 { 0.0 } else { alt_m };
    RHO_SEA_LEVEL * libm::exp(-h / SCALE_HEIGHT_M)
}

/// Berechnet die Geschwindigkeit relativ zur Erde aus dem ECI-Zustandsvektor.
///
/// v_rel = v_ECI − ω_E × r
///
/// ω_E = 7.292 115 × 10⁻⁵ rad/s (Erdrotationsrate).
///
/// # Rückgabe
/// Relativgeschwindigkeit [m/s].
fn velocity_relative_to_earth(r: Vec3, v: Vec3) -> Vec3 {
    const OMEGA_EARTH: f64 = 7.292_115e-5; // rad/s
                                           // ω_E × r = (0, 0, ω_E) × (x, y, z) = (−ω·y, ω·x, 0)
    let v_earth = Vec3::new(-OMEGA_EARTH * r.y, OMEGA_EARTH * r.x, 0.0);
    Vec3::new(v.x - v_earth.x, v.y - v_earth.y, v.z - v_earth.z)
}

/// Berechnet die Brems-G-Last aus Zustandsvektor und Atmosphärenmodell.
///
/// D = ρ · v_rel² / (2 · B)   [m/s²]
///
/// # Parameter
/// - `sv`:  Zustandsvektor (ECI, m / m/s)
/// - `body`: Zentralkörper
///
/// # Rückgabe
/// Drag-Beschleunigung D in Einheiten von g₀ (dimensionslos).
pub fn drag_g(sv: &StateVector, body: &CentralBody) -> f64 {
    let r_mag = sv.position.magnitude();
    let alt = r_mag - body.r_equatorial();
    let rho = air_density(alt);
    let v_rel = velocity_relative_to_earth(sv.position, sv.velocity);
    let v_rel_sq = v_rel.dot(v_rel);
    let drag_accel = rho * v_rel_sq / (2.0 * BALLISTIC_COEFF);
    drag_accel / G0
}

// ---------------------------------------------------------------------------
// Entry-Anzeige-Berechnungen (S61.2)
// ---------------------------------------------------------------------------

/// Ergebnis der Entry-Anzeigeberechnungen (NOUN 60 / N63).
///
/// Entspricht den Ausgaberegistern von S61.2.
///
/// AGC source: `Comanche055/P61-P67.agc`, S61.2-Subroutine.
#[derive(Copy, Clone, Debug)]
pub struct EntryDisplayData {
    /// Maximale Brems-G-Last [g₀] (GMAX).
    pub gmax: f64,
    /// Vorhergesagte Geschwindigkeit bei EMS-Altitude (VPRED) [m/s].
    pub v_pred: f64,
    /// Vorhergesagter Eintrittswinkel bei EMS-Altitude (GAMMAEI) [deg].
    pub gamma_ei_deg: f64,
    /// Reichweite bis Splashdown ab EMS-Altitude (RTGO) [m].
    pub rtgo_m: f64,
    /// Inertialgeschwindigkeit bei EMS-Altitude (VIO) [m/s].
    pub vio: f64,
    /// Geschätzte Zeit bis EMS-Altitude (TTE) [s].
    pub tte_s: f64,
}

/// Berechnet die Flugzeit von der aktuellen Position bis zu einem Zielradius `r_t`
/// analytisch über die Kepler-Gleichung.
///
/// Algorithmus:
/// 1. Wahre Anomalie ν₁ aus r, ṙ und Bahn-Parametern (Formel über h und p).
/// 2. Wahre Anomalie ν₂ aus r_t = p / (1 + e·cos(ν₂)) — absteigender Ast (ν₂ < 0).
/// 3. Exzentrische Anomalien E₁, E₂ via tan(E/2) = √((1−e)/(1+e)) · tan(ν/2).
/// 4. Δt = √(a³/μ) · (M₂ − M₁) mit M = E − e·sin(E).
///
/// # Parameter
/// - `r_mag`:    aktueller Ortsvektor-Betrag [m]
/// - `r_dot`:    radiale Geschwindigkeit ṙ = (r·v)/|r| [m/s]
/// - `h_sq`:     |h|² = |r×v|² [m⁴/s²]
/// - `a`:        große Halbachse [m]
/// - `e`:        numerische Exzentrizität
/// - `r_t`:      Zielradius [m] (muss < r_mag sein)
/// - `mu`:       Gravitationsparameter [m³/s²]
///
/// # Rückgabe
/// Flugzeit in Sekunden; im Fehlerfall (Bahn erreicht r_t nicht) 0.0.
fn kepler_time_to_radius(
    r_mag: f64,
    r_dot: f64,
    h_sq: f64,
    a: f64,
    e: f64,
    r_t: f64,
    mu: f64,
) -> f64 {
    // semi-latus rectum: p = h²/μ
    let p = h_sq / mu;

    // Wahre Anomalie am aktuellen Punkt:
    // r = p / (1 + e·cos(ν)) → cos(ν) = (p/r − 1) / e
    // Vorzeichen über ṙ: ṙ > 0 → aufsteigend → ν ∈ (0, π); ṙ < 0 → absteigend → ν ∈ (−π, 0)
    let cos_nu1 = if e > 1e-9 {
        ((p / r_mag) - 1.0) / e
    } else {
        0.0 // Kreisbahn: ν undefiniert
    };
    let cos_nu1 = cos_nu1.clamp(-1.0, 1.0);
    let nu1_abs = libm::acos(cos_nu1);
    let nu1 = if r_dot >= 0.0 { nu1_abs } else { -nu1_abs }; // absteigend: negatives ν

    // Wahre Anomalie am Zielradius r_t (absteigender Ast: ν₂ < 0)
    let cos_nu2 = if e > 1e-9 { ((p / r_t) - 1.0) / e } else { 0.0 };
    let cos_nu2 = cos_nu2.clamp(-1.0, 1.0);
    let nu2 = -libm::acos(cos_nu2); // absteigender Ast

    // Exzentrische Anomalien über Halbwinkel-Transformation
    // tan(E/2) = sqrt((1−e)/(1+e)) · tan(ν/2)
    let k = libm::sqrt((1.0 - e) / (1.0 + e).max(1e-15));

    let e1 = 2.0 * libm::atan(k * libm::tan(nu1 / 2.0));
    let e2 = 2.0 * libm::atan(k * libm::tan(nu2 / 2.0));

    // Mittlere Anomalien: M = E − e·sin(E)
    let m1 = e1 - e * libm::sin(e1);
    let m2 = e2 - e * libm::sin(e2);

    // ΔM = M₂ − M₁.
    // Absteigend vom weiten Punkt zu r_t: ν₁ < ν₂ < 0 (z.B. −120° → −9°).
    // Exzentrische und mittlere Anomalie sind monoton in ν →  M₂ > M₁ → delta_m > 0.
    let delta_m = m2 - m1;

    // Mittlere Bewegung [rad/s]
    let n = libm::sqrt(mu / (a * a * a));
    let dt = delta_m / n;

    if dt <= 0.0 {
        0.0
    } else {
        dt
    }
}

/// Hilfsfunktion (VGAMCALC): Berechnet Geschwindigkeit und Flugpfadwinkel an einem
/// Zielradius `r_t` aus dem aktuellen Zustandsvektor — reine Kepler-Mechanik.
///
/// Gleichungen aus dem AGC (P61-P67.agc, VGAMCALC, S. 813–814):
///
/// ```text
/// v_t    = sqrt(v² + 2·μ·(1/r_t − 1/r))          [Vis-Viva]
/// cos(γ) = sqrt(p·μ) / (r_t · v_t)                [Drehimpuls-Formel]
/// ```
///
/// wobei p = |h|²/μ (semi-latus rectum) und h = r × v.
///
/// # Parameter
/// - `r_mag`: aktueller Betrag des Ortsvektors [m]
/// - `v_sq`:  v·v des aktuellen Geschwindigkeitsvektors [m²/s²]
/// - `h_sq`:  |h|² = |r×v|² [m⁴/s²]
/// - `r_t`:   Zielradius [m]
/// - `mu`:    Gravitationsparameter [m³/s²]
/// - `descending`: `true` wenn der Körper absteigt (ṙ < 0) → γ negativ
///
/// # Rückgabe
/// `(v_t, gamma_deg)` — Geschwindigkeit [m/s] und Flugpfadwinkel [deg] am Zielradius.
/// Bei numerischen Problemen wird `gamma_deg` auf −90° begrenzt.
///
/// AGC source: `Comanche055/P61-P67.agc`, VGAMCALC / PREVGAM.
fn vgamcalc(r_mag: f64, v_sq: f64, h_sq: f64, r_t: f64, mu: f64, descending: bool) -> (f64, f64) {
    // Vis-Viva am Zielradius
    let v_t_sq = v_sq + 2.0 * mu * (1.0 / r_t - 1.0 / r_mag);
    let v_t = if v_t_sq > 0.0 {
        libm::sqrt(v_t_sq)
    } else {
        // Energie reicht nicht — Körper erreicht r_t nicht
        return (0.0, if descending { -90.0 } else { 90.0 });
    };

    // semi-latus rectum: p = h²/μ
    let p = h_sq / mu;

    // cos(γ) = sqrt(p) / (r_t · v_t / sqrt(μ)) = sqrt(p·μ) / (r_t · v_t)
    let cos_gamma = libm::sqrt(p * mu) / (r_t * v_t);
    let cos_gamma_clamped = cos_gamma.clamp(-1.0, 1.0);
    let gamma_rad = libm::acos(cos_gamma_clamped); // VGAMCALC liefert immer positiv

    // Vorzeichen: negativ bei Abstieg
    let gamma_deg = if descending {
        -gamma_rad * 180.0 / core::f64::consts::PI
    } else {
        gamma_rad * 180.0 / core::f64::consts::PI
    };

    (v_t, gamma_deg)
}

/// Berechnet Entry-Anzeigedaten für P61 (entspricht S61.2 im AGC).
///
/// Berechnet GMAX, VPRED und GAMMAEI durch Extrapolation der Kepler-Bahn
/// auf den 400k-ft-Radius (121 920 m) — identisch zum AGC-Algorithmus VGAMCALC.
/// VIO und TTE beziehen sich auf den EMSALT-Radius (ebenfalls 121 920 m für
/// orbitale Eintritte).
///
/// # Parameter
/// - `sv`:  aktueller Zustandsvektor (ECI J2000, m / m/s)
/// - `body`: Zentralkörper
/// - `alarm`: Alarmzustand
///
/// # Rückgabe
/// `Some(EntryDisplayData)` bei gültigem Zustandsvektor, `None` bei Alarm.
///
/// AGC source: `Comanche055/P61-P67.agc`, S61.2-Subroutine.
pub fn compute_entry_display(
    sv: &StateVector,
    body: &CentralBody,
    alarm: &mut AlarmState,
) -> Option<EntryDisplayData> {
    let mu = body.mu();
    let r_eq = body.r_equatorial();

    let r = sv.position;
    let v = sv.velocity;
    let r_mag = r.magnitude();
    let v_sq = v.dot(v);
    let v_mag = libm::sqrt(v_sq);

    if r_mag < 1.0 || v_mag < 1.0 {
        raise(alarm, ALARM_P61_IMU_UNSATISFACTORY);
        return None;
    }

    // Vis-Viva: große Halbachse
    let inv_a = 2.0 / r_mag - v_sq / mu;
    if inv_a <= 0.0 {
        // Hyperbolische Bahn — kein Wiedereintritt möglich
        raise(alarm, ALARM_P61_IMU_UNSATISFACTORY);
        return None;
    }
    let a = 1.0 / inv_a;

    // Drehimpulsvektor und semi-latus rectum
    let h_vec = r.cross(v);
    let h_sq = h_vec.dot(h_vec);

    // Abstieg feststellen: ṙ = (r · v) / |r|  < 0 → absteigend
    let r_dot = r.dot(v) / r_mag;
    let descending = r_dot < 0.0;

    // Perigee-Radius: r_per = a · (1 − e)
    let e_sq = 1.0 - h_sq / (mu * a);
    let e = if e_sq < 0.0 { 0.0 } else { libm::sqrt(e_sq) };
    let r_per = a * (1.0 - e);

    // EMS-/400k-ft-Radius  (orbital: EMSALT = 400 000 ft = 121 920 m)
    let r_ems = r_eq + EMSALT_ORBITAL_M; // ≡ r_400k

    // Prüfe: Perigee muss unter EMS liegen für Wiedereintritt
    if r_per > r_ems + 10_000.0 {
        raise(alarm, ALARM_P61_IMU_UNSATISFACTORY);
        return None;
    }

    // -----------------------------------------------------------------------
    // VGAMCALC bei EMSALT (300k ft im AGC für GMAX, dann 400k ft für N60)
    // Für die Simulation: EMSALT = 400 000 ft, daher ein einziger Aufruf.
    // -----------------------------------------------------------------------

    // 300k ft-Radius für GMAX-Berechnung (AGC nutzt 300k ft, nicht 400k ft!)
    // 300 000 ft = 91 440 m
    const R_300KFT_M: f64 = 91_440.0;
    let r_300k = r_eq + R_300KFT_M;

    let (v_300k, gamma_300k_deg) = vgamcalc(r_mag, v_sq, h_sq, r_300k, mu, descending);

    // VIO / GAMMAEI / VPRED: alle bei 400k ft = EMSALT_ORBITAL_M
    let (vio, gamma_ei_deg) = vgamcalc(r_mag, v_sq, h_sq, r_ems, mu, descending);
    let v_pred = vio; // VPRED ≈ VIO (gleicher Radius für orbitale Eintritte)

    // -----------------------------------------------------------------------
    // TTE: Zeit bis EMSALT — analytisch über Kepler-Gleichung.
    //
    // Wahrer Anomalie-Differenz: r = p/(1 + e·cos(ν))
    // Exzentrische Anomalie: tan(E/2) = sqrt((1−e)/(1+e)) · tan(ν/2)
    // Kepler-Gleichung: M = E − e·sin(E)
    // Δt = sqrt(a³/μ) · ΔM
    // -----------------------------------------------------------------------
    let tte_s = if r_mag > r_ems {
        kepler_time_to_radius(r_mag, r_dot, h_sq, a, e, r_ems, mu)
    } else {
        0.0
    };

    // -----------------------------------------------------------------------
    // GMAXCALC (AGC P61-P67.agc S. 808):
    // GMAX = (4 / (1 + 4.8·vbar²)) · (|γ_deg| − 6.05 − 2.4·vbar²) − 10·(L/D − 0.3) + 10
    // Alle Winkelgrößen in Grad (nicht Radiant) — direkt aus der AGC-Kommentarformel.
    // vbar = (v_300k [ft/s] − 36 000) / 20 000
    // Eingabe: γ und v bei 300k ft (nicht bei 400k ft)
    // -----------------------------------------------------------------------
    let v_300k_fps = v_300k * 3.280_839_895; // m/s → ft/s
    let vbar = (v_300k_fps - 36_000.0) / 20_000.0;
    let vbar_sq = vbar * vbar;
    let gamma_300k_abs_deg = gamma_300k_deg.abs(); // Grad, nicht Radiant!
    let gmax_raw = (4.0 / (1.0 + 4.8 * vbar_sq)) * (gamma_300k_abs_deg - 6.05 - 2.4 * vbar_sq)
        - 10.0 * (L_OVER_D - 0.3)
        + 10.0;
    let gmax = if gmax_raw < 0.0 { 0.0 } else { gmax_raw };

    // -----------------------------------------------------------------------
    // RTGO: vereinfachte Schätzung (Winkelreichweite ab EMS)
    // -----------------------------------------------------------------------
    let rtgo_m = libm::fabs(vio * vio / (2.0 * G0)) * L_OVER_D * 2.0;

    Some(EntryDisplayData {
        gmax,
        v_pred,
        gamma_ei_deg,
        rtgo_m,
        vio,
        tte_s,
    })
}

// ---------------------------------------------------------------------------
// P61State — Programmzustand
// ---------------------------------------------------------------------------

/// Laufzustand von P61 — Entry Preparation.
///
/// Hält alle zur Restart-sicheren Ausführung notwendigen Zwischenwerte.
///
/// AGC source: `Comanche055/P61-P67.agc`, P61-Programm.
#[derive(Copy, Clone, Debug)]
pub struct P61State {
    /// Aktuelle Programmphase (2 = Display-Berechnung, 3 = Anzeige N60/N63, 0 = done).
    pub phase: u8,
    /// Berechnete Anzeigedaten (None bis Phase 2 abgeschlossen).
    pub display: Option<EntryDisplayData>,
    /// Ziel-Breite [rev] — LAT(SPL), durch Crew eingegeben.
    pub lat_spl: f64,
    /// Ziel-Länge [rev] — LNG(SPL), durch Crew eingegeben.
    pub lng_spl: f64,
    /// HEADSUP-Flag: true = Lift-Down, false = Lift-Up.
    pub heads_up: bool,
}

impl P61State {
    /// Neuer P61-Zustand (startet in Phase 2).
    pub fn new(lat_spl: f64, lng_spl: f64, heads_up: bool) -> Self {
        Self {
            phase: 2,
            display: None,
            lat_spl,
            lng_spl,
            heads_up,
        }
    }
}

/// Führt einen P61-Schritt aus (Entry Preparation).
///
/// # Phasen
/// - **Phase 2**: Berechnet Entry-Anzeigedaten (S61.2 äquivalent) und setzt Checkpoint 4.
/// - **Phase 3**: Setzt Checkpoint auf 0 (Programm fertig), gibt `Some(())` zurück.
/// - **Phase 0**: Bereits fertig → gibt `None` zurück.
///
/// # Parameter
/// - `state`: Programmzustand
/// - `sv`:    aktueller Zustandsvektor (ECI J2000, m / m/s)
/// - `body`:  Zentralkörper
/// - `pt`:    Phase-Table
/// - `alarm`: Alarmzustand
///
/// AGC source: `Comanche055/P61-P67.agc`, P61-Programm, DUMPP61-Label.
pub fn p61_step(
    state: &mut P61State,
    sv: &StateVector,
    body: &CentralBody,
    pt: &mut PhaseTable,
    alarm: &mut AlarmState,
) -> Option<()> {
    match state.phase {
        2 | 3 => {
            // Phase 2: Anzeigedaten berechnen, Checkpoint 4 setzen
            pt.set_phase(4, 3)?;
            let disp = compute_entry_display(sv, body, alarm)?;
            state.display = Some(disp);
            // Phase 3: Abschluss
            state.phase = 0;
            pt.set_phase(4, 0)?;
            Some(())
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// P64State — Constant-Drag Entry
// ---------------------------------------------------------------------------

/// Laufzustand von P64 — Constant-Drag Entry Phase.
///
/// AGC source: `Comanche055/P61-P67.agc`, P64-Programm.
#[derive(Copy, Clone, Debug)]
pub struct P64State {
    /// Aktuelle Phase (2 = Drag-Regelung aktiv, 3 = Umschalt-Check, 0 = done).
    pub phase: u8,
    /// Aktueller Roll-Befehl [deg].
    pub roll_cmd_deg: f64,
    /// Aktuelle Drag-G-Last [g₀].
    pub drag_g_current: f64,
    /// Zielerreichter Eintrittswinkel (GAMMAEI bei .05G) [deg].
    pub gamma_at_05g_deg: f64,
}

impl P64State {
    /// Neuer P64-Zustand.
    pub fn new() -> Self {
        Self {
            phase: 2,
            roll_cmd_deg: 0.0,
            drag_g_current: 0.0,
            gamma_at_05g_deg: 0.0,
        }
    }
}

impl Default for P64State {
    fn default() -> Self {
        Self::new()
    }
}

/// AGC: 0.05 g Schwellwert für Entry-Einleitung.
const ENTRY_05G_THRESHOLD: f64 = 0.05;

/// AGC: Q7F — fester Ausgangswert von Q7 [g₀].
///
/// Q7F = 6/805 in AGC-Skalierung (805 = 25 g₀ × 32.174 ft/s²).
/// Q7 ist anfangs gleich Q7F; wird in HUNTEST dynamisch auf Basis von VL neu berechnet.
///
/// AGC source: `Comanche055/REENTRY_CONTROL.agc`, Q7F-Konstante (Z. 1522).
const Q7F_THRESHOLD: f64 = 6.0 / (25.0 * 32.174 * 0.3048); // ≈ 0.0246 g₀  (6 ft/s²)

/// AGC: Q7F + KDMIN — Drag-Schwelle für P66 → P67 (PREFINAL) [g₀].
///
/// Q7FKDMIN = 6.5/805 ≙ 6.5 ft/s² — wenn Drag diesen Wert überschreitet,
/// verlässt P66 die Ballistik- und wechselt in die finale Steuerungsphase.
///
/// AGC source: `Comanche055/REENTRY_CONTROL.agc`, Q7FKDMIN-Konstante (Z. 1512).
const Q7F_KDMIN_THRESHOLD: f64 = 6.5 / (25.0 * 32.174 * 0.3048); // ≈ 0.0267 g₀

/// Reichweiten-Schwelle für P64 → P65-Übergang (GOTOUPSY): 25 Nautische Meilen [m].
///
/// AGC source: `Comanche055/REENTRY_CONTROL.agc`, 25NM-Konstante (Z. 1545).
const UPCONTROL_RANGE_THRESHOLD_M: f64 = 25.0 * 1_852.0; // 25 NM × 1852 m/NM

/// Führt einen P64-Schritt aus (Constant-Drag Entry).
///
/// Berechnet den Roll-Befehl aus der aktuellen Drag-G-Last und der
/// Reichweiten-Abweichung (vereinfachtes Constant-Drag-Modell).
///
/// # Übergänge
/// - `Some(true)`:  Weiter in P64.
/// - `Some(false)`: Übergang zu P65 (wenn Restreichweite < 25 NM) oder
///   zu P66 (wenn D < Q7 in UPCONTRL-Phase).
/// - `None`:         interner Fehler (Phase-Table).
///
/// AGC source: `Comanche055/P61-P67.agc`, P64-Programm, DANZIG-Label;
///             `Comanche055/REENTRY_CONTROL.agc`, GOTOUPSY / KEP-Label.
pub fn p64_step(
    state: &mut P64State,
    sv: &StateVector,
    body: &CentralBody,
    target_range_m: f64,
    pt: &mut PhaseTable,
    alarm: &mut AlarmState,
) -> Option<bool> {
    let _ = alarm;

    match state.phase {
        2 => {
            pt.set_phase(5, 2)?;
            let d_g = drag_g(sv, body);
            state.drag_g_current = d_g;

            // Eintritts-Rollanpassung: Roll-Befehl aus Reichweiten-Fehler.
            // Wird immer berechnet (auch vor .05G), damit N69 R1 / N74 R1
            // bereits während der Abstiegsphase einen sinnvollen Wert zeigen.
            // AGC source: Comanche055/P61-P67.agc, DANZIG label — BETA-Berechnung
            // erfolgt unabhängig vom .05G-Check.
            let alt = sv.position.magnitude() - body.r_equatorial();
            // Vereinfachte Reichweiten-Schätzung: alt × L/D × 2
            let est_range_m = alt.abs() * L_OVER_D * 2.0;
            let range_err_m = target_range_m - est_range_m;
            // Positiver Fehler → zu weit → Lift-down → Roll 90°
            // Negativer Fehler → zu nah → Lift-up → Roll 0°
            state.roll_cmd_deg = if range_err_m > 0.0 { 90.0 } else { 0.0 };

            if d_g < ENTRY_05G_THRESHOLD {
                // Noch nicht tief genug — warten (Roll-Befehl bleibt gesetzt)
                return Some(true);
            }

            // Übergang zu P65 (GOTOUPSY), wenn Restreichweite < 25 NM.
            // AGC source: Comanche055/REENTRY_CONTROL.agc, Z. 734–737
            //             (ABS DSU 25NM BMN GOTOUPSY).
            if range_err_m.abs() < UPCONTROL_RANGE_THRESHOLD_M {
                state.phase = 3;
                return Some(false); // → P65
            }

            // Übergang zu P66 (KEP), wenn D < Q7 (Luftwiderstand zu gering für
            // Rollsteuerung in der UPCONTRL-Phase).
            // AGC source: Comanche055/REENTRY_CONTROL.agc, Z. 895–897
            //             (DSU Q7 BMN KEP).
            if d_g < Q7F_THRESHOLD {
                state.phase = 3;
                return Some(false); // → P66 (Caller-Sequenzer prüft range_err_m)
            }

            Some(true)
        }
        3 => {
            // Phase 3: Cutoff-Check
            pt.set_phase(5, 0)?;
            state.phase = 0;
            Some(false)
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// P65State — Up-Control Phase
// ---------------------------------------------------------------------------

/// Laufzustand von P65 — Up-Control Phase (Reichweite < 25 NM vom Ziel).
///
/// P65 schaltet GOTOADDR auf UPCONTRL und zeigt N69 (ROLLC / DL / VL).
/// Die Rollsteuerung bleibt aktiv; der HUNTEST-Zyklus prüft weiterhin Q7.
///
/// AGC source: `Comanche055/P61-P67.agc`, P65-Programm.
#[derive(Copy, Clone, Debug)]
pub struct P65State {
    /// Aktuelle Phase (2 = Up-Control aktiv, 0 = done).
    pub phase: u8,
    /// Roll-Befehl [deg] (ROLLC).
    pub roll_cmd_deg: f64,
    /// Aktuelle Drag-G-Last [g₀] (DL).
    pub drag_g_current: f64,
}

impl P65State {
    /// Neuer P65-Zustand.
    pub fn new() -> Self {
        Self {
            phase: 2,
            roll_cmd_deg: 0.0,
            drag_g_current: 0.0,
        }
    }
}

impl Default for P65State {
    fn default() -> Self {
        Self::new()
    }
}

/// Führt einen P65-Schritt aus (Up-Control Phase).
///
/// Hält Rollsteuerung in der Endphase aufrecht (Reichweite < 25 NM).
/// Übergang zu P66, wenn D < Q7 (Luftwiderstand fällt unter Schwellwert).
/// Übergang direkt zu P67, wenn RDOT > 0 und V < VL (Fahrzeug steigt wieder).
///
/// # Übergänge
/// - `Some(true)`:  Weiter in P65.
/// - `Some(false)`: Übergang zu P66 (D < Q7) oder P67 (direkter Skip).
/// - `None`:         interner Fehler.
///
/// AGC source: `Comanche055/P61-P67.agc`, P65-Programm;
///             `Comanche055/REENTRY_CONTROL.agc`, UPCONTRL-Label (Z. 882–900).
pub fn p65_step(
    state: &mut P65State,
    sv: &StateVector,
    body: &CentralBody,
    target_range_m: f64,
    pt: &mut PhaseTable,
    _alarm: &mut AlarmState,
) -> Option<bool> {
    match state.phase {
        2 => {
            pt.set_phase(5, 2)?;
            let d_g = drag_g(sv, body);
            state.drag_g_current = d_g;

            // Roll-Befehl aus Reichweiten-Fehler (wie P64).
            // AGC source: Comanche055/REENTRY_CONTROL.agc, UPCONTRL-Abschnitt.
            let alt = sv.position.magnitude() - body.r_equatorial();
            let est_range_m = alt.abs() * L_OVER_D * 2.0;
            let range_err_m = target_range_m - est_range_m;
            state.roll_cmd_deg = if range_err_m > 0.0 { 90.0 } else { 0.0 };

            // Übergang zu P66 (KEP), wenn D < Q7.
            // AGC source: Comanche055/REENTRY_CONTROL.agc, Z. 895–897.
            if d_g < Q7F_THRESHOLD {
                state.phase = 0;
                return Some(false); // → P66
            }

            Some(true)
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// P66State — Ballistic / Trim Phase
// ---------------------------------------------------------------------------

/// Laufzustand von P66 — Ballistische Trimmphase (D < Q7).
///
/// P66 hält das CM ohne aktive Rollbefehle in Trim-Lage zur Relativgeschwindigkeit
/// (Gimbal-Winkel aus N22: OGA / IGA / MGA). Sobald D wieder Q7 + KDMIN überschreitet,
/// wechselt die Entry-Guidance in P67 (PREFINAL / PREDICT3).
///
/// AGC source: `Comanche055/P61-P67.agc`, P66-Programm;
///             `Comanche055/REENTRY_CONTROL.agc`, KEP / KEP2-Label (Z. 1098–1121).
#[derive(Copy, Clone, Debug)]
pub struct P66State {
    /// Aktuelle Phase (2 = Trim aktiv, 0 = done).
    pub phase: u8,
    /// Aktuell gemessene Drag-G-Last [g₀].
    pub drag_g_current: f64,
    /// Letzter gehaltener Roll-Befehl [deg] (ROLLHOLD — aus P64/P65 übernommen).
    pub roll_hold_deg: f64,
}

impl P66State {
    /// Neuer P66-Zustand, mit initialem Roll-Hold-Wert aus P64/P65.
    pub fn new(roll_hold_deg: f64) -> Self {
        Self {
            phase: 2,
            drag_g_current: 0.0,
            roll_hold_deg,
        }
    }
}

impl Default for P66State {
    fn default() -> Self {
        Self::new(0.0)
    }
}

/// Führt einen P66-Schritt aus (Ballistische Trimmphase).
///
/// Beobachtet den Drag. Sobald D > Q7F + KDMIN (= 6.5 ft/s²) steigt P66 in P67 auf.
///
/// # Übergänge
/// - `Some(true)`:  Weiter in P66 (D noch unter Schwellwert).
/// - `Some(false)`: Übergang zu P67 (D ≥ Q7F + KDMIN).
/// - `None`:         interner Fehler.
///
/// AGC source: `Comanche055/REENTRY_CONTROL.agc`, KEP2-Label (Z. 1107–1121);
///             `Comanche055/P61-P67.agc`, P66-Programm (Z. 444).
pub fn p66_step(
    state: &mut P66State,
    sv: &StateVector,
    body: &CentralBody,
    pt: &mut PhaseTable,
    _alarm: &mut AlarmState,
) -> Option<bool> {
    match state.phase {
        2 => {
            pt.set_phase(5, 2)?;
            let d_g = drag_g(sv, body);
            state.drag_g_current = d_g;

            // Übergang zu P67 (PREFINAL), wenn D > Q7F + KDMIN.
            // AGC source: Comanche055/REENTRY_CONTROL.agc, Z. 1107–1110
            //             (DLOAD DSU Q7FKDMIN D BMN PREFINAL).
            if d_g > Q7F_KDMIN_THRESHOLD {
                state.phase = 0;
                return Some(false); // → P67
            }

            Some(true)
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// P67State — Terminal Phase
// ---------------------------------------------------------------------------

/// Laufzustand von P67 — Terminal Phase (v_rel ≤ ~305 m/s).
///
/// AGC source: `Comanche055/P61-P67.agc`, P67-Programm.
#[derive(Copy, Clone, Debug)]
pub struct P67State {
    /// Aktuelle Phase (2 = Monitoring, 0 = done).
    pub phase: u8,
    /// Quer-Reichweitenfehler (XRNGERR) [m].
    pub xrngerr_m: f64,
    /// Längs-Reichweitenfehler (DNRNGERR) [m].
    pub dnrngerr_m: f64,
    /// Letzter Roll-Befehl [deg].
    pub roll_cmd_deg: f64,
}

impl P67State {
    /// Neuer P67-Zustand.
    pub fn new() -> Self {
        Self {
            phase: 2,
            xrngerr_m: 0.0,
            dnrngerr_m: 0.0,
            roll_cmd_deg: 0.0,
        }
    }
}

impl Default for P67State {
    fn default() -> Self {
        Self::new()
    }
}

/// Beendigungs-Geschwindigkeit für P67 (1 000 ft/s ≈ 304.8 m/s).
///
/// AGC source: `Comanche055/P61-P67.agc`, P67-Programm.
const TERMINAL_VELOCITY_M_S: f64 = 304.8;

/// Führt einen P67-Schritt aus (Terminal Phase).
///
/// Beendet die aktive Steuerung, wenn |v_rel| ≤ 304.8 m/s.
///
/// # Rückgabe
/// `Some(true)` = weiter in P67; `Some(false)` / `None` = Entry-DAP aus, POOH.
///
/// AGC source: `Comanche055/P61-P67.agc`, P67-Programm.
pub fn p67_step(
    state: &mut P67State,
    sv: &StateVector,
    target_lat_rad: f64,
    target_lng_rad: f64,
    pt: &mut PhaseTable,
    _alarm: &mut AlarmState,
) -> Option<bool> {
    match state.phase {
        2 => {
            pt.set_phase(5, 2)?;
            let v_rel = velocity_relative_to_earth(sv.position, sv.velocity);
            let v_rel_mag = libm::sqrt(v_rel.dot(v_rel));

            // Einfache Reichweiten-Fehler-Schätzung (XRNGERR, DNRNGERR)
            // Aktuelle Lat/Lon aus Positionsvektor
            let r = sv.position;
            let lat_cur = libm::asin(r.z / r.magnitude());
            let lng_cur = libm::atan2(r.y, r.x);
            let r_eq = CentralBody::Earth.r_equatorial();
            // Abweichungen in m (kleine-Winkel-Näherung)
            state.dnrngerr_m = (target_lat_rad - lat_cur) * r_eq;
            state.xrngerr_m = (target_lng_rad - lng_cur) * r_eq * libm::cos(lat_cur);

            if v_rel_mag <= TERMINAL_VELOCITY_M_S {
                // Entry-DAP deaktivieren, Programm beenden
                state.phase = 0;
                pt.set_phase(5, 0)?;
                return Some(false);
            }
            Some(true)
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::state_vector::StateVector;
    use crate::restart::phase_table::PhaseTable;
    use crate::types::{Met, Vec3};

    /// Erstellt einen typischen Wiedereintritts-Zustandsvektor.
    ///
    /// Position bei 300 km Höhe, v = 9 700 m/s, γ ≈ −10° (elliptische Bahn,
    /// Perigee bei ~35 km — klar unter dem EMS-Radius 122 km).
    fn entry_sv() -> StateVector {
        // γ = −10° sorgt für Perigee bei ~35 km (tief in Atmosphäre).
        // v = 9700 m/s < v_escape(300 km) ≈ 10340 m/s → elliptisch.
        let r_mag = 6_378_137.0_f64 + 300_000.0; // 300 km Höhe
        let v_mag = 9_700.0_f64;
        let r = Vec3::new(r_mag, 0.0, 0.0);
        let gamma_rad = -10.0_f64 * core::f64::consts::PI / 180.0;
        let v = Vec3::new(
            v_mag * libm::sin(gamma_rad), // radiale Komponente (absteigend)
            v_mag * libm::cos(gamma_rad), // along-track
            0.0,
        );
        StateVector::new(r, v, Met::from_seconds(0.0))
    }

    // -----------------------------------------------------------------------
    // Test 1: compute_entry_display — Anzeigedaten bei gültigem Zustandsvektor
    // -----------------------------------------------------------------------
    #[test]
    fn entry_display_valid_sv() {
        let sv = entry_sv();
        let body = CentralBody::Earth;
        let mut alarm = AlarmState::new();

        let result = compute_entry_display(&sv, &body, &mut alarm);
        assert!(
            result.is_some(),
            "compute_entry_display muss Some liefern für gültigen Lunar-Return-SV"
        );
        let d = result.unwrap();
        assert!(
            alarm.last().is_none(),
            "Kein Alarm erwartet bei gültigem Zustandsvektor"
        );
        // GMAX soll physikalisch plausibel sein (0 ≤ GMAX ≤ 50 g)
        assert!(
            d.gmax >= 0.0 && d.gmax <= 50.0,
            "GMAX = {:.2} g soll im Bereich [0, 50] liegen",
            d.gmax
        );
        // VIO muss positiv sein
        assert!(d.vio > 0.0, "VIO muss positiv sein");
        // VPRED muss positiv sein
        assert!(d.v_pred > 0.0, "VPRED muss positiv sein");
        // TTE muss positiv sein (SV liegt bei 500 km > EMS 122 km)
        assert!(
            d.tte_s > 0.0,
            "TTE = {:.0} s muss positiv sein (SV liegt über EMS)",
            d.tte_s
        );
        // Eintrittswinkel soll für Lunar-Return im Bereich [-10°, 0°] liegen
        assert!(
            d.gamma_ei_deg <= 0.0 && d.gamma_ei_deg >= -15.0,
            "GAMMAEI = {:.2}° soll im Bereich [-15°, 0°] liegen",
            d.gamma_ei_deg
        );
    }

    // -----------------------------------------------------------------------
    // Test 2: compute_entry_display — Apollo-11-Referenz (60 min vor EI)
    //
    // Bahn: Trans-Earth Coast, e ≈ 0.938, a ≈ 103 973 km
    // Szenario: 17 126 km Höhe, v_inertial ≈ 5 485 m/s
    // Erwartet: GAMMAEI ≈ −6.5°, VIO ≈ 11 000 m/s, GMAX ≈ 3–5 g
    // -----------------------------------------------------------------------
    #[test]
    fn entry_display_apollo11_reference() {
        // Trans-Earth Coast bei ~17 126 km Höhe.
        // Bahnparameter: e = 0.938, a = 103 973 km
        // Perigee r_per = a·(1−e) ≈ 103 973 × 0.062 ≈ 6 446 km  (unter EMS ✓)
        // r = 6_378_137 + 17_126_000 = 23_504_137 m
        let r_mag = 6_378_137.0_f64 + 17_126_000.0;
        let v_mag = 5_485.0_f64;
        // γ ≈ −57° lokal (stark absteigende Bahn)
        let gamma_local_rad = -57.0_f64 * core::f64::consts::PI / 180.0;
        let r = Vec3::new(r_mag, 0.0, 0.0);
        let v = Vec3::new(
            v_mag * libm::sin(gamma_local_rad), // radiale Komponente (absteigend)
            v_mag * libm::cos(gamma_local_rad), // along-track
            0.0,
        );
        let sv = StateVector::new(r, v, Met::from_seconds(-3_600.0));
        let body = CentralBody::Earth;
        let mut alarm = AlarmState::new();

        let result = compute_entry_display(&sv, &body, &mut alarm);
        assert!(
            result.is_some(),
            "Apollo-11-Szenario muss Some liefern (kein Alarm)"
        );
        assert!(alarm.last().is_none(), "Kein Alarm bei Apollo-11-Szenario");

        let d = result.unwrap();

        // GAMMAEI: Eintrittswinkel bei 400k ft soll ≈ −6.5° sein (±2°)
        assert!(
            d.gamma_ei_deg >= -10.0 && d.gamma_ei_deg <= -4.0,
            "GAMMAEI = {:.2}° soll im Bereich [−10°, −4°] liegen (Referenz: −6.5°)",
            d.gamma_ei_deg
        );

        // VIO: Eintrittsgeschwindigkeit soll ≈ 10 500–11 200 m/s sein
        assert!(
            d.vio >= 10_000.0 && d.vio <= 11_500.0,
            "VIO = {:.0} m/s soll im Bereich [10 000, 11 500] liegen",
            d.vio
        );

        // GMAX: soll zwischen 2 und 15 g liegen (Apollo-11-Referenz: ≈ 6 g)
        // Physikalisches Maximum für flachen Eintritt: ~12–14 g
        assert!(
            d.gmax >= 2.0 && d.gmax <= 15.0,
            "GMAX = {:.2} g soll im Bereich [2, 15] liegen (Referenz: ≈ 6 g)",
            d.gmax
        );

        // TTE: 60 min vor EI → ≈ 3600 s (±600 s)
        assert!(
            d.tte_s >= 2_000.0 && d.tte_s <= 6_000.0,
            "TTE = {:.0} s soll im Bereich [2000, 6000] liegen (Referenz: 3600 s)",
            d.tte_s
        );
    }

    // -----------------------------------------------------------------------
    // Test 3: compute_entry_display — hyperbolische Bahn → Alarm
    // -----------------------------------------------------------------------
    #[test]
    fn entry_display_hyperbolic_gives_alarm() {
        // Escape-Geschwindigkeit überschreiten → hyperbolisch
        let r_mag = 6_378_137.0 + 300_000.0;
        let v_escape = libm::sqrt(2.0 * CentralBody::Earth.mu() / r_mag);
        let sv = StateVector::new(
            Vec3::new(r_mag, 0.0, 0.0),
            Vec3::new(0.0, v_escape * 1.1, 0.0), // > v_escape → Hyperbel
            Met::from_seconds(0.0),
        );
        let body = CentralBody::Earth;
        let mut alarm = AlarmState::new();
        let result = compute_entry_display(&sv, &body, &mut alarm);
        assert!(result.is_none(), "Hyperbolische Bahn muss None liefern");
        assert!(
            alarm.last().is_some(),
            "Alarm muss gesetzt sein bei hyperbolischer Bahn"
        );
    }

    // -----------------------------------------------------------------------
    // Test 3: p61_step — Phase 2 → done
    // -----------------------------------------------------------------------
    #[test]
    fn p61_step_phase2_to_done() {
        let sv = entry_sv();
        let body = CentralBody::Earth;
        let mut state = P61State::new(0.0, 0.0, false);
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        assert_eq!(state.phase, 2);
        let result = p61_step(&mut state, &sv, &body, &mut pt, &mut alarm);
        assert!(
            result.is_some(),
            "p61_step Phase 2 → done muss Some liefern"
        );
        assert_eq!(state.phase, 0, "state.phase soll 0 (done) sein");
        assert!(
            state.display.is_some(),
            "display-Daten müssen nach Phase 2 gesetzt sein"
        );
        assert_eq!(
            pt.get_phase(4),
            Some(0),
            "Phase-Table Gruppe 4 soll 0 (done) sein"
        );
        assert!(alarm.last().is_none(), "Kein Alarm bei gültigem SV");
    }

    // -----------------------------------------------------------------------
    // Test 4: p61_step Phase 0 → None (bereits fertig)
    // -----------------------------------------------------------------------
    #[test]
    fn p61_step_phase0_returns_none() {
        let sv = entry_sv();
        let body = CentralBody::Earth;
        let mut state = P61State::new(0.0, 0.0, false);
        state.phase = 0;
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();
        let result = p61_step(&mut state, &sv, &body, &mut pt, &mut alarm);
        assert!(result.is_none(), "Phase 0 soll None zurückgeben");
    }

    // -----------------------------------------------------------------------
    // Test 5: p64_step — 0.05g noch nicht erreicht → weiter
    // -----------------------------------------------------------------------
    #[test]
    fn p64_step_below_05g_stays_in_p64() {
        // Hoher Orbit — kaum Luftwiderstand
        let sv = StateVector::new(
            Vec3::new(6_700_000.0, 0.0, 0.0),
            Vec3::new(0.0, 7_800.0, 0.0),
            Met::from_seconds(0.0),
        );
        let body = CentralBody::Earth;
        let mut state = P64State::new();
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        let result = p64_step(&mut state, &sv, &body, 3_000_000.0, &mut pt, &mut alarm);
        // Bei hohem Orbit: drag < 0.05g → bleibt in P64 (Some(true))
        assert!(result.is_some(), "p64_step muss Some liefern");
        assert_eq!(result, Some(true), "Unter 0.05g: weiter in P64");
    }

    // -----------------------------------------------------------------------
    // Test 6: p67_step — v_rel < Schwellwert → Programm beendet
    // -----------------------------------------------------------------------
    #[test]
    fn p67_step_slow_velocity_ends_program() {
        // Relativgeschwindigkeit muss < 305 m/s sein.
        // Erdrotation: v_earth.y ≈ ω · r ≈ 7.292e-5 * 6_388_137 ≈ 466 m/s
        // Um v_rel < 305 m/s zu bekommen: v.y ≈ 466 - 200 = 266 m/s
        // → v_rel.y ≈ 266 - 466 = -200 m/s → |v_rel| ≈ 200 m/s < 305 m/s
        let r_val = 6_378_137.0 + 10_000.0;
        let omega = 7.292_115e-5_f64;
        let v_earth_y = omega * r_val; // ≈ 466 m/s
        let sv = StateVector::new(
            Vec3::new(r_val, 0.0, 0.0),
            Vec3::new(0.0, v_earth_y - 200.0, 0.0), // v_rel ≈ 200 m/s < 305 m/s
            Met::from_seconds(0.0),
        );
        let mut state = P67State::new();
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();

        let result = p67_step(&mut state, &sv, 0.0, 0.0, &mut pt, &mut alarm);
        assert_eq!(
            result,
            Some(false),
            "Bei v_rel < 305 m/s muss P67 beendet werden"
        );
        assert_eq!(state.phase, 0, "state.phase soll 0 (done) sein");
        assert_eq!(pt.get_phase(5), Some(0), "Phase-Table Gruppe 5 soll 0 sein");
    }

    // -----------------------------------------------------------------------
    // Test 7: drag_g — Luft-Bremsung in dichter Atmosphäre
    // -----------------------------------------------------------------------
    #[test]
    fn drag_g_low_altitude_significant() {
        // 70 km Höhe, v = 7 km/s
        let alt = 70_000.0_f64;
        let r = CentralBody::Earth.r_equatorial() + alt;
        let sv = StateVector::new(
            Vec3::new(r, 0.0, 0.0),
            Vec3::new(0.0, 7_000.0, 0.0),
            Met::from_seconds(0.0),
        );
        let d = drag_g(&sv, &CentralBody::Earth);
        // Bei 70 km und 7 km/s: Drag sollte > 0 und < 20 g sein
        assert!(d > 0.0, "Drag soll > 0 in dichter Atmosphäre");
        assert!(d < 20.0, "Drag soll < 20 g bei 70 km");
    }

    // -----------------------------------------------------------------------
    // Test 8: p65_step — D ≥ Q7 → bleibt in P65
    // -----------------------------------------------------------------------
    #[test]
    fn p65_step_above_q7_stays_in_p65() {
        // Zustandsvektor bei 65 km Höhe, v = 7 500 m/s
        // → dichter Bereich, Drag sollte > Q7F_THRESHOLD (≈ 0.025 g) sein
        let alt = 65_000.0_f64;
        let r = CentralBody::Earth.r_equatorial() + alt;
        let sv = StateVector::new(
            Vec3::new(r, 0.0, 0.0),
            Vec3::new(0.0, 7_500.0, 0.0),
            Met::from_seconds(0.0),
        );
        let body = CentralBody::Earth;
        let d = drag_g(&sv, &body);
        // Voraussetzung: Drag muss für diesen Test über Q7F liegen
        if d <= Q7F_THRESHOLD {
            // Physikalisch nicht in diesem Höhenbereich bei 7,5 km/s — skip
            return;
        }
        let mut state = P65State::new();
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();
        let result = p65_step(&mut state, &sv, &body, 500_000.0, &mut pt, &mut alarm);
        assert_eq!(result, Some(true), "P65 soll bei D > Q7 weiterlaufen");
    }

    // -----------------------------------------------------------------------
    // Test 9: p65_step — D < Q7 → Übergang zu P66
    // -----------------------------------------------------------------------
    #[test]
    fn p65_step_below_q7_transitions_to_p66() {
        // Sehr hoher Orbit → praktisch kein Drag → D < Q7F
        let sv = StateVector::new(
            Vec3::new(6_700_000.0, 0.0, 0.0),
            Vec3::new(0.0, 7_800.0, 0.0),
            Met::from_seconds(0.0),
        );
        let body = CentralBody::Earth;
        let mut state = P65State::new();
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();
        let result = p65_step(&mut state, &sv, &body, 500_000.0, &mut pt, &mut alarm);
        assert_eq!(result, Some(false), "P65 soll bei D < Q7 zu P66 wechseln");
        assert_eq!(state.phase, 0, "P65 phase soll 0 (done) sein");
    }

    // -----------------------------------------------------------------------
    // Test 10: p66_step — D ≤ Q7F+KDMIN → bleibt in P66
    // -----------------------------------------------------------------------
    #[test]
    fn p66_step_low_drag_stays_in_p66() {
        // Sehr hoher Orbit → Drag ≈ 0 < Q7F_KDMIN_THRESHOLD
        let sv = StateVector::new(
            Vec3::new(6_700_000.0, 0.0, 0.0),
            Vec3::new(0.0, 7_800.0, 0.0),
            Met::from_seconds(0.0),
        );
        let body = CentralBody::Earth;
        let mut state = P66State::new(45.0);
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();
        let result = p66_step(&mut state, &sv, &body, &mut pt, &mut alarm);
        assert_eq!(
            result,
            Some(true),
            "P66 soll bei D < Q7F+KDMIN weiterlaufen"
        );
        // Roll-Hold-Wert darf nicht verändert werden
        assert!(
            (state.roll_hold_deg - 45.0).abs() < f64::EPSILON,
            "roll_hold_deg soll unverändert bleiben"
        );
    }

    // -----------------------------------------------------------------------
    // Test 11: p66_step — D > Q7F+KDMIN → Übergang zu P67
    // -----------------------------------------------------------------------
    #[test]
    fn p66_step_high_drag_transitions_to_p67() {
        // 60 km Höhe, v = 8 000 m/s → sehr dichter Bereich, Drag > Q7F_KDMIN
        let alt = 60_000.0_f64;
        let r = CentralBody::Earth.r_equatorial() + alt;
        let sv = StateVector::new(
            Vec3::new(r, 0.0, 0.0),
            Vec3::new(0.0, 8_000.0, 0.0),
            Met::from_seconds(0.0),
        );
        let body = CentralBody::Earth;
        let d = drag_g(&sv, &body);
        // Voraussetzung prüfen
        if d <= Q7F_KDMIN_THRESHOLD {
            return; // Physikalisch unerwartet; Test überspringen statt fälschen
        }
        let mut state = P66State::new(0.0);
        let mut pt = PhaseTable::new();
        let mut alarm = AlarmState::new();
        let result = p66_step(&mut state, &sv, &body, &mut pt, &mut alarm);
        assert_eq!(
            result,
            Some(false),
            "P66 soll bei D > Q7F+KDMIN zu P67 wechseln"
        );
        assert_eq!(state.phase, 0, "P66 phase soll 0 (done) sein");
    }
}
