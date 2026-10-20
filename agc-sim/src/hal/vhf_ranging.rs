//! Sim-VHF-Ranging: [`VhfRangingHal`]-Implementierung für den Simulator.
//!
//! Verbindet `agc-physics::VhfRanging` mit dem `VhfRangingHal`-Trait.
//!
//! Der Simulator übergibt CM- und LM-Position nach jedem Trajektorienintegrations-
//! schritt an [`SimVhfRanging::tick`].  `agc-core` ruft dann bei Bedarf
//! [`VhfRangingHal::read_range`] ab.
//!
//! Das VHF-Ranging ist optional: [`VhfRangingHal::is_available`] gibt `false`
//! zurück wenn `SimVhfRanging` als nicht aktiv konfiguriert ist oder wenn die
//! letzte Messung außerhalb der Reichweite lag.
//!
//! # AGC-Kanalreferenzen
//! - Ranging-Anforderung: Kanal **034** (octal) Bit 4 (VHFRQST).
//! - Ranging-Messung: Kanal **034** Bits 1–12 (VHFRANGE).
//!   Quelle: `Comanche055/R31.agc`, RANGERD routine.
//!
//! # Skalierung
//! Der AGC-Ranging-Wert ist ein 12-Bit-Wort, 1 LSB ≈ 0.488 m.
//! Konversion: `agc_word = range_m / 0.488`.

use agc_core::hal::VhfRangingHal;
use agc_physics::math::Vec3;
use agc_physics::vhf_ranging::VhfRanging;

/// Skalierungsfaktor: Meter pro AGC-Ranging-LSB.
///
/// 1 LSB ≈ 0.488 m (Vollskala 2000 NM ÷ 2¹² Schritte).
///
/// AGC source: `Comanche055/R31.agc`, RANGERD routine.
const RANGE_M_PER_LSB: f64 = 0.488;

/// VHF-Ranging-HAL-Implementierung für den Simulator.
///
/// # Verwendung im Sim-Loop
/// ```rust,ignore
/// let mut ranging = SimVhfRanging::new(42, true);
/// // Nach jedem Trajektorie-Schritt:
/// ranging.tick(cm_pos, lm_pos);
/// // agc-core liest read_range() bei V87/V88-Anfragen.
/// ```
pub struct SimVhfRanging {
    physics: VhfRanging,
    /// `true` wenn das VHF-Ranging-System aktiv ist.
    active: bool,
    /// Zuletzt berechneter AGC-Ranging-Wert (oder `None` außer Reichweite).
    last_range: Option<u16>,
    /// Pending-Request-Flag: `agc-core` hat eine Messung angefordert.
    request_pending: bool,
}

impl SimVhfRanging {
    /// Erstellt ein neues Sim-VHF-Ranging.
    ///
    /// # Parameter
    /// - `seed`:   Zufallssaat für das Messrauschen.
    /// - `active`: `true` wenn das System eingeschaltet sein soll.
    pub fn new(seed: u32, active: bool) -> Self {
        Self {
            physics: VhfRanging::new(seed),
            active,
            last_range: None,
            request_pending: false,
        }
    }

    /// Erstellt ein deterministisches Sim-VHF-Ranging ohne Rauschen (für Tests).
    pub fn noiseless(active: bool) -> Self {
        Self {
            physics: VhfRanging::noiseless(),
            active,
            last_range: None,
            request_pending: false,
        }
    }

    /// Simulationsschritt: Berechnet die CM-LM-Distanz und speichert den AGC-Wert.
    ///
    /// Muss nach jedem Trajektorie-Integrationsschritt aufgerufen werden,
    /// bevor `agc-core` [`read_range`][VhfRangingHal::read_range] aufruft.
    ///
    /// # Parameter
    /// - `cm_pos`: CM-Position in ECI (m)
    /// - `lm_pos`: LM-Position in ECI (m)
    pub fn tick(&mut self, cm_pos: Vec3, lm_pos: Vec3) {
        if !self.active {
            self.last_range = None;
            return;
        }

        self.last_range = self.physics.measure(cm_pos, lm_pos).map(|m| {
            // Konvertiere Meter in AGC-Ranging-LSB, gesättigter auf u16.
            let lsb = (m.range_m / RANGE_M_PER_LSB).round() as u64;
            lsb.min(u16::MAX as u64) as u16
        });
    }

    /// Setzt den Aktiv-Status (z. B. wenn V87 gesetzt/gelöscht wird).
    pub fn set_active(&mut self, active: bool) {
        self.active = active;
        if !active {
            self.last_range = None;
        }
    }
}

impl VhfRangingHal for SimVhfRanging {
    /// Gibt zurück ob das VHF-Ranging verfügbar ist.
    ///
    /// Liefert `true` wenn aktiv und die letzte Messung innerhalb der Reichweite war.
    fn is_available(&self) -> bool {
        self.active && self.last_range.is_some()
    }

    /// Merkt eine Ranging-Anfrage für den nächsten Sim-Tick vor.
    ///
    /// AGC source: `Comanche055/R31.agc`, RANGERD routine.
    fn request_range(&mut self) {
        self.request_pending = true;
    }

    /// Liest den zuletzt berechneten AGC-Ranging-Wert.
    ///
    /// AGC source: `Comanche055/R31.agc`, RANGERD routine.
    fn read_range(&self) -> Option<u16> {
        self.last_range
    }
}
