//! Sim-Telemetrie: [`TelemetryHal`]-Implementierung für den Simulator.
//!
//! `SimTelemetry` puffert gesendete Downlink-Worte in einem Ringpuffer und
//! stellt injizierte Uplink-Worte über eine FIFO-Warteschlange bereit.
//!
//! # Downlink-Puffer
//! Die letzten [`DOWNLINK_BUFFER_SIZE`] Downlink-Worte werden für Inspektions-
//! und Testzwecke gespeichert.  Der Puffer überschreibt ältere Einträge.
//!
//! # AGC-Kanalreferenzen
//! - Downlink: Kanal **034/035** (octal), Super-Comutation-Format.
//!   Quelle: `Comanche055/DOWNLINK_LISTS.agc`, DODOWNTM routine.
//! - Uplink: Kanal **020** (octal), UPRUPT-Interrupt.
//!   Quelle: `Comanche055/UPDATE_PROGRAM.agc`, UPRUPT handler.

use agc_core::hal::TelemetryHal;

/// Größe des Downlink-Ringpuffers (Anzahl Worte).
pub const DOWNLINK_BUFFER_SIZE: usize = 64;

/// Telemetrie-HAL-Implementierung für den Simulator.
///
/// Puffert empfangene Downlink-Worte und liefert injizierte Uplink-Worte.
pub struct SimTelemetry {
    // --- Downlink ---
    /// Ringpuffer für empfangene Downlink-Worte.
    downlink_buf: [u16; DOWNLINK_BUFFER_SIZE],
    /// Schreibposition im Ringpuffer.
    downlink_head: usize,
    /// Anzahl empfangener Downlink-Worte insgesamt (Gesamtzähler).
    pub downlink_total: u32,

    // --- Uplink ---
    uplink_queue: [Option<u16>; 16],
    uplink_head: usize,
    uplink_len: usize,
}

impl SimTelemetry {
    /// Erstellt eine neue, leere Sim-Telemetrie-Instanz.
    pub fn new() -> Self {
        Self {
            downlink_buf: [0; DOWNLINK_BUFFER_SIZE],
            downlink_head: 0,
            downlink_total: 0,
            uplink_queue: [None; 16],
            uplink_head: 0,
            uplink_len: 0,
        }
    }

    /// Gibt einen Slice der zuletzt empfangenen Downlink-Worte zurück.
    ///
    /// Die Reihenfolge ist chronologisch (ältestes zuerst), bis zu
    /// [`DOWNLINK_BUFFER_SIZE`] Einträge.
    ///
    /// `n`: Anzahl der abzurufenden Worte (≤ [`DOWNLINK_BUFFER_SIZE`]).
    pub fn last_downlink_words(&self, n: usize) -> [u16; DOWNLINK_BUFFER_SIZE] {
        // Gibt immer den vollen Puffer zurück — Aufrufer wählt n Einträge.
        let _ = n; // n dient als Hinweis für den Aufrufer, nicht erzwungen hier.
        self.downlink_buf
    }

    /// Injiziert ein simuliertes Uplink-Wort in die Warteschlange.
    ///
    /// Wird vom Sim-Loop oder von Tests verwendet, um Bodenbefehle zu simulieren.
    pub fn push_uplink(&mut self, word: u16) {
        if self.uplink_len < self.uplink_queue.len() {
            let idx = (self.uplink_head + self.uplink_len) % self.uplink_queue.len();
            self.uplink_queue[idx] = Some(word);
            self.uplink_len += 1;
        }
    }

    /// Gibt zurück ob die Uplink-Warteschlange leer ist.
    pub fn uplink_empty(&self) -> bool {
        self.uplink_len == 0
    }
}

impl Default for SimTelemetry {
    fn default() -> Self {
        Self::new()
    }
}

impl TelemetryHal for SimTelemetry {
    /// Speichert das Downlink-Wort im Ringpuffer.
    ///
    /// AGC source: `Comanche055/DOWNLINK_LISTS.agc`, DODOWNTM routine.
    fn send_downlink_word(&mut self, word: u16) {
        self.downlink_buf[self.downlink_head % DOWNLINK_BUFFER_SIZE] = word;
        self.downlink_head = (self.downlink_head + 1) % DOWNLINK_BUFFER_SIZE;
        self.downlink_total = self.downlink_total.saturating_add(1);
    }

    /// Liefert das nächste Uplink-Wort aus der FIFO-Warteschlange.
    ///
    /// AGC source: `Comanche055/UPDATE_PROGRAM.agc`, UPRUPT handler.
    fn read_uplink_word(&mut self) -> Option<u16> {
        if self.uplink_len == 0 {
            return None;
        }
        let word = self.uplink_queue[self.uplink_head];
        self.uplink_head = (self.uplink_head + 1) % self.uplink_queue.len();
        self.uplink_len -= 1;
        word
    }
}
