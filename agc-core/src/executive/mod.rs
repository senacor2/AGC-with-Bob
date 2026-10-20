//! Executive & Waitlist — kooperativer Scheduler des AGC.
//!
//! # Überblick
//!
//! Der **Executive** ist der kooperative Job-Scheduler des Apollo Guidance
//! Computer. Er verwaltet bis zu 8 gleichzeitig aktive Jobs nach Priorität
//! (0–31) und einen Idle-Zustand wenn keine Jobs vorliegen.
//!
//! Die **Waitlist** ist die zeitgesteuerte Ergänzung: bis zu 9 Tasks können
//! mit einer Verzögerung in Zentisekunden eingetragen und werden über
//! [`Waitlist::tick`] ausgelöst (entspricht T3RUPT im Original-AGC).
//!
//! ## Typischer Ablauf
//!
//! ```rust,ignore
//! use agc_core::executive::{Executive, Waitlist};
//!
//! let mut exec = Executive::new();
//! let mut wl   = Waitlist::new();
//!
//! // Job einplanen
//! exec.schedule(10, || { /* guidance step */ }).unwrap();
//!
//! // Task verzögert einplanen (in 50 cs = 500 ms)
//! wl.call_after(50, || { /* delayed task */ }).unwrap();
//!
//! // Scheduler-Schleife (entspricht ENDPRCHG/ADVAN im AGC)
//! loop {
//!     // T3RUPT-Äquivalent — je nach Timer-ISR
//!     wl.tick(1);
//!     // Höchst-priorisierten Job ausführen
//!     exec.run_highest();
//! }
//! ```
//!
//! # Architektur-Hinweise
//!
//! - Kein Heap: alle Datenstrukturen sind statisch groß.
//! - Kein Blocking: Jobs und Tasks müssen kooperativ enden.
//! - In der eingebetteten Zielumgebung (Cortex-M7) werden `Executive` und
//!   `Waitlist` in einem `cortex_m::interrupt::Mutex<RefCell<T>>` gehalten —
//!   Zugriff ausschließlich über `cortex_m::interrupt::free(|cs| ...)`.
//!
//! # AGC-Quelle
//! `Comanche055/EXECUTIVE.agc` und `Comanche055/WAITLIST.agc` (pp. 1208–1235).

pub mod job_table;
pub mod waitlist;

pub use job_table::{JobEntry, JobTable, MAX_JOBS, MAX_PRIORITY};
pub use waitlist::{WaitEntry, Waitlist, MAX_DELAY_CS, MAX_TASKS};

/// Kooperativer Job-Scheduler (Executive).
///
/// Hülltyp um [`JobTable`], der die öffentliche API des Executives bereitstellt.
///
/// # AGC-Quelle
/// `Comanche055/EXECUTIVE.agc`, `NOVAC`/`FINDVAC`/`ENDOFJOB` routines (pp. 1208–1220).
#[derive(Debug)]
pub struct Executive {
    table: JobTable,
}

impl Executive {
    /// Erstellt einen leeren Executive (keine aktiven Jobs).
    pub const fn new() -> Self {
        Self {
            table: JobTable::new(),
        }
    }

    /// Plant einen Job mit der angegebenen Priorität ein.
    ///
    /// - `priority`: 1–31. 0 wird auf 1 aufgerundet, >31 auf 31 begrenzt.
    /// - `job_fn`: auszuführende Funktion (keine Argumente, kein Rückgabewert).
    ///
    /// Gibt `Some(slot_index)` bei Erfolg, `None` wenn die Job-Tabelle voll ist.
    ///
    /// Entspricht `NOVAC`/`FINDVAC` im Original-AGC.
    ///
    /// AGC source: `Comanche055/EXECUTIVE.agc`, `NOVAC`/`FINDVAC` routines (p. 1208).
    pub fn schedule(&mut self, priority: u8, job_fn: fn()) -> Option<usize> {
        self.table.schedule(priority, job_fn)
    }

    /// Führt den Job mit der höchsten Priorität aus und entfernt ihn.
    ///
    /// Gibt `true` zurück, wenn ein Job ausgeführt wurde; `false` im Idle-Zustand.
    ///
    /// Entspricht `ENDPRCHG`/`DUMMYJOB`/`ADVAN` im Original-AGC.
    ///
    /// AGC source: `Comanche055/EXECUTIVE.agc`, `ENDPRCHG`/`DUMMYJOB` routines (pp. 1214–1220).
    pub fn run_highest(&mut self) -> bool {
        self.table.run_highest()
    }

    /// Anzahl aktuell aktiver Jobs.
    pub fn active_count(&self) -> usize {
        self.table.active_count()
    }

    /// Gibt Zugriff auf die interne Job-Tabelle (für erweiterte Nutzung).
    pub fn table(&self) -> &JobTable {
        &self.table
    }
}

impl Default for Executive {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Integrationstests (Executive + Waitlist zusammen)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{Executive, Waitlist};
    use core::sync::atomic::{AtomicU32, Ordering};

    static ORDER: AtomicU32 = AtomicU32::new(0);
    static LAST_ORDER: AtomicU32 = AtomicU32::new(0);

    fn high_prio_job() {
        let seq = ORDER.fetch_add(1, Ordering::Relaxed);
        LAST_ORDER.store(seq, Ordering::Relaxed);
    }

    fn low_prio_job() {
        let seq = ORDER.fetch_add(1, Ordering::Relaxed);
        LAST_ORDER.store(seq, Ordering::Relaxed);
    }

    static WL_FIRED: AtomicU32 = AtomicU32::new(0);

    fn wl_task() {
        WL_FIRED.fetch_add(1, Ordering::Relaxed);
    }

    #[test]
    fn exec_runs_higher_priority_first() {
        ORDER.store(0, Ordering::Relaxed);
        let mut exec = Executive::new();

        // Niedrigere Priorität zuerst eintragen
        exec.schedule(5, low_prio_job).unwrap();
        exec.schedule(20, high_prio_job).unwrap();

        // Erster run_highest soll high_prio_job ausführen
        assert!(exec.run_highest()); // prio 20 → seq 0
        assert!(exec.run_highest()); // prio 5  → seq 1
        assert!(!exec.run_highest()); // leer
    }

    #[test]
    fn exec_idle_when_empty() {
        let mut exec = Executive::new();
        assert!(!exec.run_highest());
        assert_eq!(exec.active_count(), 0);
    }

    #[test]
    fn waitlist_fires_into_executive() {
        WL_FIRED.store(0, Ordering::Relaxed);
        let mut wl = Waitlist::new();

        wl.call_after(3, wl_task).unwrap();

        wl.tick(2);
        assert_eq!(WL_FIRED.load(Ordering::Relaxed), 0);

        wl.tick(1);
        assert_eq!(WL_FIRED.load(Ordering::Relaxed), 1);

        assert_eq!(wl.count(), 0);
    }

    #[test]
    fn exec_schedule_and_remove_cycle() {
        let mut exec = Executive::new();
        exec.schedule(10, high_prio_job).unwrap();
        assert_eq!(exec.active_count(), 1);
        exec.run_highest();
        assert_eq!(exec.active_count(), 0);
    }
}
