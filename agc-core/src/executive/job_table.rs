//! Job-Tabelle des Executive-Schedulers.
//!
//! # Konzept
//!
//! Der AGC-Executive ist ein **kooperativer Scheduler ohne Preemption**.
//! Jobs melden sich mit einer Priorität (0–31) an und laufen bis zum freiwilligen
//! Ende (`ENDOFJOB`). Ein Job darf nicht blockieren.
//!
//! Diese Implementierung hält maximal [`MAX_JOBS`] gleichzeitig aktive Jobs in
//! einer statisch großen, heap-freien Tabelle. Die Tabelle ist nach Priorität
//! sortiert; [`run_highest`] startet stets den dringlichsten wartenden Job.
//!
//! ## Prioritäten
//!
//! | Wert | Bedeutung                         |
//! |------|-----------------------------------|
//! | 0    | niedrigste Priorität (Idle)       |
//! | 31   | höchste Priorität                 |
//!
//! ## Verweis auf den AGC-Original-Scheduler
//!
//! Im Original finden sich bis zu 7 „Core Sets" (NOVAC/FINDVAC); hier verwenden
//! wir 8 Slots, um einen kleinen Puffer für kurzzeitige Überlastspitzen zu haben.
//!
//! # AGC-Quelle
//! `Comanche055/EXECUTIVE.agc`, `NOVAC`/`FINDVAC`/`ENDJOB1`/`EJSCAN` routines (pp. 1208–1220).

/// Maximale Anzahl gleichzeitig aktiver Jobs.
///
/// Der originale AGC hatte 7 Core Sets. Wir verwenden 8, um dem Verhalten von
/// `NO.CORES DEC 6` (6 zusätzliche Sets neben Core Set 0) exakt zu entsprechen
/// und einen Reserveslot für den Idle-Job zu behalten.
///
/// AGC source: `Comanche055/EXECUTIVE.agc`, `NO.CORES DEC 6` (p. 1212).
pub const MAX_JOBS: usize = 8;

/// Höchste erlaubte Priorität (entspricht dem AGC-Prioritätsbereich 0–31).
///
/// AGC source: `Comanche055/EXECUTIVE.agc`, Prioritäten 0–31 (Kommentar p. 1211).
pub const MAX_PRIORITY: u8 = 31;

/// Ein einzelner Eintrag in der Job-Tabelle.
///
/// Enthält die Funktion (`job_fn`) und die Priorität des Jobs.
/// Ein freier Slot wird durch `priority == 0` **und** `job_fn == None` markiert.
///
/// AGC source: `Comanche055/EXECUTIVE.agc`, Core-Set-Struktur (pp. 1211–1212).
#[derive(Clone, Copy)]
pub struct JobEntry {
    /// Jobfunktion. `None` bedeutet: Slot frei.
    pub job_fn: Option<fn()>,
    /// Priorität 0–31.  0 = inaktiv/leer.
    pub priority: u8,
}

impl JobEntry {
    /// Leerer (freier) Slot.
    pub const fn empty() -> Self {
        Self {
            job_fn: None,
            priority: 0,
        }
    }

    /// Gibt `true` zurück, wenn der Slot belegt ist.
    pub fn is_active(&self) -> bool {
        self.job_fn.is_some()
    }
}

impl Default for JobEntry {
    fn default() -> Self {
        Self::empty()
    }
}

/// Statische Job-Tabelle des Executive.
///
/// Hält bis zu [`MAX_JOBS`] aktive Jobs. Die Tabelle ist **nicht** automatisch
/// nach Priorität sortiert; [`JobTable::highest`] scannt linear — das ist auf
/// 8 Slots O(n) und damit für den eingebetteten Einsatz hinreichend.
///
/// # Keine Heap-Allokation
///
/// Alle Slots sind statisch-groß (`[JobEntry; MAX_JOBS]`). Kein `Vec`, kein `Box`.
///
/// AGC source: `Comanche055/EXECUTIVE.agc`, `NOVAC2`/`EJSCAN` routines (pp. 1212–1219).
pub struct JobTable {
    slots: [JobEntry; MAX_JOBS],
}

impl JobTable {
    /// Leere Tabelle (alle Slots frei).
    pub const fn new() -> Self {
        Self {
            slots: [JobEntry::empty(); MAX_JOBS],
        }
    }

    /// Trägt einen neuen Job ein.
    ///
    /// - `priority`: 1–31 (0 ist reserviert für „leer"). Werte > [`MAX_PRIORITY`]
    ///   werden auf [`MAX_PRIORITY`] begrenzt.
    /// - `job_fn`: auszuführende Funktion.
    ///
    /// Gibt `Some(slot_index)` zurück bei Erfolg, `None` wenn die Tabelle voll ist.
    ///
    /// AGC source: `Comanche055/EXECUTIVE.agc`, `NOVAC`/`NOVAC2`/`CORFOUND` routines.
    pub fn schedule(&mut self, priority: u8, job_fn: fn()) -> Option<usize> {
        let priority = priority.clamp(1, MAX_PRIORITY);
        for (i, slot) in self.slots.iter_mut().enumerate() {
            if !slot.is_active() {
                *slot = JobEntry {
                    job_fn: Some(job_fn),
                    priority,
                };
                return Some(i);
            }
        }
        None
    }

    /// Gibt den Index des Jobs mit der höchsten Priorität zurück.
    ///
    /// Bei Gleichstand gewinnt der Job mit dem **kleineren** Index (FIFO-Reihenfolge
    /// innerhalb einer Prioritätsstufe — entspricht dem AGC-Verhalten).
    ///
    /// Gibt `None` zurück, wenn kein aktiver Job vorhanden ist.
    ///
    /// AGC source: `Comanche055/EXECUTIVE.agc`, `EJSCAN`/`EJ1`/`EJ2` routines (pp. 1218–1219).
    pub fn highest(&self) -> Option<usize> {
        let mut best_idx: Option<usize> = None;
        let mut best_prio: u8 = 0;
        for (i, slot) in self.slots.iter().enumerate() {
            if slot.is_active() && slot.priority > best_prio {
                best_prio = slot.priority;
                best_idx = Some(i);
            }
        }
        best_idx
    }

    /// Entfernt den Job am angegebenen Slot-Index (ENDOFJOB).
    ///
    /// Gibt `true` zurück, wenn der Slot belegt war; `false` bei ungültigem Index
    /// oder leerem Slot.
    ///
    /// AGC source: `Comanche055/EXECUTIVE.agc`, `ENDOFJOB`/`ENDJOB1` routine (p. 1218).
    pub fn remove(&mut self, index: usize) -> bool {
        if index >= MAX_JOBS {
            return false;
        }
        if self.slots[index].is_active() {
            self.slots[index] = JobEntry::empty();
            true
        } else {
            false
        }
    }

    /// Führt den Job mit der höchsten Priorität aus und entfernt ihn danach.
    ///
    /// Gibt `true` zurück, wenn ein Job ausgeführt wurde; `false` wenn die Tabelle
    /// leer ist (Idle-Zustand).
    ///
    /// AGC source: `Comanche055/EXECUTIVE.agc`, `ENDPRCHG`/`DUMMYJOB` routines (pp. 1214–1220).
    pub fn run_highest(&mut self) -> bool {
        match self.highest() {
            None => false,
            Some(idx) => {
                // Job-Funktion extrahieren, Slot freigeben, dann aufrufen.
                // Die Reihenfolge (erst freigeben, dann aufrufen) spiegelt das
                // AGC-Verhalten wider: der Slot ist bereits frei, wenn der Job
                // selbst neue Jobs einplant.
                let f = self.slots[idx].job_fn;
                self.slots[idx] = JobEntry::empty();
                if let Some(job) = f {
                    job();
                }
                true
            }
        }
    }

    /// Anzahl aktuell aktiver Jobs.
    pub fn active_count(&self) -> usize {
        self.slots.iter().filter(|s| s.is_active()).count()
    }
}

impl Default for JobTable {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{JobTable, MAX_JOBS, MAX_PRIORITY};

    fn noop() {}
    fn job_a() {}
    fn job_b() {}
    fn job_c() {}

    #[test]
    fn new_table_is_empty() {
        let t = JobTable::new();
        assert_eq!(t.active_count(), 0);
        assert!(t.highest().is_none());
    }

    #[test]
    fn schedule_one_job() {
        let mut t = JobTable::new();
        let idx = t.schedule(10, noop);
        assert!(idx.is_some());
        assert_eq!(t.active_count(), 1);
    }

    #[test]
    fn highest_returns_max_priority_job() {
        let mut t = JobTable::new();
        t.schedule(5, job_a).unwrap();
        t.schedule(20, job_b).unwrap();
        t.schedule(10, job_c).unwrap();
        let hi = t.highest().unwrap();
        assert_eq!(t.slots[hi].priority, 20);
    }

    #[test]
    fn priority_clamped_to_max() {
        let mut t = JobTable::new();
        let idx = t.schedule(200, noop).unwrap();
        assert_eq!(t.slots[idx].priority, MAX_PRIORITY);
    }

    #[test]
    fn priority_zero_clamped_to_one() {
        let mut t = JobTable::new();
        let idx = t.schedule(0, noop).unwrap();
        assert_eq!(t.slots[idx].priority, 1);
    }

    #[test]
    fn run_highest_executes_and_removes() {
        let mut t = JobTable::new();
        t.schedule(5, job_a).unwrap();
        t.schedule(15, job_b).unwrap();
        assert!(t.run_highest());
        // Nach run_highest: Job mit prio=15 wurde ausgeführt und entfernt
        assert_eq!(t.active_count(), 1);
        assert_eq!(t.slots[t.highest().unwrap()].priority, 5);
    }

    #[test]
    fn run_highest_on_empty_returns_false() {
        let mut t = JobTable::new();
        assert!(!t.run_highest());
    }

    #[test]
    fn schedule_fills_all_slots() {
        let mut t = JobTable::new();
        for _ in 0..MAX_JOBS {
            assert!(t.schedule(1, noop).is_some());
        }
        // Tabelle voll — weiteres schedule muss None zurückgeben
        assert!(t.schedule(1, noop).is_none());
    }

    #[test]
    fn remove_frees_slot_for_reuse() {
        let mut t = JobTable::new();
        let idx = t.schedule(7, noop).unwrap();
        assert!(t.remove(idx));
        assert_eq!(t.active_count(), 0);
        // Slot kann wieder belegt werden
        assert!(t.schedule(7, noop).is_some());
    }

    #[test]
    fn fifo_tiebreak_within_same_priority() {
        let mut t = JobTable::new();
        // Drei Jobs mit gleicher Priorität — erster eingetragener gewinnt
        let i0 = t.schedule(10, job_a).unwrap();
        let _i1 = t.schedule(10, job_b).unwrap();
        let _i2 = t.schedule(10, job_c).unwrap();
        assert_eq!(t.highest().unwrap(), i0);
    }
}
