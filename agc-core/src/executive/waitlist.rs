//! Zeitgesteuerte Task-Liste (Waitlist).
//!
//! # Konzept
//!
//! Die Waitlist verwaltet bis zu [`MAX_TASKS`] zeitlich verzögerte Tasks in
//! Zentisekunden. Im Original-AGC wird sie über den **T3RUPT** (Timer 3 Interrupt)
//! getaktet; jeder Interrupt entspricht einem Tick.
//!
//! ## Datenstruktur (analog AGC LST1/LST2)
//!
//! | AGC-Register | Rust-Äquivalent          | Inhalt                          |
//! |--------------|--------------------------|----------------------------------|
//! | `LST1[0..7]` | `delays[0..7]`           | Delta-Zeit in Zentisekunden bis zum nächsten Task (≥ 1) |
//! | `LST2[0..8]` | `tasks[0..8].task_fn`    | Funktionszeiger des Tasks        |
//! | `TIME3`      | `time_to_first`          | Verbleibende Zeit bis Task 0 |
//!
//! Die Liste ist **sortiert nach Ausführungszeit** (nächster Task an Index 0).
//! `delays[i]` speichert die Zeitdifferenz zwischen Task i und Task i+1, nicht
//! die absolute Zeit — identisch mit dem AGC-LST1-Format.
//!
//! ## Tick-Semantik
//!
//! [`Waitlist::tick`] muss mit der T3RUPT-Rate aufgerufen werden (typisch: 1 cs
//! pro Aufruf). Wenn `time_to_first` auf ≤ 0 fällt, wird Task 0 ausgeführt und
//! die Liste um eine Stelle verschoben. Ist `delays[0]` ebenfalls 0, werden
//! weitere Tasks direkt gefeuert (entspricht `RUPTAGN`-Verhalten im AGC).
//!
//! Für Tasks die auf den [`crate::context::AgcContext`] zugreifen müssen, steht
//! die freie Funktion [`tick_with_ctx`] bereit (verwendet `core::mem::take`-Pattern
//! um Borrow-Konflikte zu vermeiden).
//!
//! ## Einschränkungen
//!
//! - Kein Heap: feste Arrays mit `MAX_TASKS`-Einträgen.
//! - Kein Blocking: Tasks laufen synchron im Tick-Kontext.
//! - `call_after(0, ...)` wird auf 1 Zentisekunde aufgerundet (entspricht AGC
//!   Alarm 1204: „WAITLIST CALL WITH ZERO OR NEG DT").
//!
//! # AGC-Quelle
//! `Comanche055/WAITLIST.agc`, `WAITLIST`/`T3RUPT`/`TASKOVER` routines (pp. 1221–1235).

/// Maximale Anzahl gleichzeitig wartender Tasks.
///
/// AGC source: `Comanche055/WAITLIST.agc`, Kommentar „9 TASKS MAXIMUM" (p. 1221).
pub const MAX_TASKS: usize = 9;

/// Maximale erlaubte Delta-Zeit in Zentisekunden.
///
/// Entspricht DTMAX = 16250 cs = 162,5 s aus dem Waitlist-Kommentar.
///
/// AGC source: `Comanche055/WAITLIST.agc`, Kommentar „1 <= C(A) <= 16250D" (p. 1221).
pub const MAX_DELAY_CS: u32 = 16_250;

/// Ein einzelner Warteeintrag.
#[derive(Clone, Copy, Debug)]
pub struct WaitEntry {
    /// Funktionszeiger. `None` = Slot leer.
    pub task_fn: Option<fn(&mut crate::context::AgcContext)>,
}

impl WaitEntry {
    /// Leerer Slot.
    pub const fn empty() -> Self {
        Self { task_fn: None }
    }

    /// Gibt `true` zurück, wenn der Slot belegt ist.
    pub fn is_active(&self) -> bool {
        self.task_fn.is_some()
    }
}

impl Default for WaitEntry {
    fn default() -> Self {
        Self::empty()
    }
}

/// Zeitgesteuerte Warteschlange — analog zu `LST1`/`LST2` im AGC.
///
/// # Felder
///
/// - `count`: Anzahl der aktuell eingetragenen Tasks (0..=`MAX_TASKS`).
/// - `time_to_first`: verbleibende Zentisekunden bis zur Ausführung von `tasks[0]`.
/// - `delays[i]`: Zeitdifferenz in cs zwischen Task i und Task i+1.
///   Für `i >= count - 1` undefiniert.
/// - `tasks[i]`: Task an Position i der sortierten Liste.
///
/// AGC source: `Comanche055/WAITLIST.agc`, `LST1`/`LST2` memory layout (pp. 1221–1222).
#[derive(Debug)]
pub struct Waitlist {
    /// Anzahl aktiver Einträge.
    count: usize,
    /// Zentisekunden bis zum nächsten Feuern (TIME3-Äquivalent).
    time_to_first: u32,
    /// Zeitdifferenzen zwischen benachbarten Tasks (LST1).
    delays: [u32; MAX_TASKS],
    /// Task-Funktionen (LST2).
    tasks: [WaitEntry; MAX_TASKS],
}

impl Waitlist {
    /// Leere Waitlist.
    pub const fn new() -> Self {
        Self {
            count: 0,
            time_to_first: 0,
            delays: [0; MAX_TASKS],
            tasks: [WaitEntry::empty(); MAX_TASKS],
        }
    }

    /// Trägt einen neuen Task ein, der nach `delay_cs` Zentisekunden ausgeführt
    /// werden soll.
    ///
    /// - `delay_cs`: 1–[`MAX_DELAY_CS`]. Wert 0 wird auf 1 aufgerundet.
    ///   Werte > [`MAX_DELAY_CS`] werden auf [`MAX_DELAY_CS`] begrenzt.
    /// - `task_fn`: aufzurufende Funktion — erhält `&mut AgcContext`.
    ///
    /// Gibt `Some(())` bei Erfolg, `None` wenn die Warteschlange voll ist.
    ///
    /// Die interne Darstellung verwendet **Differenz-Zeiten** (LST1-Format):
    /// - `time_to_first` = absolute Zeit bis Task 0
    /// - `delays[i]` = Delta zwischen Task i und Task i+1 (für i < count−1)
    ///
    /// AGC source: `Comanche055/WAITLIST.agc`, `WAITLIST`/`WTLST2`/`WTLST4`/`WTLST5` routines.
    pub fn call_after(
        &mut self,
        delay_cs: u32,
        task_fn: fn(&mut crate::context::AgcContext),
    ) -> Option<()> {
        if self.count >= MAX_TASKS {
            return None;
        }

        // Delay-Sanitierung (entspricht AGC Alarm 1204 / DTMAX-Begrenzung)
        let delay_cs = delay_cs.clamp(1, MAX_DELAY_CS);

        let entry = WaitEntry {
            task_fn: Some(task_fn),
        };

        if self.count == 0 {
            // Erste Eintragung: direkt als TIME3-Äquivalent setzen.
            self.time_to_first = delay_cs;
            self.tasks[0] = entry;
            self.count = 1;
            return Some(());
        }

        // Einfüge-Position bestimmen (sortiert nach absoluter Zeit).
        // abs_times[i] = absolute Ausführungszeit von Task i.
        // Wir suchen das erste i, für das delay_cs <= abs_times[i].
        let mut abs_time: u32 = self.time_to_first;
        let mut insert_at: usize = self.count; // default: ans Ende

        for i in 0..self.count {
            if delay_cs <= abs_time {
                insert_at = i;
                break;
            }
            abs_time = abs_time.saturating_add(self.delays[i]);
        }

        // abs_at_insert = absolute Zeit des Tasks, der nach der Einfügung an
        // Position insert_at steht (falls insert_at < self.count, sonst irrelevant).
        // abs_time hat nach der Schleife den Wert der abs. Zeit von insert_at
        // (oder des Endes, wenn insert_at == count).
        let abs_at_insert = abs_time;

        // Verschieben (ab insert_at nach rechts, bevor count erhöht wird)
        let mut i = self.count;
        while i > insert_at {
            self.tasks[i] = self.tasks[i - 1];
            self.delays[i] = self.delays[i - 1];
            i -= 1;
        }

        // Neuen Eintrag setzen und Zähler erhöhen
        self.tasks[insert_at] = entry;
        self.count += 1;

        if insert_at == 0 {
            // Neuer erster Task: TIME3 anpassen, alter erster wird Folge-Delta.
            let old_first = self.time_to_first;
            self.time_to_first = delay_cs;
            self.delays[0] = old_first.saturating_sub(delay_cs);
        } else {
            // Einfügung in der Mitte oder am Ende.
            // abs_prev = absolute Zeit des Vorgängers (Tasks[insert_at - 1]).
            let abs_prev = {
                let mut t = self.time_to_first;
                for k in 0..insert_at - 1 {
                    t = t.saturating_add(self.delays[k]);
                }
                t
            };
            // delays[insert_at - 1] = Abstand Vorgänger → neuer Task
            self.delays[insert_at - 1] = delay_cs.saturating_sub(abs_prev);
            if insert_at < self.count - 1 {
                // delays[insert_at] = Abstand neuer Task → Nachfolger
                self.delays[insert_at] = abs_at_insert.saturating_sub(delay_cs);
            }
        }

        Some(())
    }

    /// Anzahl aktuell wartender Tasks.
    pub fn count(&self) -> usize {
        self.count
    }

    /// Verbleibende Zentisekunden bis zum nächsten Task, oder `None` wenn leer.
    pub fn time_to_next(&self) -> Option<u32> {
        if self.count == 0 {
            None
        } else {
            Some(self.time_to_first)
        }
    }
}

impl Default for Waitlist {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// tick_with_ctx — freie Funktion mit mem::take-Pattern
// ---------------------------------------------------------------------------

/// Verarbeitet `ticks` Zentisekunden gegen die Waitlist in `ctx`.
///
/// Diese freie Funktion ersetzt [`Waitlist::tick`] für alle Kontexte, in denen
/// Task-Funktionen auf [`crate::context::AgcContext`] zugreifen müssen (z. B.
/// um sich selbst via `ctx.waitlist.call_after(...)` neu einzuplanen).
///
/// ## Borrow-Strategie (mem::take-Pattern)
///
/// 1. Die Waitlist wird via `core::mem::take` aus `ctx.waitlist` herausgenommen
///    — `ctx.waitlist` ist danach leer (Default).
/// 2. Zeitberechnung und Listenmanipulation finden auf der lokalen `wl` statt.
/// 3. Fällige Task-Funktionszeiger werden zunächst in ein lokales Array
///    `fired_fns` gesammelt, bevor Tasks ausgeführt werden.
/// 4. Nicht-fällige Resteinträge werden in `ctx.waitlist` zurückgeschrieben.
/// 5. Fällige Tasks werden nacheinander mit `f(ctx)` aufgerufen. Tasks, die
///    sich selbst neu einplanen, schreiben in das jetzt leere (bzw. bereits
///    teilweise befüllte) `ctx.waitlist`.
///
/// Nach dem Durchlauf enthält `ctx.waitlist` die nicht-fälligen Resteinträge
/// plus alle neu eingetragenen Tasks (z. B. selbst-wiederholende Monitor-Tasks).
///
/// AGC source: `Comanche055/WAITLIST.agc`, `T3RUPT`/`T3RUPT2`/`TASKOVER` routines.
pub fn tick_with_ctx(ctx: &mut crate::context::AgcContext, ticks: u32) -> usize {
    // Schnellpfad: Waitlist leer
    if ctx.waitlist.count == 0 {
        return 0;
    }

    // ① Waitlist herausnehmen — ctx.waitlist ist danach Default (leer)
    let mut wl = core::mem::take(&mut ctx.waitlist);

    let mut fired_fns = [None::<fn(&mut crate::context::AgcContext)>; MAX_TASKS];
    let mut fired_count = 0;

    if ticks >= wl.time_to_first {
        let mut remaining_ticks = ticks.saturating_sub(wl.time_to_first);
        wl.time_to_first = 0;

        // Alle fälligen Tasks sammeln (analog T3RUPT2/RUPTAGN-Schleife)
        loop {
            if wl.count == 0 {
                break;
            }
            if wl.time_to_first != 0 {
                break;
            }

            // Task 0 extrahieren und Delta zum Nachfolger lesen (vor Shift)
            let f = wl.tasks[0].task_fn;
            let delta_to_next = wl.delays[0];

            // Liste nach vorne schieben
            wl.count -= 1;
            for i in 0..wl.count {
                wl.tasks[i] = wl.tasks[i + 1];
                wl.delays[i] = wl.delays[i + 1];
            }
            wl.tasks[wl.count] = WaitEntry::empty();
            wl.delays[wl.count] = 0;

            // Nächsten time_to_first berechnen
            if wl.count > 0 {
                if remaining_ticks >= delta_to_next {
                    remaining_ticks = remaining_ticks.saturating_sub(delta_to_next);
                    wl.time_to_first = 0;
                } else {
                    wl.time_to_first = delta_to_next.saturating_sub(remaining_ticks);
                    remaining_ticks = 0;
                }
            }

            // Funktionszeiger merken (noch nicht aufrufen)
            if fired_count < MAX_TASKS {
                fired_fns[fired_count] = f;
                fired_count += 1;
            }
        }

        // Falls noch Ticks übrig und nächster Task noch nicht fällig
        if wl.count > 0 && remaining_ticks > 0 {
            wl.time_to_first = wl.time_to_first.saturating_sub(remaining_ticks);
        }
    } else {
        wl.time_to_first -= ticks;
    }

    // ③ Verbleibende nicht-fällige Einträge zurückschreiben
    ctx.waitlist = wl;

    // ④ Fällige Tasks mit &mut ctx aufrufen.
    //    Tasks, die call_after() aufrufen, schreiben in ctx.waitlist
    //    (das jetzt die nicht-fälligen Resteinträge enthält).
    for f in fired_fns.iter().take(fired_count).flatten() {
        f(ctx);
    }

    fired_count
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{tick_with_ctx, Waitlist, MAX_TASKS};
    use crate::context::AgcContext;
    use core::sync::atomic::{AtomicU32, Ordering};

    // Atomare Zähler für Task-Ausführungsnachweis
    static COUNTER_A: AtomicU32 = AtomicU32::new(0);
    static COUNTER_B: AtomicU32 = AtomicU32::new(0);
    static COUNTER_C: AtomicU32 = AtomicU32::new(0);

    fn task_a(_ctx: &mut AgcContext) {
        COUNTER_A.fetch_add(1, Ordering::Relaxed);
    }
    fn task_b(_ctx: &mut AgcContext) {
        COUNTER_B.fetch_add(1, Ordering::Relaxed);
    }
    fn task_c(_ctx: &mut AgcContext) {
        COUNTER_C.fetch_add(1, Ordering::Relaxed);
    }

    fn reset_counters() {
        COUNTER_A.store(0, Ordering::Relaxed);
        COUNTER_B.store(0, Ordering::Relaxed);
        COUNTER_C.store(0, Ordering::Relaxed);
    }

    fn ctx_with_waitlist(wl: Waitlist) -> AgcContext {
        let mut ctx = AgcContext::default();
        ctx.waitlist = wl;
        ctx
    }

    #[test]
    fn new_waitlist_is_empty() {
        let w = Waitlist::new();
        assert_eq!(w.count(), 0);
        assert!(w.time_to_next().is_none());
    }

    #[test]
    fn single_task_fires_after_delay() {
        reset_counters();
        let mut w = Waitlist::new();
        w.call_after(5, task_a).unwrap();

        assert_eq!(w.count(), 1);
        assert_eq!(w.time_to_next(), Some(5));

        let mut ctx = ctx_with_waitlist(w);

        // 4 Ticks — noch nicht fällig
        let fired = tick_with_ctx(&mut ctx, 4);
        assert_eq!(fired, 0);
        assert_eq!(COUNTER_A.load(Ordering::Relaxed), 0);
        assert_eq!(ctx.waitlist.time_to_next(), Some(1));

        // 1 letzter Tick — fällig
        let fired = tick_with_ctx(&mut ctx, 1);
        assert_eq!(fired, 1);
        assert_eq!(COUNTER_A.load(Ordering::Relaxed), 1);
        assert_eq!(ctx.waitlist.count(), 0);
    }

    #[test]
    fn tick_with_exact_delay_fires_task() {
        reset_counters();
        let mut w = Waitlist::new();
        w.call_after(10, task_a).unwrap();
        let mut ctx = ctx_with_waitlist(w);
        let fired = tick_with_ctx(&mut ctx, 10);
        assert_eq!(fired, 1);
        assert_eq!(COUNTER_A.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn tick_beyond_delay_fires_task() {
        reset_counters();
        let mut w = Waitlist::new();
        w.call_after(3, task_a).unwrap();
        let mut ctx = ctx_with_waitlist(w);
        let fired = tick_with_ctx(&mut ctx, 100);
        assert_eq!(fired, 1);
        assert_eq!(COUNTER_A.load(Ordering::Relaxed), 1);
        assert_eq!(ctx.waitlist.count(), 0);
    }

    #[test]
    fn tasks_fire_in_time_order() {
        reset_counters();
        let mut w = Waitlist::new();

        // task_b soll nach task_a feuern
        w.call_after(10, task_a).unwrap();
        w.call_after(20, task_b).unwrap();

        let mut ctx = ctx_with_waitlist(w);

        // Nach 10 Ticks: nur task_a feuert
        let fired = tick_with_ctx(&mut ctx, 10);
        assert_eq!(fired, 1);
        assert_eq!(COUNTER_A.load(Ordering::Relaxed), 1);
        assert_eq!(COUNTER_B.load(Ordering::Relaxed), 0);

        // Nach weiteren 10 Ticks: task_b feuert
        let fired = tick_with_ctx(&mut ctx, 10);
        assert_eq!(fired, 1);
        assert_eq!(COUNTER_B.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn insert_before_first_task() {
        reset_counters();
        let mut w = Waitlist::new();

        w.call_after(20, task_b).unwrap();
        // task_a wird mit kleinerer Verzögerung eingetragen → soll zuerst feuern
        w.call_after(5, task_a).unwrap();

        assert_eq!(w.time_to_next(), Some(5));

        let mut ctx = ctx_with_waitlist(w);
        let fired = tick_with_ctx(&mut ctx, 5);
        assert_eq!(fired, 1);
        assert_eq!(COUNTER_A.load(Ordering::Relaxed), 1);
        assert_eq!(COUNTER_B.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn max_tasks_capacity() {
        let mut w = Waitlist::new();
        for i in 1..=MAX_TASKS {
            assert!(w.call_after(i as u32 * 10, task_a).is_some());
        }
        // Überlauf: kein weiterer Platz
        assert!(w.call_after(1, task_a).is_none());
    }

    #[test]
    fn zero_delay_clamped_to_one() {
        let mut w = Waitlist::new();
        w.call_after(0, task_a).unwrap();
        assert_eq!(w.time_to_next(), Some(1));
    }

    #[test]
    fn two_tasks_same_tick_both_fire() {
        reset_counters();
        let mut w = Waitlist::new();
        w.call_after(5, task_a).unwrap();
        w.call_after(5, task_b).unwrap();

        let mut ctx = ctx_with_waitlist(w);
        let fired = tick_with_ctx(&mut ctx, 5);
        // Beide sollen feuern
        assert_eq!(fired, 2);
        assert_eq!(COUNTER_A.load(Ordering::Relaxed), 1);
        assert_eq!(COUNTER_B.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn three_tasks_sequential_ticks() {
        reset_counters();
        let mut w = Waitlist::new();
        w.call_after(3, task_a).unwrap();
        w.call_after(6, task_b).unwrap();
        w.call_after(9, task_c).unwrap();

        let mut ctx = ctx_with_waitlist(w);

        tick_with_ctx(&mut ctx, 3);
        assert_eq!(COUNTER_A.load(Ordering::Relaxed), 1);
        assert_eq!(COUNTER_B.load(Ordering::Relaxed), 0);
        assert_eq!(COUNTER_C.load(Ordering::Relaxed), 0);

        tick_with_ctx(&mut ctx, 3);
        assert_eq!(COUNTER_B.load(Ordering::Relaxed), 1);
        assert_eq!(COUNTER_C.load(Ordering::Relaxed), 0);

        tick_with_ctx(&mut ctx, 3);
        assert_eq!(COUNTER_C.load(Ordering::Relaxed), 1);
        assert_eq!(ctx.waitlist.count(), 0);
    }

    /// Prüft das self-rescheduling-Pattern: Ein Task trägt sich in `ctx.waitlist`
    /// selbst wieder ein. Nach zwei Tick-Zyklen muss er zweimal gefeuert haben.
    #[test]
    fn self_rescheduling_task() {
        reset_counters();

        fn repeating(_ctx: &mut AgcContext) {
            COUNTER_A.fetch_add(1, Ordering::Relaxed);
            // Sich selbst nach 5 cs wieder einplanen
            let _ = _ctx.waitlist.call_after(5, repeating);
        }

        let mut ctx = AgcContext::default();
        ctx.waitlist.call_after(5, repeating).unwrap();

        // Erster Feuerzyklus nach 5 cs
        tick_with_ctx(&mut ctx, 5);
        assert_eq!(COUNTER_A.load(Ordering::Relaxed), 1);
        // Task hat sich selbst wieder eingetragen
        assert_eq!(ctx.waitlist.count(), 1);

        // Zweiter Feuerzyklus nach weiteren 5 cs
        tick_with_ctx(&mut ctx, 5);
        assert_eq!(COUNTER_A.load(Ordering::Relaxed), 2);
        assert_eq!(ctx.waitlist.count(), 1);
    }
}
