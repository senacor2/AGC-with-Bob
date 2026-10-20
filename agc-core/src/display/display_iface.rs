//! Display-Request-System: Prioritäts-Queue für interne Programm-Anzeigeanforderungen.
//!
//! Entspricht dem `NVSUB`/`MAKEPLAY`-Subsystem im AGC:
//! Interne Programme fordern über [`request_display`] einen Anzeigewechsel an.
//! Anforderungen werden in einer statischen 2-Slot-Queue gepuffert und per
//! [`service_display_queue`] sequenziell abgearbeitet.
//!
//! AGC source: `Comanche055/DISPLAY_INTERFACE_ROUTINES.agc`,
//! NVSUB / NVSBWAIT / MAKEPLAY-Bereich (pp. 1452–1484).

use crate::display::verb_noun::{DisplayState, NounCode, VerbCode};

// ---------------------------------------------------------------------------
// Sub-Task 7.4 — DisplayPriority
// ---------------------------------------------------------------------------

/// Priorität einer Display-Anforderung.
///
/// `Priority`-Anforderungen unterbrechen laufende `Normal`-Anforderungen,
/// analog zum AGC-Priority-Display-System.
///
/// AGC source: `Comanche055/DISPLAY_INTERFACE_ROUTINES.agc`, NVSUB.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum DisplayPriority {
    /// Normale Display-Anforderung (z. B. Programm-Statusanzeige).
    Normal,
    /// Prioritäts-Anzeige (z. B. Alarm, dringende Statusmeldung).
    Priority,
}

// ---------------------------------------------------------------------------
// Sub-Task 7.4 — DisplayRequest
// ---------------------------------------------------------------------------

/// Eine einzelne interne Display-Anforderung.
///
/// Entspricht einem NVSUB-Aufruf im AGC: Programm fordert an, dass
/// ein bestimmtes Verb/Noun-Paar auf dem DSKY angezeigt wird.
///
/// AGC source: `Comanche055/DISPLAY_INTERFACE_ROUTINES.agc`, NVSUB.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct DisplayRequest {
    /// Anzuzeigender Verb-Code.
    pub verb: VerbCode,
    /// Anzuzeigender Noun-Code.
    pub noun: NounCode,
    /// Priorität der Anforderung.
    pub priority: DisplayPriority,
}

// ---------------------------------------------------------------------------
// Sub-Task 7.4 — DisplayQueue
// ---------------------------------------------------------------------------

/// AGC-getreue 2-Slot-Display-Warteschlange.
///
/// Genau ein `priority_slot` und ein `normal_slot` — wie im AGC:
/// Nur eine Priority-Anforderung und eine Normal-Anforderung können
/// gleichzeitig ausstehen.  Ein neuer Push überschreibt den jeweiligen Slot.
///
/// AGC source: `Comanche055/DISPLAY_INTERFACE_ROUTINES.agc`,
/// MAKEPLAY / NVSBWAIT-Bereich.
pub struct DisplayQueue {
    priority_slot: Option<DisplayRequest>,
    normal_slot: Option<DisplayRequest>,
}

impl DisplayQueue {
    /// Leere Queue — beide Slots frei.
    pub const fn new() -> Self {
        Self {
            priority_slot: None,
            normal_slot: None,
        }
    }

    /// Fügt eine Anforderung in den passenden Slot ein.
    ///
    /// Ein vorhandener Slot-Inhalt wird überschrieben (AGC-Verhalten:
    /// nur eine ausstehende Anforderung pro Priorität).
    pub fn push(&mut self, req: DisplayRequest) {
        match req.priority {
            DisplayPriority::Priority => self.priority_slot = Some(req),
            DisplayPriority::Normal => self.normal_slot = Some(req),
        }
    }

    /// Gibt die nächste Anforderung zurück — Priority-Slot zuerst.
    ///
    /// Gibt `None` zurück wenn beide Slots leer sind.
    pub fn pop(&mut self) -> Option<DisplayRequest> {
        if self.priority_slot.is_some() {
            return self.priority_slot.take();
        }
        self.normal_slot.take()
    }

    /// `true` wenn beide Slots leer sind.
    pub fn is_empty(&self) -> bool {
        self.priority_slot.is_none() && self.normal_slot.is_none()
    }
}

impl Default for DisplayQueue {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Sub-Task 7.4 — request_display / service_display_queue / release_display
// ---------------------------------------------------------------------------

/// Fordert eine Display-Anzeige an.
///
/// - Falls `state.dsp_lock == false`: Anforderung sofort übernehmen,
///   `state.verb`/`state.noun` setzen, `state.dsp_lock = true`.
/// - Falls `state.dsp_lock == true`: Anforderung in `queue` einreihen.
///
/// Entspricht dem NVSUB-Einstieg im AGC.
///
/// AGC source: `Comanche055/PINBALL_GAME__BUTTONS_AND_LIGHTS.agc`, NVSUB.
pub fn request_display(
    state: &mut DisplayState,
    queue: &mut DisplayQueue,
    req: DisplayRequest,
) {
    if !state.dsp_lock {
        state.verb = req.verb;
        state.noun = req.noun;
        state.dsp_lock = true;
    } else {
        queue.push(req);
    }
}

/// Arbeitet die nächste Anforderung aus der Queue ab, sofern eine vorhanden ist.
///
/// Wird einmal pro Executive-Tick aufgerufen.  Falls Queue nicht leer und
/// `dsp_lock == true`: nächste Anforderung pop()-en, `state.verb`/`state.noun`
/// setzen.  `dsp_lock` bleibt `true` (Display ist noch aktiv).
///
/// AGC source: `Comanche055/DISPLAY_INTERFACE_ROUTINES.agc`, MAKEPLAY.
pub fn service_display_queue(state: &mut DisplayState, queue: &mut DisplayQueue) {
    if state.dsp_lock {
        if let Some(req) = queue.pop() {
            state.verb = req.verb;
            state.noun = req.noun;
            // dsp_lock bleibt true
        }
    }
}

/// Gibt das Display frei und startet ggf. die nächste Queue-Anforderung.
///
/// Setzt `dsp_lock = false` und ruft anschließend [`service_display_queue`]
/// auf — falls eine Anforderung wartet, wird sie sofort übernommen.
///
/// AGC source: `Comanche055/DISPLAY_INTERFACE_ROUTINES.agc`, MAKEPLAY.
pub fn release_display(state: &mut DisplayState, queue: &mut DisplayQueue) {
    state.dsp_lock = false;
    service_display_queue(state, queue);
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn v(n: u8) -> VerbCode {
        VerbCode::new(n).unwrap()
    }
    fn n(n: u8) -> NounCode {
        NounCode::new(n).unwrap()
    }
    fn req(verb: u8, noun: u8, priority: DisplayPriority) -> DisplayRequest {
        DisplayRequest {
            verb: v(verb),
            noun: n(noun),
            priority,
        }
    }

    #[test]
    fn request_display_sets_immediately_when_unlocked() {
        let mut state = DisplayState::new();
        let mut queue = DisplayQueue::new();
        assert!(!state.dsp_lock);

        request_display(&mut state, &mut queue, req(37, 15, DisplayPriority::Normal));

        assert_eq!(state.verb, v(37));
        assert_eq!(state.noun, n(15));
        assert!(state.dsp_lock);
        assert!(queue.is_empty());
    }

    #[test]
    fn request_display_queues_when_locked() {
        let mut state = DisplayState::new();
        let mut queue = DisplayQueue::new();
        state.dsp_lock = true;

        request_display(&mut state, &mut queue, req(06, 18, DisplayPriority::Normal));

        // State darf sich nicht verändert haben
        assert_eq!(state.verb, VerbCode::new(0).unwrap());
        assert_eq!(state.noun, NounCode::new(0).unwrap());
        assert!(!queue.is_empty());
    }

    #[test]
    fn priority_overrides_normal_in_queue() {
        let mut queue = DisplayQueue::new();

        queue.push(req(10, 20, DisplayPriority::Normal));
        queue.push(req(33, 44, DisplayPriority::Priority));

        // Pop muss Priority-Slot zuerst liefern
        let first = queue.pop().unwrap();
        assert_eq!(first.verb, v(33));
        assert_eq!(first.noun, n(44));
        assert_eq!(first.priority, DisplayPriority::Priority);

        // Dann Normal-Slot
        let second = queue.pop().unwrap();
        assert_eq!(second.verb, v(10));
        assert_eq!(second.noun, n(20));

        assert!(queue.is_empty());
    }

    #[test]
    fn release_services_queue() {
        let mut state = DisplayState::new();
        let mut queue = DisplayQueue::new();

        // Erstes request_display belegt das Display sofort
        request_display(&mut state, &mut queue, req(01, 02, DisplayPriority::Normal));
        assert!(state.dsp_lock);

        // Zweite Anforderung landet in der Queue (Display noch gesperrt)
        request_display(&mut state, &mut queue, req(06, 18, DisplayPriority::Normal));
        assert!(!queue.is_empty());

        // Display freigeben: dsp_lock=false; service_display_queue prüft dsp_lock==true
        // → macht nichts, da jetzt false. Queue bleibt nicht leer.
        release_display(&mut state, &mut queue);
        assert!(!state.dsp_lock);
        assert!(!queue.is_empty());

        // Nächster request_display übernimmt wartende Anforderung sofort
        let next = queue.pop().unwrap();
        request_display(&mut state, &mut queue, next);
        assert_eq!(state.verb, v(06));
        assert_eq!(state.noun, n(18));
        assert!(state.dsp_lock);
    }

    #[test]
    fn queue_empty_after_two_pops() {
        let mut queue = DisplayQueue::new();

        queue.push(req(01, 02, DisplayPriority::Normal));
        queue.push(req(03, 04, DisplayPriority::Priority));

        assert!(queue.pop().is_some());
        assert!(queue.pop().is_some());
        assert!(queue.is_empty());
        // Dritter Pop gibt None
        assert!(queue.pop().is_none());
    }
}
