//! Server-driven step callbacks: the per-agent "one-in-flight" state that
//! decides when to push a `Tick` pulse and what `dt` it carries. Pure logic in
//! `PulseState` (unit-tested here) + a `PulseStates` resource keyed by
//! connection; the sending system lives in the transport bridge.
//!
//! The contract: an agent opts in with `subscribe`, then the server pushes one
//! pulse and waits (never blocking the sim) for the matching `ack` before the
//! next. A fast agent that acks immediately is pulsed again next tick (dt ~
//! one physics step); a slow agent gets fewer pulses with a larger dt. The
//! rate is best-effort -- `dt` is the authoritative measure of elapsed
//! sim-time.

use std::collections::HashMap;

use bevy::prelude::Resource;
use transport::ConnectionId;

/// One agent's callback state.
#[derive(Default, Clone, Copy, Debug, PartialEq)]
pub struct PulseState {
    subscribed: bool,
    /// A pulse has been sent and its `ack` not yet seen -- gate the next one.
    awaiting_ack: bool,
    /// Tick of the last pulse sent (or of subscription, before the first
    /// pulse), so the next pulse's `dt` measures sim-time since it.
    last_pulse_tick: u64,
}

impl PulseState {
    /// Opt in. Idempotent: a repeat subscribe while already subscribed is a
    /// no-op, so it can't reset an in-flight pulse. Anchors `dt` to `tick`.
    pub fn subscribe(&mut self, tick: u64) {
        if !self.subscribed {
            self.subscribed = true;
            self.awaiting_ack = false;
            self.last_pulse_tick = tick;
        }
    }

    /// Acknowledge the outstanding pulse, releasing the next. Only the pulse
    /// actually in flight (matching `last_pulse_tick`) counts; a stale or
    /// mismatched ack is ignored.
    pub fn ack(&mut self, tick: u64) {
        if self.awaiting_ack && tick == self.last_pulse_tick {
            self.awaiting_ack = false;
        }
    }

    /// If a pulse is due at `tick`, mark it in flight and return its `dt`
    /// (sim-seconds since the previous pulse). `None` when not subscribed or
    /// still awaiting the last ack -- the caller sends a pulse exactly when
    /// this is `Some`.
    pub fn poll(&mut self, tick: u64, tick_hz: f64) -> Option<f32> {
        if !self.subscribed || self.awaiting_ack {
            return None;
        }
        let dt = tick.saturating_sub(self.last_pulse_tick) as f64 / tick_hz;
        self.awaiting_ack = true;
        self.last_pulse_tick = tick;
        Some(dt as f32)
    }
}

/// Per-connection callback state. An entry is created on first subscribe/ack
/// and dropped when the connection goes away.
#[derive(Resource, Default)]
pub struct PulseStates(pub HashMap<ConnectionId, PulseState>);

#[cfg(test)]
mod tests {
    use super::*;

    const HZ: f64 = 64.0;

    #[test]
    fn unsubscribed_never_pulses() {
        let mut s = PulseState::default();
        assert_eq!(s.poll(10, HZ), None);
    }

    #[test]
    fn first_pulse_after_subscribe_has_zero_dt() {
        let mut s = PulseState::default();
        s.subscribe(100);
        assert_eq!(s.poll(100, HZ), Some(0.0));
    }

    #[test]
    fn one_in_flight_gates_until_acked() {
        let mut s = PulseState::default();
        s.subscribe(100);
        assert_eq!(s.poll(100, HZ), Some(0.0)); // pulse for tick 100 in flight
        assert_eq!(s.poll(101, HZ), None); // gated: no ack yet
        s.ack(100);
        // dt now measures 132 - 100 = 32 ticks = 0.5 s at 64 Hz.
        assert_eq!(s.poll(132, HZ), Some(0.5));
    }

    #[test]
    fn a_mismatched_ack_does_not_release() {
        let mut s = PulseState::default();
        s.subscribe(0);
        s.poll(0, HZ); // pulse for tick 0 in flight
        s.ack(7); // wrong tick
        assert_eq!(s.poll(64, HZ), None); // still gated
        s.ack(0); // correct
        assert_eq!(s.poll(64, HZ), Some(1.0));
    }

    #[test]
    fn fast_agent_pulses_every_tick_with_one_step_dt() {
        let mut s = PulseState::default();
        s.subscribe(0);
        assert_eq!(s.poll(0, HZ), Some(0.0));
        s.ack(0);
        assert_eq!(s.poll(1, HZ), Some(1.0 / HZ as f32));
        s.ack(1);
        assert_eq!(s.poll(2, HZ), Some(1.0 / HZ as f32));
    }

    #[test]
    fn resubscribe_is_a_noop_and_keeps_the_pulse_in_flight() {
        let mut s = PulseState::default();
        s.subscribe(50);
        s.poll(50, HZ); // in flight, awaiting ack
        s.subscribe(90); // must not reset anything
        assert_eq!(s.poll(90, HZ), None); // still awaiting the tick-50 ack
        s.ack(50);
        assert_eq!(s.poll(114, HZ), Some(1.0)); // 114 - 50 = 64 ticks = 1 s
    }
}
