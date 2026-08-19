//! The scenario's ownership of run length: converting a `duration` in
//! sim-seconds into a tick deadline, and deciding when the run has reached it.
//! Pure functions + a `Deadline` resource; the enforcing system lives in the
//! transport bridge.

use bevy::prelude::Resource;

/// The fixed physics step rate. Not scenario-owned -- Rapier is tuned to it and
/// it's the `tick_rate` advertised to viewers (which key their playback clock
/// to it), so it's an engine invariant. `duration` (sim-seconds) and a pulse's
/// `dt` both convert through it.
pub const TICK_HZ: f64 = 64.0;

/// The tick at which a run of `duration_seconds` ends, at the fixed physics
/// rate. `None` (unbounded) stays `None`. A negative duration clamps to 0
/// (ends immediately) rather than wrapping.
pub fn deadline_tick(duration_seconds: Option<f64>, tick_hz: f64) -> Option<u64> {
    duration_seconds.map(|d| (d.max(0.0) * tick_hz).round() as u64)
}

/// Whether a run at `tick` has reached its `deadline`. An unbounded run
/// (`None`) is never reached.
pub fn reached(tick: u64, deadline: Option<u64>) -> bool {
    matches!(deadline, Some(d) if tick >= d)
}

/// The precomputed tick deadline for this run, `None` if unbounded. Set once at
/// startup from the scenario's `time.duration`.
#[derive(Resource, Default, Clone, Copy)]
pub struct Deadline(pub Option<u64>);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unbounded_has_no_deadline_and_is_never_reached() {
        let d = deadline_tick(None, TICK_HZ);
        assert_eq!(d, None);
        assert!(!reached(0, d));
        assert!(!reached(u64::MAX, d));
    }

    #[test]
    fn duration_converts_to_ticks_at_the_rate() {
        // 30 sim-seconds at 64 Hz = 1920 ticks.
        assert_eq!(deadline_tick(Some(30.0), 64.0), Some(1920));
        // Rounds to the nearest tick.
        assert_eq!(deadline_tick(Some(0.01), 64.0), Some(1)); // 0.64 -> 1
        assert_eq!(deadline_tick(Some(0.007), 64.0), Some(0)); // 0.448 -> 0
    }

    #[test]
    fn negative_duration_clamps_to_zero() {
        assert_eq!(deadline_tick(Some(-5.0), 64.0), Some(0));
    }

    #[test]
    fn reached_at_and_after_the_deadline() {
        let d = deadline_tick(Some(1.0), 64.0); // Some(64)
        assert!(!reached(63, d));
        assert!(reached(64, d));
        assert!(reached(65, d));
    }
}
