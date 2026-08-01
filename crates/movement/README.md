# movement

Pluggable per-entity embodiment: turns a desired velocity into a physical
force each tick. Independent of networking and scenario logic.

## Contents

- **`MovementModel`** — the trait every embodiment implements
  (`drive(&mut self, desired, current, dt) -> force`). `&mut self` + `dt`
  let a model carry state that evolves over time. The seam where the models
  below (and a future `FullVehicle`) differ.
- **`Holonomic`** — free horizontal steering in any direction via a
  proportional controller, with a separate higher force ceiling for urgent
  (braking) commands.
- **`CarLike`** — non-holonomic: forward-only thrust, a bounded turn rate
  (so it makes sweeping turns rather than instant ones), and lateral grip
  that cancels sideways sliding.
- **`DesiredVelocity`** — the shared control contract, written by `server`'s
  arbitration and read by the movement systems. Carries an `urgent` flag so
  brakes aren't limited by the cruising force cap.
- **`MovementPlugin`** — registers force application (monomorphized per
  embodiment, ordered before Rapier's step) and the cosmetic
  face-direction-of-travel system.

Force is written to `bevy_rapier3d`'s `ExternalForce` by overwrite, never
accumulation.
