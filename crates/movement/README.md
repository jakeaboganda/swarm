# movement

Pluggable per-entity embodiment: turns a desired velocity into a physical
force each tick. Independent of networking and scenario logic.

## Contents

- **`MovementModel`** — the trait every embodiment implements
  (`drive(&mut self, desired, body, dt) -> Actuation`, a force plus a yaw
  torque). `&mut self` + `dt` let a model carry state that evolves over time
  (e.g. a steering angle). The seam where the models below differ.
- **`Holonomic`** — free horizontal steering in any direction via a
  proportional controller, with a separate higher force ceiling for urgent
  (braking) commands.
- **`CarLike`** — non-holonomic: forward-only thrust, a bounded turn rate
  (so it makes sweeping turns rather than instant ones), and lateral grip
  that cancels sideways sliding. Yaw is a cosmetic facing.
- **`FullVehicle`** — single-track ("bicycle") dynamics: real physical yaw
  from a linear tire model (front/rear cornering forces + a yaw torque), so
  understeer/oversteer and sliding emerge instead of being scripted. A two-
  layer split: a *driver* maps `DesiredVelocity` to steering/drive controls,
  a *plant* maps those to forces. The one embodiment that opts into yaw as a
  real physical DOF.
- **`DesiredVelocity`** — the shared control contract, written by `server`'s
  arbitration and read by the movement systems. Carries an `urgent` flag so
  brakes aren't limited by the cruising force cap.
- **`MovementPlugin`** — registers force application (monomorphized per
  embodiment, ordered before Rapier's step) and the cosmetic
  face-direction-of-travel system.

Force and yaw torque are written to `bevy_rapier3d`'s `ExternalForce` by
overwrite, never accumulation.
