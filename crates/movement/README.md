# movement

Pluggable per-entity embodiment: turns a desired velocity into a physical
force each tick. Independent of networking and scenario logic.

## Contents

- **`MovementModel`** — the trait every embodiment implements
  (`compute_force(desired, current) -> force`). The seam where `Holonomic`
  and, later, `CarLike`/`FullVehicle` differ.
- **`Holonomic`** — v1's only model: free horizontal steering via a
  proportional controller, with a separate higher force ceiling for urgent
  (braking) commands.
- **`DesiredVelocity`** — the shared control contract, written by `server`'s
  arbitration and read by the movement systems. Carries an `urgent` flag so
  brakes aren't limited by the cruising force cap.
- **`MovementPlugin`** — registers force application (monomorphized per
  embodiment, ordered before Rapier's step) and the cosmetic
  face-direction-of-travel system.

Force is written to `bevy_rapier3d`'s `ExternalForce` by overwrite, never
accumulation.
