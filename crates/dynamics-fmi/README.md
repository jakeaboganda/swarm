# dynamics-fmi

The engine-free core for the `FmuVehicle` embodiment. It loads an FMI 3.0
co-simulation **FMU**, wires the scenario's variable bindings to it, maps a
plan to pedals, and reads the FMU's integrated pose back out. No
`bevy`/`rapier` deps, so it unit-tests in isolation against an in-memory
fake `FmuInstance`. `server` owns the Bevy/Rapier side. It steps the FMU
each tick and imposes the pose on a kinematic body.

## Contents

- **`FmuInstance`**. The trait a real FMI 3.0 instance satisfies (set
  inputs, step, get outputs). The seam the `fmi`-crate-backed impl and the
  test fake both implement.
- **`Fmu`**. Loads an `.fmu` and its `ModelDescription` (variables,
  causality, value references).
- **`BindingSpec` / `ResolvedBinding`**. Resolves roles (steer, throttle,
  brake, ground query, pose outputs) to FMI value references once at spawn.
- **`Driver`**. Maps a `DriverInput` (plan progress plus speed error) to
  pedal `Controls`, holding its PI integrator state.
- **`read_pose` / `Pose` / `FmuFrame`**. Reads the FMU's output pose and
  converts it from the FMU's frame into sim-local coordinates.

Depends on the `fmi` crate (which runs bindgen, so a build needs libclang;
see the `Cargo.toml` note).
