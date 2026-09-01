# opencardynamics.fmu

An FMI 3.0 Co-Simulation wrapper around one concrete [TUMFTM
Open-Car-Dynamics](https://github.com/TUMFTM/Open-Car-Dynamics) (OCD) vehicle
model, so the swarm sim's `FmuVehicle` embodiment can drive a real
vehicle-dynamics plant instead of the VanDerPol toy. OCD is Apache-2.0.

## What's here

- `wrapper.cpp` -- the `fmi3` C entry points over one concrete OCD `VehicleModel`.
- `modelDescription.xml` -- FMI 3.0 CS interface (the 9 variables below).
- `CMakeLists.txt` -- trimmed build: compiles ONLY the concrete combo's source
  subset + the wrapper, links Eigen. Excludes the OCD packages that pull Boost
  (ccma / track / vehicle-handler) and matplotlib / ROS 2.
- `harness.cpp` -- standalone proof: `dlopen`s the built `.so`, drives it through
  the real `fmi3` API, asserts the car accelerates under throttle and yaws under
  steer.
- `build.sh` -- reproducible build: fetches OCD (pinned) + Eigen, builds, runs
  the harness, assembles the `.fmu`.
- `fmi3/` -- the FMI 3.0 standard headers (from the `fmi-sys` crate's vendored set).
- `opencardynamics.fmu` -- the built artifact (Linux x86_64), committed like the
  VanDerPol reference fixture.

## Model combo (v1)

`WHEEL_TORQUE` drivetrain x `PT1` steering actuator x `SINGLE_TRACK` vehicle
dynamics x `MF_Simple` tire x `DEFAULT` aerodynamics. The concrete C++ type:

```cpp
tam::ocd::VehicleModel<
  tam::ocd::drivetrain::DrivetrainWheelTorqueModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel<
    tam::ocd::tire_models::MF_Simple,
    tam::ocd::aerodynamics::DefaultAerodynamicsModel>>
```

The model's declared parameter defaults are the real (AV21-derived) values -- the
config JSONs in OCD's `python3/.../config/` are just dumps of them -- so no
external parameter file is loaded. Internal integration step is 8e-4 s; one
`fmi3DoStep(h)` runs `round(h / 8e-4)` OCD `step()`s.

OCD pin: `94f8fb187fb0ed22bba1d809bd74f66d1ff75af4`.

## Binding (value references)

| vr | name            | dir | maps to                                             |
|----|-----------------|-----|-----------------------------------------------------|
| 1  | steer           | in  | steering-actuator `steering_angle_rad`              |
| 2  | throttle        | in  | per-wheel drive torque (throttle x DRIVE_TORQUE_NM) |
| 3  | brake           | in  | per-wheel brake torque (brake x BRAKE_TORQUE_NM)    |
| 4  | ground_height   | in  | `ExternalInfluences.z_height_road_m` (all wheels)   |
| 5  | ground_friction | in  | `ExternalInfluences.lambda_mue` (all wheels)        |
| 10 | x               | out | `vehicle_dynamics_output.position_m.x`              |
| 11 | y               | out | `vehicle_dynamics_output.position_m.y`              |
| 12 | z               | out | `vehicle_dynamics_output.position_m.z`              |
| 13 | yaw             | out | `vehicle_dynamics_output.orientation_rad.z`         |

This matches the `FmuConfig` binding a swarm scenario uses for an `FmuVehicle`.
`normal_z` has no OCD counterpart and is intentionally left unbound.

## Rebuild

```
./build.sh
```

## Known-open items (for the swarm-side slices)

- **Coordinate frame.** Outputs are in OCD's OWN world frame (its x forward,
  y left, z up, yaw about z). The swarm sim is Y-up. The `FmuVehicle` path
  stamps the FMU pose straight onto the Transform, so frame reconciliation
  (axis mapping and/or a spawn rebase) is a swarm-side concern -- slice C.
- **Throttle/brake -> torque + steer sign/units.** The pedal->torque scale
  (`DRIVE_TORQUE_NM` / `BRAKE_TORQUE_NM`) and the steering-angle sign are v1
  guesses; verify/tune once the car drives a plan in-sim (slice B/C). OCD steer
  is a wheel angle in rad, clamped by `steering_actuator.angle_max_rad` (0.3).
- **Per-wheel ground.** v1 feeds the single-point ground height/friction to all
  four wheels; per-wheel is the v2 the swarm ground raycast defers.
- **`ground_height` is physically inert for this combo.** The `SINGLE_TRACK`
  model is a planar bicycle model with no heave DOF: `z_height_road_m` is
  accepted and threaded into `ExternalInfluences`, but the single-track equations
  never read it (vertical tire load comes from static weight + load transfer, and
  the aero heave input is hardcoded to 0). So the channel is wired but changing
  `ground_height` has no effect until a model with a vertical DOF (double-track)
  is used. `ground_friction` (`lambda_mue`) *does* feed the tire model.
- **Multi-instance.** All model state is per-instance (owned by each `Instance`'s
  `VehicleModel`), so several FMU vehicles spawned from one loaded `.so` run
  independently with no cross-talk. The one process-wide static in the reachable
  set (`param_management`'s `InitializationBackend::_storage`) stays empty because
  this wrapper never calls `tam::pmg::init()`. If a future revision loads
  per-instance parameter overrides via that call, it would write shared state
  every instance reads -- design around it then.
- **Portability.** Ships a single Linux x86_64 binary. Other platforms need a
  matching `binaries/<platform>/` entry (rebuild there).
