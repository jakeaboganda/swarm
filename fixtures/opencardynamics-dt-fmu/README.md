# opencardynamics_dt.fmu

A **double-track** TUMFTM [Open-Car-Dynamics](https://github.com/TUMFTM/Open-Car-Dynamics)
vehicle model wrapped as an FMI 3.0 Co-Simulation FMU. Unlike the single-track
`opencardynamics-fmu/` sibling, this exposes **roll** and **pitch** and accepts a
road **bank / grade**, so an FMU vehicle can lean through a canted corner.

## What's here
- `wrapper.cpp` -- fmi3 CS entry points over one concrete double-track combo.
- `harness.cpp` -- standalone proof: `dlopen`s the built `.so`, drives three
  conditions, and asserts the car leans in corners and that the bank couples in.
- `CMakeLists.txt`, `build.sh`, `fmi3/*.h`, `modelDescription.xml`.
- `opencardynamics_dt.fmu` -- the built artifact (Linux x86_64).

## Model combo
`WHEEL_TORQUE` drivetrain x `PT1` steering actuator x `DOUBLE_TRACK` vehicle
dynamics x `MF52` tire x `DEFAULT` aero. The double-track model integrates real
3D DOF: roll `phi`, pitch `theta`, heave `z`, and per-wheel vertical states.

## Binding (value references)
| VR | name | dir | meaning |
|----|------|-----|---------|
| 1 | steer | in | steering-actuator `steering_angle_rad` |
| 2 | throttle | in | per-wheel drive torque (throttle x 500 Nm) |
| 3 | brake | in | per-wheel brake torque (brake x 1500 Nm) |
| 4 | ground_height | in | road height, all wheels (INERT -- see below) |
| 5 | ground_friction | in | tire-road `lambda_mue`, all wheels |
| 6 | bank | in | road bank/superelevation (rad) -- injected lateral force |
| 7 | grade | in | road longitudinal grade (rad) -- injected longitudinal force |
| 10-13 | x, y, z, yaw | out | pose (OCD frame): x/y planar, z = heave, yaw = psi |
| 14 | roll | out | body roll `phi` (rad) |
| 15 | pitch | out | body pitch `theta` (rad) |

## How the bank works (the modeling core)
OCD ignores road height entirely (`z_height_road_m` is only logged, never used in
the equations), so banking cannot enter via the road surface. Instead the wrapper
**injects** it: on a road canted by `bank`, gravity has a component along the
surface, added as `ExternalInfluences.external_force_N` in the vehicle frame
(`F_y = -m*g*sin(bank)`, `F_x = -m*g*sin(grade)`, `m` = 800 kg nominal, `g` = 9.81).
The double-track model's roll DOF then responds. Roll/pitch/heave are read from
the model's logger (`vehicle_dynamics/x_vec/{phi_rad,theta_rad,z_m}`) since they
are not in `VehicleModelOutput`.

## Known-open items
- **Roll is small.** The AV21-derived params are a stiff racecar: cornering roll
  is ~0.25 deg and a 0.15 rad bank adds ~0.05 deg. The mechanism is correct but
  the visual demo will want softer roll params or display exaggeration (later).
- **Bank sign** vs corner direction is provisional (confirmed empirically by the
  orchestrator) -- a banked corner should push the car toward the inside.
- **Frame / road-conform** (mapping OCD's flat output onto a banked 3D track) is
  the swarm-side integration slice, not done here.
- `ground_height` is inert (a planar-datum tire model); friction does feed the tire.
- **Roll tuning:** the wrapper softens the suspension roll stiffness from the AV21
  racecar's values (anti-roll bars front 12000 / rear 5000 N/m) to road-car values
  (~1200 / 700, softer springs) after `reset()`, via the param manager
  (`vehicle_dynamics_double_track.suspension.*`). The stiff racecar rolls <0.25 deg
  in a hard corner -- correct but invisible; the road-car tuning gives ~0.5 deg so
  the body lean reads. On a banked track the dominant visible cant is the car
  conforming to the banked surface; this body roll is the realistic addition on top.
- Ships a single Linux x86_64 binary.

## Rebuild
```
./build.sh
```
