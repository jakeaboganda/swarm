// FMI 3.0 Co-Simulation wrapper around one concrete TUMFTM Open-Car-Dynamics
// vehicle model. Exposes the plant as a standard .fmu so the swarm sim's
// FmuVehicle embodiment can drive it. See README.md for the value-reference
// binding table and the open frame/sign items.
//
// Concrete combo (v1): WHEEL_TORQUE drivetrain x PT1 steering actuator x
// SINGLE_TRACK vehicle dynamics x MF_Simple tire x DEFAULT aero. The model's
// declared parameter defaults are the real (AV21-derived) values -- the shipped
// config JSON is just a dump of them -- so no external parameter file is loaded.

#include <cmath>
#include <cstring>
#include <new>
#include <string>

// OCD model headers (the concrete combo's transitive set).
#include "ocd_aerodynamics_models_cpp/default.hpp"
#include "ocd_drivetrain_wheel_torque_cpp/drivetrain_direct_torque_model.hpp"
#include "ocd_steering_actuator_pt1_cpp/steering_actuator_pt1_model.hpp"
#include "ocd_tire_models_cpp/mf_simple.hpp"
#include "ocd_types_cpp/types.hpp"
#include "ocd_vehicle_dynamics_single_track_cpp/vehicle_dynamics_model.hpp"
#include "ocd_vehicle_model_cpp/vehicle_model.hpp"

#include "fmi3Functions.h"

using OcdVehicle = tam::ocd::VehicleModel<
  tam::ocd::drivetrain::DrivetrainWheelTorqueModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsSingleTrackModel<
    tam::ocd::tire_models::MF_Simple, tam::ocd::aerodynamics::DefaultAerodynamicsModel>>;

namespace
{
// The model's internal fixed integration step (from its param defaults,
// `integration_step_size_s`). `step()` advances the ODE by exactly this.
constexpr double INTERNAL_STEP_S = 0.0008;

// v1 pedal -> per-wheel drive/brake torque mapping. Rough but enough to move the
// car; real feel is tuned once it drives in-sim (see README open items).
constexpr double DRIVE_TORQUE_NM = 500.0;  // per wheel at full throttle
constexpr double BRAKE_TORQUE_NM = 1500.0;  // per wheel at full brake

// Value references -- MUST match the swarm scenario binding + modelDescription.xml.
enum ValueRef : fmi3ValueReference {
  VR_STEER = 1,
  VR_THROTTLE = 2,
  VR_BRAKE = 3,
  VR_GROUND_HEIGHT = 4,
  VR_GROUND_FRICTION = 5,
  VR_X = 10,
  VR_Y = 11,
  VR_Z = 12,
  VR_YAW = 13,
};

struct Instance {
  OcdVehicle veh;
  // Latched inputs (set between do_steps).
  double steer_rad = 0.0;
  double throttle = 0.0;  // 0..1
  double brake = 0.0;     // 0..1
  double ground_height_m = 0.0;
  double ground_friction = 1.0;
  double time_s = 0.0;
  std::string name;
  fmi3LogMessageCallback log = nullptr;
  fmi3InstanceEnvironment env = nullptr;
};

using DataPerWheel = tam::types::common::DataPerWheel<double>;
}  // namespace

extern "C" {

fmi3Instance fmi3InstantiateCoSimulation(
  fmi3String instanceName, fmi3String /*instantiationToken*/, fmi3String /*resourcePath*/,
  fmi3Boolean /*visible*/, fmi3Boolean /*loggingOn*/, fmi3Boolean /*eventModeUsed*/,
  fmi3Boolean /*earlyReturnAllowed*/, const fmi3ValueReference[] /*required*/, size_t /*nRequired*/,
  fmi3InstanceEnvironment instanceEnvironment, fmi3LogMessageCallback logMessage,
  fmi3IntermediateUpdateCallback /*intermediateUpdate*/)
{
  Instance * inst = new (std::nothrow) Instance();
  if (inst == nullptr) {
    return nullptr;
  }
  inst->name = instanceName != nullptr ? instanceName : "opencardynamics";
  inst->log = logMessage;
  inst->env = instanceEnvironment;
  return inst;
}

void fmi3FreeInstance(fmi3Instance instance) { delete static_cast<Instance *>(instance); }

fmi3Status fmi3EnterInitializationMode(
  fmi3Instance /*instance*/, fmi3Boolean /*toleranceDefined*/, fmi3Float64 /*tolerance*/,
  fmi3Float64 /*startTime*/, fmi3Boolean /*stopTimeDefined*/, fmi3Float64 /*stopTime*/)
{
  return fmi3OK;
}

fmi3Status fmi3ExitInitializationMode(fmi3Instance instance)
{
  Instance * inst = static_cast<Instance *>(instance);
  // Apply the model's initial_state parameter defaults (at-rest, at origin).
  inst->veh.reset();
  inst->time_s = 0.0;
  return fmi3OK;
}

fmi3Status fmi3Terminate(fmi3Instance /*instance*/) { return fmi3OK; }

fmi3Status fmi3SetFloat64(
  fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences,
  const fmi3Float64 values[], size_t nValues)
{
  Instance * inst = static_cast<Instance *>(instance);
  if (nValueReferences != nValues) {
    return fmi3Error;
  }
  for (size_t i = 0; i < nValueReferences; ++i) {
    switch (valueReferences[i]) {
      case VR_STEER:
        inst->steer_rad = values[i];
        break;
      case VR_THROTTLE:
        inst->throttle = values[i];
        break;
      case VR_BRAKE:
        inst->brake = values[i];
        break;
      case VR_GROUND_HEIGHT:
        inst->ground_height_m = values[i];
        break;
      case VR_GROUND_FRICTION:
        inst->ground_friction = values[i];
        break;
      default:
        return fmi3Error;  // outputs / unknown are not settable
    }
  }
  return fmi3OK;
}

fmi3Status fmi3GetFloat64(
  fmi3Instance instance, const fmi3ValueReference valueReferences[], size_t nValueReferences,
  fmi3Float64 values[], size_t nValues)
{
  Instance * inst = static_cast<Instance *>(instance);
  if (nValueReferences != nValues) {
    return fmi3Error;
  }
  const auto out = inst->veh.get_vehicle_model_output().vehicle_dynamics_output;
  for (size_t i = 0; i < nValueReferences; ++i) {
    switch (valueReferences[i]) {
      case VR_X:
        values[i] = out.position_m.x;
        break;
      case VR_Y:
        values[i] = out.position_m.y;
        break;
      case VR_Z:
        values[i] = out.position_m.z;
        break;
      case VR_YAW:
        values[i] = out.orientation_rad.z;  // OCD's own frame; reconciled sim-side (slice C)
        break;
      default:
        return fmi3Error;  // inputs / unknown are not readable as outputs
    }
  }
  return fmi3OK;
}

fmi3Status fmi3DoStep(
  fmi3Instance instance, fmi3Float64 currentCommunicationPoint, fmi3Float64 communicationStepSize,
  fmi3Boolean /*noSetFMUStatePriorToCurrentPoint*/, fmi3Boolean * eventHandlingNeeded,
  fmi3Boolean * terminateSimulation, fmi3Boolean * earlyReturn, fmi3Float64 * lastSuccessfulTime)
{
  Instance * inst = static_cast<Instance *>(instance);

  // Steering: driver steering angle straight through (rad).
  OcdVehicle::SteeringActuatorDriverInputType steer_in{};
  steer_in.steering_angle_rad = inst->steer_rad;
  inst->veh.set_steering_input(steer_in);

  // Drivetrain: net per-wheel torque = throttle drive minus brake (v1 mapping).
  OcdVehicle::DrivetrainDriverInputType dt_in{};
  const double torque_nm = inst->throttle * DRIVE_TORQUE_NM - inst->brake * BRAKE_TORQUE_NM;
  dt_in.drivetrain_input_torque_per_wheel_Nm = DataPerWheel(torque_nm);
  inst->veh.set_drivetrain_input(dt_in);

  // Ground: single-point height + friction, applied to all four wheels (v1;
  // per-wheel is v2). normal_z has no OCD counterpart and is intentionally unbound.
  tam::ocd::types::ExternalInfluences ext;
  ext.z_height_road_m = DataPerWheel(inst->ground_height_m);
  ext.lambda_mue = DataPerWheel(inst->ground_friction);
  inst->veh.set_external_influences(ext);

  // Advance the ODE to cover the communication step. `step()` is a fixed
  // INTERNAL_STEP_S advance, so take the whole number of sub-steps that fits.
  long n = std::lround(communicationStepSize / INTERNAL_STEP_S);
  if (n < 1) {
    n = 1;
  }
  for (long i = 0; i < n; ++i) {
    inst->veh.step();
  }
  inst->time_s = currentCommunicationPoint + static_cast<double>(n) * INTERNAL_STEP_S;

  if (eventHandlingNeeded != nullptr) {
    *eventHandlingNeeded = fmi3False;
  }
  if (terminateSimulation != nullptr) {
    *terminateSimulation = fmi3False;
  }
  if (earlyReturn != nullptr) {
    *earlyReturn = fmi3False;
  }
  if (lastSuccessfulTime != nullptr) {
    // Advisory: lands on the internal 0.0008 s grid, not exactly current+step.
    *lastSuccessfulTime = inst->time_s;
  }
  return fmi3OK;
}

}  // extern "C"
