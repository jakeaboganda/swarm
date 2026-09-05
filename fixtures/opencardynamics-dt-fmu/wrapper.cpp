// FMI 3.0 Co-Simulation wrapper around a concrete DOUBLE-TRACK TUMFTM
// Open-Car-Dynamics vehicle model. Unlike the single-track wrapper, this exposes
// ROLL and PITCH (the double-track model integrates them as real DOF) and accepts
// a road BANK / GRADE input, which is injected as an external body force so the
// car's own roll dynamics respond to a canted road.
//
// Concrete combo: WHEEL_TORQUE drivetrain x PT1 steering actuator x DOUBLE_TRACK
// vehicle dynamics x MF52 tire x DEFAULT aero. Parameter defaults are the model's
// declared (AV21-derived) values -- no external param file is loaded.
//
// Roll/pitch are NOT in `VehicleModelOutput` (it exposes only yaw + planar x/y),
// so they are read from the model's logger, which publishes every state under
// `vehicle_dynamics/x_vec/<name>` (phi_rad = roll, theta_rad = pitch, z_m = heave).

#include <cmath>
#include <cstring>
#include <new>
#include <string>
#include <unordered_map>
#include <variant>

// OCD model headers (the concrete combo's transitive set).
#include "ocd_aerodynamics_models_cpp/default.hpp"
#include "ocd_drivetrain_wheel_torque_cpp/drivetrain_direct_torque_model.hpp"
#include "ocd_steering_actuator_pt1_cpp/steering_actuator_pt1_model.hpp"
#include "ocd_tire_models_cpp/mf_52.hpp"
#include "ocd_types_cpp/types.hpp"
#include "ocd_vehicle_dynamics_double_track_cpp/vehicle_dynamics_model.hpp"
#include "ocd_vehicle_model_cpp/vehicle_model.hpp"

#include "fmi3Functions.h"

using OcdVehicle = tam::ocd::VehicleModel<
  tam::ocd::drivetrain::DrivetrainWheelTorqueModel,
  tam::ocd::steering_actuator::PT1SteeringActuatorModel,
  tam::ocd::vehicle_dynamics::VehicleDynamicsDoubleTrackModel<
    tam::ocd::tire_models::MF52, tam::ocd::aerodynamics::DefaultAerodynamicsModel>>;

namespace
{
constexpr double INTERNAL_STEP_S = 0.0008;
constexpr double DRIVE_TORQUE_NM = 500.0;
constexpr double BRAKE_TORQUE_NM = 1500.0;

// Gravity, and the nominal vehicle mass (the model's `mass_vehicle_kg` default).
// Used only to size the injected bank force; the injection is an approximation
// of "gravity along a canted road", so the nominal mass is sufficient.
constexpr double G = 9.81;
constexpr double MASS_KG = 800.0;

// Logger keys for the roll/pitch/heave states (double-track publishes every
// state as `vehicle_dynamics/x_vec/<name>`).
const char * KEY_ROLL = "vehicle_dynamics/x_vec/phi_rad";
const char * KEY_PITCH = "vehicle_dynamics/x_vec/theta_rad";
const char * KEY_HEAVE = "vehicle_dynamics/x_vec/z_m";

enum ValueRef : fmi3ValueReference {
  VR_STEER = 1,
  VR_THROTTLE = 2,
  VR_BRAKE = 3,
  VR_GROUND_HEIGHT = 4,
  VR_GROUND_FRICTION = 5,
  VR_BANK = 6,   // road bank / superelevation angle at the car (rad)
  VR_GRADE = 7,  // road longitudinal grade at the car (rad)
  VR_X = 10,
  VR_Y = 11,
  VR_Z = 12,
  VR_YAW = 13,
  VR_ROLL = 14,
  VR_PITCH = 15,
};

using DataPerWheel = tam::types::common::DataPerWheel<double>;

struct Instance {
  OcdVehicle veh;
  double steer_rad = 0.0;
  double throttle = 0.0;
  double brake = 0.0;
  double ground_height_m = 0.0;
  double ground_friction = 1.0;
  double bank_rad = 0.0;
  double grade_rad = 0.0;
  // Cached outputs read from the logger after each step.
  double roll_rad = 0.0;
  double pitch_rad = 0.0;
  double heave_m = 0.0;
  double internal_time_s = 0.0;
  double carry_s = 0.0;
  std::string name;
  fmi3LogMessageCallback log = nullptr;
  fmi3InstanceEnvironment env = nullptr;
};

// Pull one logged double out of the model's data map (0 if absent).
double logged(const tam::tsl::data_map_t & m, const char * key)
{
  auto it = m.find(key);
  if (it == m.end()) {
    return 0.0;
  }
  if (const double * v = std::get_if<double>(&it->second)) {
    return *v;
  }
  return 0.0;
}
}  // namespace

extern "C" {

const char * fmi3GetVersion(void) { return fmi3Version; }

fmi3Status fmi3SetDebugLogging(
  fmi3Instance /*instance*/, fmi3Boolean /*loggingOn*/, size_t /*nCategories*/,
  const fmi3String[] /*categories*/)
{
  return fmi3OK;
}

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
  inst->name = instanceName != nullptr ? instanceName : "opencardynamics_dt";
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

// Soften the roll stiffness from the AV21 racecar's values to a road car's, so
// the body actually leans a few degrees in a corner (the stiff racecar rolls
// well under 1 deg -- correct, but invisible). Applied after reset(), which
// restores the declared defaults. This is a demo/road-car tuning, documented in
// the README.
static void soften_roll(Instance * inst)
{
  auto pm = inst->veh.get_param_manager();
  auto set = [&](const char * name, double v) {
    if (pm->has_parameter(name)) pm->set_value(name, v);
  };
  const char * P = "vehicle_dynamics_double_track.suspension.";
  auto key = [&](const char * s){ return std::string(P) + s; };
  set(key("antirollbar_virtual_spring_stiffness_Npm.front").c_str(), 1200.0);
  set(key("antirollbar_virtual_spring_stiffness_Npm.rear").c_str(), 700.0);
  set(key("vehicle_spring_stiffness_Npm.front").c_str(), 32000.0);
  set(key("vehicle_spring_stiffness_Npm.rear").c_str(), 26000.0);
}

fmi3Status fmi3ExitInitializationMode(fmi3Instance instance)
{
  Instance * inst = static_cast<Instance *>(instance);
  inst->veh.reset();
  soften_roll(inst);
  inst->internal_time_s = 0.0;
  inst->carry_s = 0.0;
  return fmi3OK;
}

fmi3Status fmi3Terminate(fmi3Instance /*instance*/) { return fmi3OK; }

fmi3Status fmi3Reset(fmi3Instance instance)
{
  Instance * inst = static_cast<Instance *>(instance);
  inst->veh.reset();
  inst->steer_rad = 0.0;
  inst->throttle = 0.0;
  inst->brake = 0.0;
  inst->ground_height_m = 0.0;
  inst->ground_friction = 1.0;
  inst->bank_rad = 0.0;
  inst->grade_rad = 0.0;
  inst->roll_rad = 0.0;
  inst->pitch_rad = 0.0;
  inst->heave_m = 0.0;
  inst->internal_time_s = 0.0;
  inst->carry_s = 0.0;
  return fmi3OK;
}

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
      case VR_STEER: inst->steer_rad = values[i]; break;
      case VR_THROTTLE: inst->throttle = values[i]; break;
      case VR_BRAKE: inst->brake = values[i]; break;
      case VR_GROUND_HEIGHT: inst->ground_height_m = values[i]; break;
      case VR_GROUND_FRICTION: inst->ground_friction = values[i]; break;
      case VR_BANK: inst->bank_rad = values[i]; break;
      case VR_GRADE: inst->grade_rad = values[i]; break;
      default: return fmi3Error;
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
      case VR_X: values[i] = out.position_m.x; break;
      case VR_Y: values[i] = out.position_m.y; break;
      case VR_Z: values[i] = inst->heave_m; break;  // z_m heave (not in output struct)
      case VR_YAW: values[i] = out.orientation_rad.z; break;
      case VR_ROLL: values[i] = inst->roll_rad; break;
      case VR_PITCH: values[i] = inst->pitch_rad; break;
      default: return fmi3Error;
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

  OcdVehicle::SteeringActuatorDriverInputType steer_in{};
  steer_in.steering_angle_rad = inst->steer_rad;
  inst->veh.set_steering_input(steer_in);

  OcdVehicle::DrivetrainDriverInputType dt_in{};
  const double torque_nm = inst->throttle * DRIVE_TORQUE_NM - inst->brake * BRAKE_TORQUE_NM;
  dt_in.drivetrain_input_torque_per_wheel_Nm = DataPerWheel(torque_nm);
  inst->veh.set_drivetrain_input(dt_in);

  // External influences: ground + the BANK/GRADE injection. On a road canted by
  // `bank_rad`, gravity has a component along the surface pointing toward the
  // low (inside) edge -- inject it as a body-frame lateral force so the model's
  // roll DOF responds. Grade injects a longitudinal force. Vehicle frame is
  // x-forward, y-left; a positive bank (road tilts up to the left) pushes the
  // car to the right (-y). Sign confirmed empirically by the orchestrator.
  tam::ocd::types::ExternalInfluences ext;
  ext.z_height_road_m = DataPerWheel(inst->ground_height_m);
  ext.lambda_mue = DataPerWheel(inst->ground_friction);
  ext.external_force_N.y = -MASS_KG * G * std::sin(inst->bank_rad);
  ext.external_force_N.x = -MASS_KG * G * std::sin(inst->grade_rad);
  inst->veh.set_external_influences(ext);

  inst->carry_s += communicationStepSize;
  long n = static_cast<long>(std::floor(inst->carry_s / INTERNAL_STEP_S));
  if (n < 0) {
    n = 0;
  }
  for (long i = 0; i < n; ++i) {
    inst->veh.step();
  }
  inst->carry_s -= static_cast<double>(n) * INTERNAL_STEP_S;
  inst->internal_time_s += static_cast<double>(n) * INTERNAL_STEP_S;

  // Read roll/pitch/heave out of the model logger (not exposed in the output).
  tam::tsl::data_map_t data;
  inst->veh.get_logger()->get_data(data, "");
  inst->roll_rad = logged(data, KEY_ROLL);
  inst->pitch_rad = logged(data, KEY_PITCH);
  inst->heave_m = logged(data, KEY_HEAVE);

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
    *lastSuccessfulTime = inst->internal_time_s;
  }
  return fmi3OK;
}

}  // extern "C"
