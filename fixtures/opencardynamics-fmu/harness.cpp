// Standalone proof that the built opencardynamics.so is a working FMI 3.0 CS
// FMU: dlopen it, drive it through the real fmi3 C API, and assert the vehicle
// actually moves under throttle and yaws under steer. No Rust, no swarm -- this
// is the slice-A gate.

#include <dlfcn.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "fmi3Functions.h"

namespace
{
constexpr fmi3ValueReference VR_STEER = 1;
constexpr fmi3ValueReference VR_THROTTLE = 2;
constexpr fmi3ValueReference VR_X = 10;
constexpr fmi3ValueReference VR_Y = 11;
constexpr fmi3ValueReference VR_YAW = 13;
constexpr double DT = 1.0 / 64.0;  // the swarm sim tick

template <typename T>
T load(void * handle, const char * name)
{
  T sym = reinterpret_cast<T>(dlsym(handle, name));
  if (sym == nullptr) {
    std::fprintf(stderr, "missing symbol: %s\n", name);
    std::exit(2);
  }
  return sym;
}

void set1(fmi3SetFloat64TYPE * set, fmi3Instance inst, fmi3ValueReference vr, double v)
{
  const fmi3ValueReference vrs[1] = {vr};
  const fmi3Float64 vals[1] = {v};
  set(inst, vrs, 1, vals, 1);
}

double get1(fmi3GetFloat64TYPE * get, fmi3Instance inst, fmi3ValueReference vr)
{
  const fmi3ValueReference vrs[1] = {vr};
  fmi3Float64 vals[1] = {0.0};
  get(inst, vrs, 1, vals, 1);
  return vals[0];
}
}  // namespace

int main(int argc, char ** argv)
{
  const char * so = argc > 1 ? argv[1] : "./build/opencardynamics.so";
  void * h = dlopen(so, RTLD_NOW | RTLD_LOCAL);
  if (h == nullptr) {
    std::fprintf(stderr, "dlopen(%s) failed: %s\n", so, dlerror());
    return 2;
  }

  auto * instantiate = load<fmi3InstantiateCoSimulationTYPE *>(h, "fmi3InstantiateCoSimulation");
  auto * enter_init = load<fmi3EnterInitializationModeTYPE *>(h, "fmi3EnterInitializationMode");
  auto * exit_init = load<fmi3ExitInitializationModeTYPE *>(h, "fmi3ExitInitializationMode");
  auto * set = load<fmi3SetFloat64TYPE *>(h, "fmi3SetFloat64");
  auto * get = load<fmi3GetFloat64TYPE *>(h, "fmi3GetFloat64");
  auto * do_step = load<fmi3DoStepTYPE *>(h, "fmi3DoStep");
  auto * terminate = load<fmi3TerminateTYPE *>(h, "fmi3Terminate");
  auto * free_instance = load<fmi3FreeInstanceTYPE *>(h, "fmi3FreeInstance");

  fmi3Instance inst = instantiate(
    "ocd-harness", "opencardynamics", nullptr, fmi3False, fmi3False, fmi3False, fmi3False, nullptr,
    0, nullptr, nullptr, nullptr);
  if (inst == nullptr) {
    std::fprintf(stderr, "instantiate returned null\n");
    return 2;
  }
  enter_init(inst, fmi3False, 0.0, 0.0, fmi3False, 0.0);
  exit_init(inst);

  // Phase 1: full throttle, straight, ~2 s. Expect forward position to grow.
  const double x0 = get1(get, inst, VR_X);
  double t = 0.0;
  set1(set, inst, VR_STEER, 0.0);
  set1(set, inst, VR_THROTTLE, 1.0);
  const int steps = static_cast<int>(2.0 / DT);
  for (int i = 0; i < steps; ++i) {
    fmi3Boolean ev = fmi3False, term = fmi3False, early = fmi3False;
    fmi3Float64 last = 0.0;
    do_step(inst, t, DT, fmi3True, &ev, &term, &early, &last);
    t += DT;
    if (i % 16 == 0) {
      std::printf(
        "t=%.3f x=%.4f y=%.4f yaw=%.5f\n", t, get1(get, inst, VR_X), get1(get, inst, VR_Y),
        get1(get, inst, VR_YAW));
    }
  }
  const double x1 = get1(get, inst, VR_X);
  const double dist = std::hypot(x1 - x0, get1(get, inst, VR_Y));

  // Phase 2: hold a steering angle, ~1 s. Expect the yaw to change.
  const double yaw0 = get1(get, inst, VR_YAW);
  set1(set, inst, VR_STEER, 0.15);  // rad
  set1(set, inst, VR_THROTTLE, 0.5);
  const int steer_steps = static_cast<int>(1.0 / DT);
  for (int i = 0; i < steer_steps; ++i) {
    fmi3Boolean ev = fmi3False, term = fmi3False, early = fmi3False;
    fmi3Float64 last = 0.0;
    do_step(inst, t, DT, fmi3True, &ev, &term, &early, &last);
    t += DT;
  }
  const double yaw_change = std::fabs(get1(get, inst, VR_YAW) - yaw0);

  terminate(inst);
  free_instance(inst);
  dlclose(h);

  std::printf("\nRESULT: forward distance under throttle = %.3f m; |yaw change| under steer = %.4f rad\n", dist, yaw_change);

  bool ok = true;
  if (!(dist > 1.0)) {
    std::fprintf(stderr, "FAIL: car did not accelerate forward (dist=%.3f m, expected > 1)\n", dist);
    ok = false;
  }
  if (!(yaw_change > 1e-3)) {
    std::fprintf(stderr, "FAIL: steering produced no yaw (|dyaw|=%.5f, expected > 1e-3)\n", yaw_change);
    ok = false;
  }
  if (ok) {
    std::printf("PASS: FMU loads and drives.\n");
  }
  return ok ? 0 : 1;
}
