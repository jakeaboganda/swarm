// Standalone proof harness for the double-track OCD FMU. Loads the built .so and
// drives THREE conditions, printing roll each, to prove: (1) straight -> ~0 roll,
// (2) cornering -> nonzero roll (natural load-transfer lean), (3) cornering +
// injected road bank -> roll differs from (2) (the bank couples in).
//
// Not an FMI importer: it dlopens the library and calls the fmi3 C entry points
// directly (the same ABI the swarm loader uses).

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>

#include "fmi3Functions.h"

namespace
{
constexpr double DT = 1.0 / 64.0;
constexpr double RAD2DEG = 57.29577951308232;

enum Vr : fmi3ValueReference {
  STEER = 1, THROTTLE = 2, BRAKE = 3, GROUND_HEIGHT = 4, GROUND_FRICTION = 5,
  BANK = 6, GRADE = 7, X = 10, Y = 11, Z = 12, YAW = 13, ROLL = 14, PITCH = 15,
};

void * sym(void * lib, const char * name)
{
  void * s = dlsym(lib, name);
  if (s == nullptr) {
    std::fprintf(stderr, "missing symbol %s\n", name);
    std::exit(2);
  }
  return s;
}
}  // namespace

int main(int argc, char ** argv)
{
  const char * path = argc > 1 ? argv[1] : "build/opencardynamics_dt.so";
  void * lib = dlopen(path, RTLD_NOW | RTLD_LOCAL);
  if (lib == nullptr) {
    std::fprintf(stderr, "dlopen failed: %s\n", dlerror());
    return 2;
  }

  auto instantiate = reinterpret_cast<fmi3InstantiateCoSimulationTYPE *>(sym(lib, "fmi3InstantiateCoSimulation"));
  auto enter_init = reinterpret_cast<fmi3EnterInitializationModeTYPE *>(sym(lib, "fmi3EnterInitializationMode"));
  auto exit_init = reinterpret_cast<fmi3ExitInitializationModeTYPE *>(sym(lib, "fmi3ExitInitializationMode"));
  auto set_f64 = reinterpret_cast<fmi3SetFloat64TYPE *>(sym(lib, "fmi3SetFloat64"));
  auto get_f64 = reinterpret_cast<fmi3GetFloat64TYPE *>(sym(lib, "fmi3GetFloat64"));
  auto do_step = reinterpret_cast<fmi3DoStepTYPE *>(sym(lib, "fmi3DoStep"));
  auto reset = reinterpret_cast<fmi3ResetTYPE *>(sym(lib, "fmi3Reset"));
  auto free_inst = reinterpret_cast<fmi3FreeInstanceTYPE *>(sym(lib, "fmi3FreeInstance"));

  fmi3Instance c = instantiate("dt", "tok", nullptr, fmi3False, fmi3False, fmi3False, fmi3False, nullptr, 0, nullptr, nullptr, nullptr);
  if (c == nullptr) { std::fprintf(stderr, "instantiate failed\n"); return 2; }

  auto set1 = [&](fmi3ValueReference vr, double v) { fmi3ValueReference r = vr; set_f64(c, &r, 1, &v, 1); };
  auto get1 = [&](fmi3ValueReference vr) { fmi3ValueReference r = vr; double v = 0; get_f64(c, &r, 1, &v, 1); return v; };

  // One condition: reset, hold steer/throttle/bank for `secs`, return final roll.
  auto run = [&](double steer, double throttle, double bank, double secs) {
    reset(c);
    enter_init(c, fmi3False, 0, 0, fmi3False, 0);
    exit_init(c);
    set1(GROUND_FRICTION, 1.0);
    double t = 0.0;
    const long steps = static_cast<long>(secs / DT);
    for (long i = 0; i < steps; ++i) {
      set1(THROTTLE, throttle);
      set1(STEER, steer);
      set1(BANK, bank);
      fmi3Boolean e = fmi3False, term = fmi3False, er = fmi3False; fmi3Float64 lst = 0;
      do_step(c, t, DT, fmi3True, &e, &term, &er, &lst);
      t += DT;
    }
    struct R { double roll, pitch, yaw, x, y, speed_proxy; };
    return R{get1(ROLL), get1(PITCH), get1(YAW), get1(X), get1(Y), 0.0};
  };

  // (1) straight, no bank -- baseline roll ~ 0.
  auto s1 = run(0.0, 0.6, 0.0, 4.0);
  // (2) cornering on flat -- natural load-transfer lean.
  auto s2 = run(0.10, 0.6, 0.0, 4.0);
  // (3) same corner + a 0.15 rad (~8.6 deg) road bank injected.
  auto s3 = run(0.10, 0.6, 0.15, 4.0);

  std::printf("(1) straight,     no bank : roll=%+7.3f deg  yaw=%+7.3f  x=%7.2f y=%7.2f\n", s1.roll * RAD2DEG, s1.yaw * RAD2DEG, s1.x, s1.y);
  std::printf("(2) corner,       no bank : roll=%+7.3f deg  yaw=%+7.3f  x=%7.2f y=%7.2f\n", s2.roll * RAD2DEG, s2.yaw * RAD2DEG, s2.x, s2.y);
  std::printf("(3) corner, bank=0.15 rad : roll=%+7.3f deg  yaw=%+7.3f  x=%7.2f y=%7.2f\n", s3.roll * RAD2DEG, s3.yaw * RAD2DEG, s3.x, s3.y);

  const double roll2 = std::fabs(s2.roll * RAD2DEG);
  const double dbank = std::fabs((s3.roll - s2.roll) * RAD2DEG);
  bool pass = true;
  if (!(roll2 > 0.05)) { std::printf("FAIL: cornering roll (%.3f deg) is not materially nonzero\n", roll2); pass = false; }
  if (!(dbank > 0.02)) { std::printf("FAIL: injected bank did not change roll (%.3f deg)\n", dbank); pass = false; }

  free_inst(c);
  dlclose(lib);
  if (pass) {
    std::printf("PASS: double-track leans in corners AND the bank injection couples in.\n");
    return 0;
  }
  return 1;
}
