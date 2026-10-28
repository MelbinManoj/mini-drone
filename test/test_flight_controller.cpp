// Host-side tests for the flight-control logic, including a simple
// physics simulation of one axis to check that the PID levels the drone.
// Build & run:  g++ -std=c++11 -I../mini_drone test_flight_controller.cpp -o t && ./t
#include <cstdio>
#include <cmath>
#include "flight_controller.h"

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while (0)

int main() {
  // 1. Attitude: accelerometer gives the right angle when the drone is tilted 30 deg right
  { AttitudeEstimator a;
    float s = sinf(30 * M_PI / 180), c = cosf(30 * M_PI / 180);
    for (int i = 0; i < 500; i++) a.update(0, 0, 0, s, c, 0.004f);
    CHECK(fabsf(a.getRoll() - 30) < 0.5f); CHECK(fabsf(a.getPitch()) < 0.5f); }

  // 2. Gyro bias drift is corrected by the accelerometer (stays bounded, does not run away)
  { AttitudeEstimator a;
    for (int i = 0; i < 5000; i++) a.update(2.0f, 0, 0, 0, 1, 0.004f);  // 2 deg/s bias for 20 s
    CHECK(fabsf(a.getRoll()) < 1.0f); }

  // 3. Mixer: rolling right-side-down needs more thrust on the right motors
  { int m[4]; mixMotors(120, -20, 0, 0, m);
    CHECK(m[1] > m[0] && m[3] > m[2]);
    mixMotors(120, 0, 20, 0, m);            // positive pitch correction (nose too low) -> front motors faster
    CHECK(m[0] > m[2] && m[1] > m[3]);
    mixMotors(250, 30, 30, 0, m);           // outputs are clamped to PWM range
    for (int i = 0; i < 4; i++) CHECK(m[i] >= 0 && m[i] <= 255);
    mixMotors(120, 0, 0, 0, m);
    CHECK(m[0] == 120 && m[1] == 120 && m[2] == 120 && m[3] == 120); }

  // 4. Closed-loop simulation of the roll axis: drone starts tilted 15 deg, PID must level it
  { PID pid(1.6f, 0.4f, 0.05f, 50.0f);
    float angle = 15, rate = 0, dt = 0.004f;
    const float inertiaGain = 40.0f;        // deg/s^2 per unit of motor differential (toy model)
    float worst = 0;
    for (int i = 0; i < 1250; i++) {        // 5 seconds
      float out = pid.update(0, angle, dt);
      int m[4]; mixMotors(130, out, 0, 0, m);
      float torque = ((m[0] + m[2]) - (m[1] + m[3])) / 2.0f;  // left minus right thrust
      // more thrust on the left lifts the left side -> right side goes down -> roll increases
      rate += torque * inertiaGain * 0.1f * dt - rate * 2.0f * dt;
      angle += rate * dt;
      if (i > 250 && fabsf(angle) > worst) worst = fabsf(angle);   // overshoot after the first second
    }
    CHECK(fabsf(angle) < 0.5f); CHECK(worst < 8.0f);
    printf("  roll after 5 s of stabilisation: %.2f deg (max overshoot %.1f deg)\n", angle, worst); }

  // 5. Safety: arming needs zero throttle; tilt over 45 deg cuts motors; lost link lands slowly
  { SafetyMonitor s(1000, 45);
    CHECK(!s.requestArm(100, 0)); CHECK(!s.isArmed());
    CHECK(s.requestArm(0, 0)); CHECK(s.isArmed());
    CHECK(s.check(150, 0, 0, 500) == 150);
    CHECK(s.check(150, 0, 0, 1500) < 150 && s.check(150, 0, 0, 1500) > 0);  // descending
    CHECK(s.check(150, 0, 0, 5000) == 0); CHECK(!s.isArmed());              // landed + disarmed
    s.requestArm(0, 6000);
    CHECK(s.check(150, 60, 0, 6100) == 0); CHECK(!s.isArmed()); }           // crash cutoff

  if (failures == 0) printf("All flight controller tests passed.\n");
  return failures == 0 ? 0 : 1;
}
