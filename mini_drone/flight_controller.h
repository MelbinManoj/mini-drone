// Flight-control math for the mini quadcopter: attitude estimation,
// PID stabilisation, motor mixing and safety checks.
// No Arduino dependencies, so it can be unit-tested on a PC.
#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <math.h>
#include <stdint.h>

// Complementary filter: trusts the gyro short-term and the accelerometer long-term.
class AttitudeEstimator {
 public:
  explicit AttitudeEstimator(float alpha = 0.98f) : a(alpha), roll(0), pitch(0), init(false) {}

  // gyro in deg/s, accel in g, dt in seconds
  void update(float gx, float gy, float ax, float ay, float az, float dt) {
    float accRoll = atan2f(ay, az) * 57.2958f;
    float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2958f;
    if (!init) { roll = accRoll; pitch = accPitch; init = true; return; }
    roll = a * (roll + gx * dt) + (1 - a) * accRoll;
    pitch = a * (pitch + gy * dt) + (1 - a) * accPitch;
  }
  float getRoll() const { return roll; }
  float getPitch() const { return pitch; }

 private:
  float a, roll, pitch; bool init;
};

class PID {
 public:
  PID(float p, float i, float d, float iLimit) : kp(p), ki(i), kd(d), lim(iLimit), integ(0), prevErr(0), first(true) {}
  float update(float setpoint, float measured, float dt) {
    float err = setpoint - measured;
    integ += err * dt;
    if (integ > lim) integ = lim;          // anti-windup
    if (integ < -lim) integ = -lim;
    float deriv = first ? 0 : (err - prevErr) / dt;
    prevErr = err; first = false;
    return kp * err + ki * integ + kd * deriv;
  }
  void reset() { integ = 0; prevErr = 0; first = true; }
 private:
  float kp, ki, kd, lim, integ, prevErr; bool first;
};

// Quad "X" layout seen from above, front at the top:
//   M0 front-left (CW)   M1 front-right (CCW)
//   M2 rear-left (CCW)   M3 rear-right (CW)
// Positive roll = right side down, positive pitch = nose up.
inline void mixMotors(float throttle, float rollOut, float pitchOut, float yawOut, int out[4]) {
  float m[4];
  m[0] = throttle + rollOut + pitchOut - yawOut;   // front-left
  m[1] = throttle - rollOut + pitchOut + yawOut;   // front-right
  m[2] = throttle + rollOut - pitchOut + yawOut;   // rear-left
  m[3] = throttle - rollOut - pitchOut - yawOut;   // rear-right
  for (int i = 0; i < 4; i++) {
    if (m[i] < 0) m[i] = 0;
    if (m[i] > 255) m[i] = 255;
    out[i] = (int)(m[i] + 0.5f);
  }
}

// Arming and failsafe rules.
class SafetyMonitor {
 public:
  SafetyMonitor(uint32_t linkTimeoutMs, float maxTiltDeg)
      : timeout(linkTimeoutMs), maxTilt(maxTiltDeg), armed(false), lastCmd(0) {}

  bool requestArm(int throttle, uint32_t nowMs) {     // only arm at zero throttle
    if (throttle > 0) return false;
    armed = true; lastCmd = nowMs; return true;
  }
  void disarm() { armed = false; }
  void commandReceived(uint32_t nowMs) { lastCmd = nowMs; }

  // Returns the throttle that is actually allowed right now.
  int check(int throttle, float roll, float pitch, uint32_t nowMs) {
    if (!armed) return 0;
    if (fabsf(roll) > maxTilt || fabsf(pitch) > maxTilt) { armed = false; return 0; }  // crash cutoff
    if (nowMs - lastCmd > timeout) {                   // lost link: gentle descent
      int reduced = throttle - (int)((nowMs - lastCmd - timeout) / 20);
      if (reduced <= 0) { armed = false; return 0; }
      return reduced;
    }
    return throttle;
  }
  bool isArmed() const { return armed; }

 private:
  uint32_t timeout; float maxTilt; bool armed; uint32_t lastCmd;
};

#endif
