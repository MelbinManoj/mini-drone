// Mini Drone Flight Controller - Arduino Uno / Nano
// Stabilises a small quadcopter with brushed coreless motors (e.g. 8520) and
// an MPU-6050 IMU. Throttle and arming commands arrive over serial, so you can
// fly it from a PC or from a phone through an HC-05 Bluetooth module.
//
// Wiring:
//   MPU-6050: SDA -> A4, SCL -> A5, VCC -> 5V (or 3.3V), GND -> GND
//   Motor MOSFET gates (e.g. SI2302 + flyback diode per motor):
//     front-left -> D3, front-right -> D9, rear-left -> D10, rear-right -> D11
//   HC-05 (optional): TX -> D0 (RX), RX -> D1 via a voltage divider, 9600 baud
//   1S LiPo powers the motors; the board via a 5 V boost converter (common GND)
//
// Serial commands (end each with a newline):
//   A        arm (throttle must be 0)        D        disarm
//   T<0-255> set throttle, e.g. T140         R<deg>   roll target   P<deg> pitch target
// Commands must arrive at least every 1 s, otherwise the drone slowly lands.

#include <Wire.h>
#include "flight_controller.h"

const uint8_t MOTOR_PIN[4] = {3, 9, 10, 11};
const uint8_t MPU_ADDR = 0x68;

AttitudeEstimator attitude(0.98f);
PID rollPid(1.6f, 0.4f, 0.05f, 50.0f);
PID pitchPid(1.6f, 0.4f, 0.05f, 50.0f);
PID yawRatePid(1.2f, 0.2f, 0.0f, 50.0f);
SafetyMonitor safety(1000UL, 45.0f);

int throttleCmd = 0;
float rollTarget = 0, pitchTarget = 0;
float gyroBias[3] = {0, 0, 0};
unsigned long lastLoop = 0, lastPrint = 0;
char cmdBuf[12]; uint8_t cmdLen = 0;

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

// Reads accel (g) and gyro (deg/s). Returns false if the sensor did not answer.
bool readImu(float acc[3], float gyro[3]) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;
  int16_t v[7];
  for (int i = 0; i < 7; i++) v[i] = (Wire.read() << 8) | Wire.read();
  for (int i = 0; i < 3; i++) acc[i] = v[i] / 16384.0f;             // +/-2 g range
  for (int i = 0; i < 3; i++) gyro[i] = v[i + 4] / 65.5f - gyroBias[i];  // +/-500 deg/s
  return true;
}

void writeMotors(const int m[4]) { for (int i = 0; i < 4; i++) analogWrite(MOTOR_PIN[i], m[i]); }
void stopMotors() { const int z[4] = {0, 0, 0, 0}; writeMotors(z); }

void handleCommand(const char* c) {
  safety.commandReceived(millis());
  switch (c[0]) {
    case 'A': Serial.println(safety.requestArm(throttleCmd, millis()) ? F("ARMED") : F("Lower throttle to arm")); break;
    case 'D': safety.disarm(); throttleCmd = 0; Serial.println(F("DISARMED")); break;
    case 'T': throttleCmd = constrain(atoi(c + 1), 0, 255); break;
    case 'R': rollTarget = constrain(atof(c + 1), -20, 20); break;
    case 'P': pitchTarget = constrain(atof(c + 1), -20, 20); break;
    default: break;   // empty line works as a keep-alive
  }
}

void readSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmdLen > 0 || ch == '\n') { cmdBuf[cmdLen] = 0; handleCommand(cmdBuf); }
      cmdLen = 0;
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = ch;
    }
  }
}

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 4; i++) pinMode(MOTOR_PIN[i], OUTPUT);
  stopMotors();
  Wire.begin();
  Wire.setClock(400000);
  mpuWrite(0x6B, 0x00);   // wake up
  mpuWrite(0x1A, 0x03);   // 44 Hz digital low-pass filter
  mpuWrite(0x1B, 0x08);   // gyro +/-500 deg/s
  mpuWrite(0x1C, 0x00);   // accel +/-2 g

  Serial.println(F("Calibrating gyro - keep the drone still..."));
  float acc[3], gyro[3], sum[3] = {0, 0, 0};
  for (int n = 0; n < 500; n++) {
    if (readImu(acc, gyro)) for (int i = 0; i < 3; i++) sum[i] += gyro[i];
    delay(2);
  }
  for (int i = 0; i < 3; i++) gyroBias[i] = sum[i] / 500;
  Serial.println(F("Ready. Send A to arm, T<0-255> for throttle."));
  lastLoop = micros();
}

void loop() {
  readSerial();

  unsigned long nowUs = micros();
  if (nowUs - lastLoop < 4000) return;          // 250 Hz control loop
  float dt = (nowUs - lastLoop) / 1000000.0f;
  lastLoop = nowUs;

  float acc[3], gyro[3];
  if (!readImu(acc, gyro)) { safety.disarm(); stopMotors(); return; }  // IMU lost: cut motors
  attitude.update(gyro[0], gyro[1], acc[0], acc[1], acc[2], dt);

  int throttle = safety.check(throttleCmd, attitude.getRoll(), attitude.getPitch(), millis());
  if (throttle < 20) {                          // idle on the ground: no corrections
    rollPid.reset(); pitchPid.reset(); yawRatePid.reset();
    stopMotors();
  } else {
    float r = rollPid.update(rollTarget, attitude.getRoll(), dt);
    float p = pitchPid.update(pitchTarget, attitude.getPitch(), dt);
    float y = yawRatePid.update(0, gyro[2], dt);  // hold heading (zero yaw rate)
    int m[4];
    mixMotors(throttle, r, p, y, m);
    writeMotors(m);
  }

  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.print(F("roll=")); Serial.print(attitude.getRoll(), 1);
    Serial.print(F(" pitch=")); Serial.print(attitude.getPitch(), 1);
    Serial.print(F(" thr=")); Serial.print(throttle);
    Serial.println(safety.isArmed() ? F(" ARMED") : F(" disarmed"));
  }
}
