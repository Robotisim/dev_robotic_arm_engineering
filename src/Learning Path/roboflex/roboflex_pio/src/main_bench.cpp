#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Production channel map: arm joints = PCA9685 ch 1..4, gripper = ch 5.
// Physical movement has been confirmed for channels 1 and 2; observe the
// remaining joints individually while establishing the reference pose.
// These are servo command angles, not measured joint angles or ROS radians.
// The existing project's pulse calibration is retained: 150..600 at 50 Hz.
static constexpr uint8_t kChannels[] = {1, 2, 3, 4, 5};
static constexpr size_t kJointCount = sizeof(kChannels) / sizeof(kChannels[0]);
static constexpr float kMinDeg = 0.0f;
static constexpr float kMaxDeg[kJointCount] = {120.0f, 120.0f, 80.0f, 120.0f, 120.0f};
static constexpr float kMaxMoveDeg = 50.0f;
static constexpr float kStepDeg = 0.2f;  // 10 command degrees/second.
static constexpr uint32_t kStepMs = 20;
static constexpr uint32_t kLinkTimeoutMs = 2000;

Adafruit_PWMServoDriver pwm(0x40);
struct Joint {
  bool initialized = false;
  float command = 0;
  float target = 0;
};
Joint joints[kJointCount];
bool ready = false;
bool fault = false;
uint32_t lastStep = 0;
uint32_t lastContact = 0;
char line[80];
size_t lineLength = 0;
bool discardLine = false;

void holdCommands() {
  for (auto &joint : joints) joint.target = joint.command;
}

bool writeAngle(size_t index, float angle) {
  const uint16_t pulse = (uint16_t)lroundf(150.0f + angle * 450.0f / 180.0f);
  if (pwm.setPWM(kChannels[index], 0, pulse) != 0) {
    holdCommands();
    fault = true;
    Serial.println("ERR I2C write failed; motion commands locked until reset");
    return false;
  }
  return true;
}

void printStatus() {
  Serial.printf("STATUS ready=%d fault=%d feedback=none\n", ready, fault);
  for (size_t i = 0; i < kJointCount; ++i) {
    Serial.printf("CH %u enabled=%d command_deg=%.1f target_deg=%.1f\n",
                  kChannels[i], joints[i].initialized, joints[i].command, joints[i].target);
  }
}

bool isMoving() {
  for (const auto &joint : joints)
    if (joint.target != joint.command) return true;
  return false;
}

void processLine(char *input) {
  char *save = nullptr;
  char *cmd = strtok_r(input, " \t\r", &save);
  if (!cmd) return;
  char *channel = strtok_r(nullptr, " \t\r", &save);
  char *angleText = strtok_r(nullptr, " \t\r", &save);
  char *extra = strtok_r(nullptr, " \t\r", &save);

  if (!strcmp(cmd, "status") || !strcmp(cmd, "ping") || !strcmp(cmd, "stop")) {
    if (channel) { Serial.println("ERR unexpected argument"); return; }
    lastContact = millis();
    if (!strcmp(cmd, "stop")) {
      holdCommands();
      Serial.println("OK holding last PWM commands; this is not a power disconnect");
    } else if (!strcmp(cmd, "status")) printStatus();
    else Serial.println("PONG");
    return;
  }

  const bool initial = !strcmp(cmd, "init");
  if ((!initial && strcmp(cmd, "move")) || !channel || !angleText || extra) {
    Serial.println("ERR use: status | ping | stop | init <1..5> <deg> | move <1..5> <deg>");
    return;
  }
  if (strlen(channel) != 1 || channel[0] < '1' || channel[0] > '5') {
    Serial.println("ERR only channels 1..5 are allowed");
    return;
  }
  const size_t index = (size_t)(channel[0] - '1');
  char *end = nullptr;
  const float angle = strtof(angleText, &end);
  if (end == angleText || *end || !isfinite(angle) || angle < kMinDeg || angle > kMaxDeg[index]) {
    Serial.printf("ERR ch=%s angle must be finite in 0..%.0f degrees\n", channel, kMaxDeg[index]);
    return;
  }
  if (!ready || fault) { Serial.println("ERR driver not ready or fault latched"); return; }
  if (isMoving()) { Serial.println("ERR wait for current move or send stop"); return; }
  Joint &joint = joints[index];
  if (initial) {
    if (joint.initialized) { Serial.println("ERR already initialized; use move"); return; }
    // The first pulse sets an absolute target. Its travel and speed cannot be
    // bounded without knowing the physical starting position of the servo.
    if (!writeAngle(index, angle)) return;
    joint.initialized = true;
    joint.command = joint.target = angle;
    lastContact = millis();
    Serial.printf("INITIAL_TARGET ch=%s deg=%.1f actual_position=unknown\n", channel, angle);
  } else {
    if (!joint.initialized) { Serial.println("ERR initialize starting command first"); return; }
    if (fabsf(angle - joint.command) > kMaxMoveDeg) {
      Serial.println("ERR requested move exceeds 50 command degrees");
      return;
    }
    joint.target = angle;
    lastContact = millis();
    Serial.printf("MOVE ch=%s from=%.1f to=%.1f rate_deg_s=10\n", channel, joint.command, angle);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setTimeOut(50);
  Serial.println("RoboFlex five-joint bench v2; no WiFi, no automatic pose");
  if (!pwm.begin()) {
    fault = true;
    Serial.println("ERR PCA9685 missing at 0x40; check logic power, SDA21/SCL22 and common GND");
    return;
  }
  // Motor power must remain OFF during upload/setup: the library resets and
  // configures the PCA9685 before these writes clear any retained outputs.
  for (uint8_t channel = 0; channel < 16; ++channel) {
    if (pwm.setPWM(channel, 0, 4096) != 0) fault = true;
  }
  if (fault) { Serial.println("ERR could not disable all PWM outputs"); return; }
  pwm.setPWMFreq(50);
  ready = true;
  Serial.println("READY outputs=off arm_ch=1..4 gripper_ch=5 max_deg=120,120,80,120,120");
  Serial.println("First init is an absolute target, not a measured or speed-controlled move.");
  printStatus();
}

void loop() {
  // A bounded read prevents continuous serial input from starving the timeout.
  for (uint8_t n = 0; n < 80 && Serial.available() > 0; ++n) {
    const char ch = (char)Serial.read();
    if (ch == '\n') {
      if (!discardLine) { line[lineLength] = '\0'; processLine(line); }
      lineLength = 0;
      discardLine = false;
    } else if (!discardLine) {
      if (ch == '\0' || lineLength >= sizeof(line) - 1) {
        discardLine = true;
        Serial.println("ERR malformed/overlong line discarded");
      } else line[lineLength++] = ch;
    }
  }
  const uint32_t now = millis();
  if (isMoving() && now - lastContact > kLinkTimeoutMs) {
    holdCommands();
    Serial.println("HOLD link timeout; last PWM commands retained");
  }
  if (!ready || fault || now - lastStep < kStepMs) return;
  lastStep = now;
  for (size_t i = 0; i < kJointCount; ++i) {
    Joint &joint = joints[i];
    const float delta = joint.target - joint.command;
    if (!joint.initialized || joint.command == joint.target) continue;
    const float next = fabsf(delta) <= kStepDeg ? joint.target :
                       joint.command + (delta > 0 ? kStepDeg : -kStepDeg);
    if (!writeAngle(i, next)) break;
    joint.command = next;
    if (joint.command == joint.target)
      Serial.printf("DONE ch=%u command_deg=%.1f actual_position=unknown\n", kChannels[i], next);
  }
  delay(1);
}
