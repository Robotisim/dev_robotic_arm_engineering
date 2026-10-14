#pragma once

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

namespace roboflex {
// Match the real-hardware URDF position limits, in ROS radians.
static constexpr size_t kCommandJointCount = 5;
static constexpr float kCommandMin[] = {0.0f, -1.57f, -1.57f, -1.57f, 0.0f};
static constexpr float kCommandMax[] = {1.57f, 0.523333f, -0.174444f, 0.523333f, 0.523333f};

inline bool parseCommandPacket(char *packet, size_t length, size_t capacity,
                               float *commands, size_t &count) {
  count = 0;
  if (!length || length >= capacity || memchr(packet, '\0', length)) return false;
  packet[length] = '\0';
  for (size_t i = 0; i < length; ++i)
    if (packet[i] == ',' || packet[i] == ';') packet[i] = ' ';
  char *save = nullptr;
  char *token = strtok_r(packet, " \t\r\n", &save);
  if (!token) return false;
  if (!strcmp(token, "CMD") || !strcmp(token, "cmd") ||
      !strcmp(token, "MOTOR") || !strcmp(token, "motor"))
    token = strtok_r(nullptr, " \t\r\n", &save);
  float parsed[kCommandJointCount];
  while (token && count < kCommandJointCount) {
    char *end = nullptr;
    const float value = strtof(token, &end);
    if (end == token || *end || !isfinite(value) ||
        value < kCommandMin[count] - 0.00001f ||
        value > kCommandMax[count] + 0.00001f) return false;
    parsed[count] = fminf(kCommandMax[count], fmaxf(kCommandMin[count], value));
    ++count;
    token = strtok_r(nullptr, " \t\r\n", &save);
  }
  if (count != kCommandJointCount || token) return false;
  memcpy(commands, parsed, sizeof(parsed));
  return true;
}
}  // namespace roboflex
