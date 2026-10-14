#include <cassert>
#include <cstring>
#include <iostream>
#include "../src/command_packet.h"

bool parse(const char *text, float *out) {
  char packet[193];
  const size_t length = strlen(text);
  assert(length < sizeof(packet));
  memcpy(packet, text, length);
  size_t count = 0;
  const bool accepted = roboflex::parseCommandPacket(packet, length, sizeof(packet), out, count);
  if (accepted) assert(count == 5);
  return accepted;
}

int main() {
  float q[5] = {99, 99, 99, 99, 99};
  for (const char *bad : {"", "CMD", "CMD 0 0 -0.5 0", "CMD 0 0 -0.5 0 0 9",
       "CMD banana 0 -0.5 0 0", "CMD nan 0 -0.5 0 0", "CMD inf 0 -0.5 0 0",
       "CMD 0 0 -0.5x 0 0", "CMD 0 0 0 0 0", "CMD -0.1 0 -0.5 0 0",
       "CMD 1.6 0 -0.5 0 0", "CMD 0 -2 -0.5 0 0", "CMD 0 0 -0.5 0 0.6",
       "CMD 0 0 -0.5 0 1e99", "CMD 0 0 -0.5 0 0 junk"}) {
    assert(!parse(bad, q));
    for (float value : q) assert(value == 99);  // Reject atomically.
  }
  assert(parse("CMD 0.0349066 -0.3839724 -0.9250245 -0.3665191 0", q));
  assert(parse("CMD 0,-1.57;-1.57 0 0", q));
  assert(parse("1.57 0.523333 -0.174444 0.523333 0.523333", q));
  char packet[] = "CMD 0 0 -0.5 0 0\0 junk";
  size_t count = 0;
  assert(!roboflex::parseCommandPacket(packet, sizeof(packet)-1, sizeof(packet), q, count));
  assert(!roboflex::parseCommandPacket(packet, sizeof(packet), sizeof(packet), q, count));
  std::cout << "PASS: exact joint count, finite values, full tokens, ROS bounds, atomic rejection, packet length\n";
}
