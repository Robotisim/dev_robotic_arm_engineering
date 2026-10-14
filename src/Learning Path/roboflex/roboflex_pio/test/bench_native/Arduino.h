
#pragma once
#include <cstdint>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>
#include <tuple>
inline uint32_t clockMs=0;
inline uint32_t millis(){return clockMs;}
inline void delay(unsigned int){}
struct FakeSerial {
 std::deque<char> input;
 void begin(int){}
 void println(const char*){}
 template<class... T> void printf(const char*,T...){}
 int available(){return input.size();}
 int read(){char c=input.front();input.pop_front();return c;}
};
inline FakeSerial Serial;
