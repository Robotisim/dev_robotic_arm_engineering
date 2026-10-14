
#pragma once
#include <cstdint>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>
#include <tuple>
inline uint32_t clockMs=0;
inline uint32_t millis(){return clockMs;}
inline void delay(unsigned int n){clockMs+=n;}
constexpr int HIGH=1,LOW=0,OUTPUT=1,HEX=16;
inline void pinMode(int,int){}
inline void digitalWrite(int,int){}
inline int digitalRead(int){return 0;}
struct FakeSerial {
 std::deque<char> input;
 void begin(int){}
 template<class... T> void println(T...){}
 template<class... T> void print(T...){}
 template<class... T> void printf(const char*,T...){}
 int available(){return input.size();}
 int read(){char c=input.front();input.pop_front();return c;}
};
inline FakeSerial Serial;
