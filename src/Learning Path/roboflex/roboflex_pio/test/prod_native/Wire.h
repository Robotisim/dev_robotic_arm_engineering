#pragma once
struct FakeWire{void begin(int,int){} void setTimeOut(int){} void beginTransmission(int){} int endTransmission(){return 0;}}; inline FakeWire Wire;
