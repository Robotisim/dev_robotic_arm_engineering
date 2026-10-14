#pragma once
struct FakeWire{void begin(int,int){} void setTimeOut(int){}}; inline FakeWire Wire;
