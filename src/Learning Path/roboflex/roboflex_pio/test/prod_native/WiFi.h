#pragma once
constexpr int WIFI_STA=1, WL_CONNECTED=3;
struct FakeWifi{void mode(int){} void setSleep(bool){} void begin(const char*,const char*){} int status(){return WL_CONNECTED;} const char* localIP(){return "127.0.0.1";}};
inline FakeWifi WiFi;
