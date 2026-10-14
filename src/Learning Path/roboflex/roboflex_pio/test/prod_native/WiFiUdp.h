
#pragma once
#include <deque>
#include <string>
#include <cstring>
struct WiFiUDP {
 std::deque<std::string> input;
 bool begin(int){return true;}
 int parsePacket(){return input.empty()?0:input.front().size();}
 int read(char* p,int capacity){auto s=input.front();input.pop_front();int n=std::min(capacity,(int)s.size());memcpy(p,s.data(),n);return n;}
};
