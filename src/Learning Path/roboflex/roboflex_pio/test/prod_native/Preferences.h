
#pragma once
#include <cstdint>
inline bool prefsOpen=false;
inline unsigned prefsCount=6;
struct Preferences {
 bool begin(const char*,bool){return prefsOpen;}
 void end(){}
 bool getBool(const char*,bool){return true;}
 uint32_t getUInt(const char* k,unsigned){return !strcmp(k,"count")?prefsCount:2;}
 float getFloat(const char*,float v){return v;}
 void putBool(const char*,bool){}
 void putUInt(const char*,uint32_t){}
 void putFloat(const char*,float){}
};
