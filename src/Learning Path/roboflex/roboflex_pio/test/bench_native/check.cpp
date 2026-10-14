
#include <cassert>
#include <iostream>
#include "../../src/main_bench.cpp"
void cmd(const char*s){char b[120];snprintf(b,sizeof b,"%s",s);processLine(b);}
void tick(){clockMs+=20;loop();}
int main(){
 setup();assert(ready&&!fault&&writes.size()==16);
 for(auto [c,on,off]:writes){assert(off==4096&&on==0);}
 writes.clear();
 for(auto s:{"init 0 45","init 6 45","init 16 45","init -1 45","init 01 45","init 1 nan","init 1 inf","init 1 121","init 1 -1","init 1 45x","init 1 45 extra","move 1 45","init 1","init 1e0 45","init 2 1e999","init 3 81","init 4 121","init 5 121"})cmd(s);
 assert(writes.empty());
 for(const auto &joint:joints)assert(!joint.initialized);
 cmd("init 1 45");assert(writes.size()==1&&std::get<0>(writes.back())==1&&std::get<2>(writes.back())==263);
 cmd("init 1 0");cmd("move 1 100");assert(writes.size()==1&&joints[0].target==45);
 cmd("move 1 85");assert(joints[0].target==85);
 cmd("init 2 45");assert(!joints[1].initialized);
 float previous=joints[0].command;
 for(int n=0;n<210;++n){if(n%25==0)cmd("ping");tick();assert(joints[0].command>=previous&&joints[0].command-previous<=0.201f);previous=joints[0].command;}
 assert(joints[0].command==85);
 for(auto [c,on,off]:writes)assert(c==1&&off>=150&&off<=450);
 cmd("move 1 45");tick();cmd("stop");assert(joints[0].target==joints[0].command);
 cmd("move 1 45");clockMs+=2100;loop();assert(joints[0].target==joints[0].command);
 std::string invalid(90,'x');invalid+="init 2 45\n";for(char c:invalid)Serial.input.push_back(c);loop();loop();assert(!joints[1].initialized);
 std::string nul="garbage";nul.push_back('\0');nul+="init 2 45\n";for(char c:nul)Serial.input.push_back(c);loop();assert(!joints[1].initialized);
 cmd("init 2 45");assert(joints[1].initialized&&std::get<0>(writes.back())==2);
 // Every configured channel can be commanded to zero, individually. No
 // position write may reach channel 0 or the unused channels 6..15.
 cmd("move 1 45");
 for(int n=0;n<260;++n){if(n%25==0)cmd("ping");tick();}
 for(int channel=1;channel<=5;++channel){
  char command[40];snprintf(command,sizeof command,"%s %d 0",channel<=2?"move":"init",channel);cmd(command);
  for(int n=0;n<260;++n){if(n%25==0)cmd("ping");tick();}
  assert(joints[channel-1].initialized&&joints[channel-1].command==0&&joints[channel-1].target==0);
  assert(std::get<0>(writes.back())==channel&&std::get<2>(writes.back())==150);
 }
 for(auto [c,on,off]:writes)assert(c>=1&&c<=5&&on==0&&off>=150&&off<=450);
 cmd("move 3 81");assert(joints[2].target==0);
 cmd("move 3 50");for(int n=0;n<260;++n){if(n%25==0)cmd("ping");tick();}
 cmd("move 3 80");for(int n=0;n<160;++n){if(n%25==0)cmd("ping");tick();}
 assert(joints[2].command==80&&std::get<2>(writes.back())==350);
 cmd("move 3 81");assert(joints[2].target==80);
 cmd("move 2 45");for(int n=0;n<260;++n){if(n%25==0)cmd("ping");tick();}
 cmd("move 2 85");failWrite=true;tick();assert(fault&&joints[1].command==45);
 auto count=writes.size();cmd("move 1 60");assert(writes.size()==count);
 ready=false;fault=false;beginOk=false;writes.clear();setup();assert(!ready&&fault&&writes.empty());
 std::cout<<"PASS: startup off, channels 1..5 zero targets, channel 3 limit, input validation, slew bound, single move, timeout hold, overflow discard, I2C fault lock, missing driver\n";
}
