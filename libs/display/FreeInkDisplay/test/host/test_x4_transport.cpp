#include <Uc8279X4Transport.h>
#include <cassert>
#include <vector>
#include <iostream>
using T=freeink::Uc8279X4Transport;
int main(){
 freeink::EpdBus b;T t(b);t.reset();assert(t.powerOn(0));
 assert(t.poll(1)==T::Result::Pending);b.busy=true;assert(t.poll(2)==T::Result::Pending);
 b.busy=false;assert(t.poll(3)==T::Result::Complete);
 std::vector<uint8_t> data(48000,0xA5);assert(!t.upload(0x13,{data.data(),4},{0,0,800,480}));
 assert(t.upload(0x10,{data.data(),data.size()},{0,0,800,480}));assert(b.bytes==60000);
 assert(t.upload(0x13,{data.data(),data.size()},{496,90,272,40}));assert(b.bytes==61360);
 T::Luts l{};assert(!t.activate(l,{496,90,272,40},true,4));
 assert(t.activate(l,{0,90,800,40},true,4));
 size_t window=0,psr=0,drf=0;
 for(size_t i=0;i<b.writes.size();++i){if(b.writes[i].cmd==0x90)window=i;if(b.writes[i].cmd==0x00)psr=i;if(b.writes[i].cmd==0x12)drf=i;}
 assert(window<psr && psr<drf);
 assert((b.writes[window].data==std::vector<uint8_t>{0,0,3,31,0,210,0,249,1}));
 assert(!t.upload(0x10,{data.data(),data.size()},{0,0,800,480}));
 b.busy=true;t.poll(5);b.busy=false;assert(t.poll(6)==T::Result::Complete);
 assert(t.powerOff(7));assert(t.poll(1007)==T::Result::AssertionTimeout);assert(!t.sleep());
 t.reset();assert(t.powerOn(0xFFFFFFF0));b.busy=true;t.poll(0xFFFFFFF1);b.busy=false;
 assert(t.poll(3)==T::Result::Complete);assert(t.powerOff(5));b.busy=true;t.poll(6);
 assert(t.poll(30005)==T::Result::CompletionTimeout);
 t.reset();b.busy=false;assert(t.sleep());assert(!t.powerOn(0));
 std::cout<<"transport: command order, full-line restore, gate offset, packing, timeouts, wrap, sleep passed\n";
}
