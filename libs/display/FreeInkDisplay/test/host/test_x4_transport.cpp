#include <Uc8279X4Transport.h>

#include <cassert>
#include <iostream>
#include <vector>
using T = freeink::Uc8279X4Transport;
uint32_t clockNow;
uint32_t clockRead() { return clockNow; }
int main() {
  freeink::EpdBus b;
  T t(b);
  t.reset();
  assert(t.powerOn(0));
  assert(t.poll(1) == T::Result::Pending);
  b.busy = true;
  assert(t.poll(2) == T::Result::Pending);
  b.busy = false;
  assert(t.poll(3) == T::Result::Complete);
  std::vector<uint8_t> data(48000);
  for (size_t i = 0; i < data.size(); ++i) data[i] = uint8_t(i / 100 + i % 100);
  assert(!t.upload(0x13, {data.data(), 4}, {0, 0, 800, 480}));
  assert(t.upload(0x10, {data.data(), data.size()}, {0, 0, 800, 480}));
  assert(b.bytes == 60000);
  assert(t.upload(0x13, {data.data(), data.size()}, {496, 90, 272, 40}));
  assert(b.bytes == 61360);
  const auto& packed = b.writes[b.writes.size() - 3].data;
  assert(packed.size() == 1360);
  for (size_t y = 0; y < 40; ++y)
    for (size_t x = 0; x < 34; ++x)
      assert(packed[y * 34 + x] == data[(90 + y) * 100 + 62 + x]);

  T::Luts l{};
  assert(!t.activate(l, {496, 90, 272, 40}, true, 4));
  assert(t.activate(l, {0, 90, 800, 40}, true, 4));
  size_t window = 0, psr = 0, drf = 0;
  for (size_t i = 0; i < b.writes.size(); ++i) {
    if (b.writes[i].cmd == 0x90) window = i;
    if (b.writes[i].cmd == 0x00) psr = i;
    if (b.writes[i].cmd == 0x12) drf = i;
  }
  assert(window < psr && psr < drf);
  assert((b.writes[window].data ==
          std::vector<uint8_t>{0, 0, 3, 31, 0, 210, 0, 249, 1}));
  assert(!t.upload(0x10, {data.data(), data.size()}, {0, 0, 800, 480}));
  b.busy = true;
  t.poll(5);
  b.busy = false;
  assert(t.poll(6) == T::Result::Complete);
  assert(t.powerOff(7));
  assert(t.poll(1007) == T::Result::AssertionTimeout);
  assert(!t.sleep());
  t.reset();
  assert(t.powerOn(0xFFFFFFF0));
  b.busy = true;
  t.poll(0xFFFFFFF1);
  b.busy = false;
  assert(t.poll(3) == T::Result::Complete);
  assert(t.powerOff(5));
  b.busy = true;
  t.poll(6);
  assert(t.poll(30005) == T::Result::CompletionTimeout);
  t.reset();
  b.busy = false;
  assert(t.sleep());
  assert(!t.powerOn(0));
  freeink::EpdBus matched;
  T m(matched, clockRead);
  clockNow = 100;
  m.reset();
  assert(m.powerOn(0));  // the supplied time can predate a long RAM upload
  matched.busy = true; clockNow = 102; m.poll(0);
  matched.busy = false; clockNow = 110; m.poll(0);
  assert(m.activate(l, {0,0,800,480}, false, 0, true));
  auto reg = [&](uint8_t cmd) {
    for (auto i = matched.writes.rbegin(); i != matched.writes.rend(); ++i)
      if (i->cmd == cmd && !i->data.empty()) return i->data[0];
    return uint8_t(0);
  };
  assert(reg(0x50) == 0x97 && reg(0xE0) == 2 && reg(0xE5) == 0x1E);
  assert(reg(0x00) == 0x37);
  matched.busy = true; clockNow = 112; m.poll(0);
  matched.busy = false; clockNow = 120; m.poll(0);
  assert(m.activateOtp(false, 0));
  assert(reg(0x50) == 0x97 && reg(0xE0) == 2 && reg(0xE5) == 0x1E);
  assert(reg(0x00) == 0x17);
  bool asserted = false, released = false;
  for (size_t i = 0; i < m.traceSize(); ++i) {
    const auto& e = m.traceEvent(i);
    if (e.command == 0x12 && e.event == T::Event::BusyOn)
      asserted |= e.at == 112 && e.detail == 2;
    if (e.command == 0x12 && e.event == T::Event::BusyOff)
      released |= e.at == 120 && e.detail == 10;
  }
  assert(asserted && released);
  clockNow = 1120;
  assert(m.poll(0) == T::Result::AssertionTimeout);
  matched.busy = true;
  assert(!m.powerOff(0));
  matched.busy = false;
  assert(m.powerOff(0));  // safe shutdown attempt, never an implicit reset
  matched.busy = true; clockNow = 1122; m.poll(0);
  matched.busy = false; clockNow = 1130;
  assert(m.poll(0) == T::Result::Complete && m.status() == T::Status::Off);
  assert(m.powerOn(0));
  assert(reg(0x00) == 0x17 && matched.writes.back().cmd == 0x04);
  freeink::EpdBus padded;
  T h(padded); h.reset(); h.powerOn(0);
  padded.busy=true; h.poll(1); padded.busy=false; h.poll(2);
  assert(h.upload(0x13,{data.data(),data.size()},{0,0,800,480},255));
  const auto& full=padded.writes[padded.writes.size()-3].data;
  assert(full.size()==60000);
  for(size_t i=0;i<12000;++i) assert(full[i]==255);
  assert(std::equal(data.begin(),data.end(),full.begin()+12000));
  m.clearTrace(); assert(m.traceSize() == 0);
  // The opt-in power setup is sourced only from the verified known module.
  // Reject changed headers, unverified data and other modules. No voltage
  // payload is accepted from an unrecognized memory map.
  std::array<uint8_t, 4096> mtp{};
  mtp[0]=mtp[1]=0xA5; mtp[0x1A]=0x68;
  mtp[11]=3; mtp[12]=0x20; mtp[13]=2; mtp[14]=0x58;
  mtp[4]=0x25; mtp[5]=0x25; mtp[6]=0x3C;
  std::array<uint8_t,8> boundaries{5,10,15,20,80,95,100,127};
  std::copy(boundaries.begin(),boundaries.end(),mtp.begin()+0x28);
  std::array<uint8_t,9> header{9,0,0x78,0x78,0x14,0x24,0,0,0};
  const size_t table=0x33+4*0x121;
  std::copy(header.begin(),header.end(),mtp.begin()+table);
  const size_t coolerTable = 0x33 + 3 * 0x121;
  std::copy(header.begin(), header.end(), mtp.begin() + coolerTable);
  mtp[coolerTable + 9 + 1] = 32;
  auto cooler = freeink::observedX4FactoryProfile(mtp.data(),mtp.size(),true,0x68,20);
  assert(cooler.power.valid && cooler.range == 3 && cooler.upperC == 20 &&
         cooler.rows[0][1] == 32);
  assert(freeink::observedX4FactoryProfile(mtp.data(),mtp.size(),true,0x68,21).range == 4);
  assert(!freeink::observedX4FactoryProfile(mtp.data(),mtp.size(),true,0x68,90).power.valid);
  auto profile=freeink::observedX4PanelPower(mtp.data(),mtp.size(),true,0x68);
  assert(profile.valid && profile.pwr[0]==0x43 && profile.pwr[4]==0x14 &&
         profile.vcom==0x24 && profile.pll==9);
  assert(!freeink::observedX4PanelPower(mtp.data(),mtp.size(),false,0x68).valid);
  assert(!freeink::observedX4PanelPower(mtp.data(),300,true,0x68).valid);
  assert(!freeink::observedX4PanelPower(mtp.data(),mtp.size(),true,0x66).valid);
  ++mtp[table+2];
  assert(!freeink::observedX4PanelPower(mtp.data(),mtp.size(),true,0x68).valid);
  freeink::EpdBus powered; T p(powered); p.reset();
  p.setPanelPower(profile,1); powered.writes.clear(); assert(p.powerOn(0));
  assert(powered.writes[0].cmd==0 && powered.writes[0].data[0]==0x37);
  assert(powered.writes[1].cmd==1 && powered.writes[1].data==
         (std::vector<uint8_t>{0x43,0,0x78,0x78,0x14}));
  assert(powered.writes[2].cmd==0x82 && powered.writes[2].data[0]==0x24);
  assert(powered.writes[3].cmd==6 && powered.writes[3].data==
         (std::vector<uint8_t>{0x25,0x25,0x3C}));
  assert(powered.writes.back().cmd==4 && p.activePowerMode()==1);
  powered.busy=true; p.poll(1); powered.busy=false; p.poll(2);
  auto powerPll=[&]() {
    for(auto i=powered.writes.rbegin();i!=powered.writes.rend();++i) {
      if(i->cmd==0x30) return i->data[0];
    }
    return uint8_t(0);
  };
  p.setPanelPower(profile,2);  // next PON only, not a live clock/voltage edit
  assert(p.activate(l,{0,0,800,480},false,3,true));
  assert(powerPll()==0x0E && p.activePowerMode()==1);
  powered.busy=true; p.poll(4); powered.busy=false; p.poll(5);
  assert(p.powerOff(6)); powered.busy=true; p.poll(7); powered.busy=false;p.poll(8);
  assert(p.powerOn(9) && p.activePowerMode()==2 && powerPll()==9);
  std::cout << "transport: command order, full-line restore, gate offset, "
               "packing, timeouts, wrap, sleep passed\n";
}
