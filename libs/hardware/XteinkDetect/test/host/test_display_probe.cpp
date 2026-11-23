// Exercises the production detector with GPIO/time simulation and real board profiles.
#include <Arduino.h>
#include <BoardConfig.h>
#include <XteinkDetect.h>
#include <array>
#include <cassert>
#include <cstdio>
#include <map>
#include <vector>

namespace {
unsigned long now;
unsigned long readyAt;
std::array<int, 64> levels{}, modes{};
std::map<int, std::vector<uint8_t>> replies;
std::vector<int> commands;
std::vector<unsigned long> resets;
std::vector<int> resetLevels;
int command, commandBits, readBits, samples, holds;
bool sampleHigh;
bool corruptMtpRepeat;
int mtpReads;
auto pins = BoardConfig::XTEINK_X3.display;

void reset(BoardConfig::Board board, std::vector<uint8_t> ver, bool high = true) {
  BoardConfig::selectDevice(board);
  pins = BoardConfig::ACTIVE.display;
  now = readyAt = 0;
  command = commandBits = readBits = samples = holds = 0;
  sampleHigh = high;
  corruptMtpRepeat = false;
  mtpReads = 0;
  levels.fill(HIGH);
  modes.fill(INPUT);
  commands.clear(); resets.clear(); resetLevels.clear();
  replies = {{0x70, ver}, {0x71, {0x13}}, {0xA2, std::vector<uint8_t>(49, 0)}};
}

[[maybe_unused]] void checkX3Wire() {
  assert(commands == std::vector<int>{0x70});
  assert(samples == 24);
  assert(resets == (std::vector<unsigned long>{0, 10, 60}));
  assert(resetLevels == (std::vector<int>{HIGH, LOW, HIGH}));
  assert(holds == 1);
  assert(modes[pins.sclk] == INPUT && modes[pins.mosi] == INPUT);
  assert(modes[pins.cs] == INPUT_PULLUP && levels[pins.cs] == HIGH);
  assert(modes[pins.dc] == INPUT && modes[pins.rst] == INPUT);
  const auto& diag = freeink::getXteinkDisplayProbeDiag();
  assert(diag.valid && diag.verBytesRead == 3);
  assert(diag.ver[3] == 0 && diag.ver[4] == 0 && diag.flg == 0);
  assert(!diag.mtpValid);
  for (auto byte : diag.mtp) assert(byte == 0);
}
}

void pinMode(int pin, int mode) { modes.at(pin) = mode; }
void gpio_hold_dis(gpio_num_t pin) { if (pin == pins.rst) ++holds; }
void digitalWrite(int pin, int value) {
  if (pin == pins.rst) { resets.push_back(now); resetLevels.push_back(value); }
  if (pin == pins.cs && value == LOW) command = commandBits = readBits = 0;
  if (pin == pins.sclk && value == HIGH && levels[pin] == LOW && levels[pins.cs] == LOW &&
      levels[pins.dc] == LOW && modes[pins.mosi] == OUTPUT) {
    command = (command << 1) | levels[pins.mosi];
    if (++commandBits == 8) {
      commands.push_back(command);
      if (command == 0xA2) ++mtpReads;
    }
  }
  levels.at(pin) = value;
}
int digitalRead(int pin) {
  if (pin == pins.busy) return now >= readyAt ? HIGH : LOW;
  assert(pin == pins.mosi && levels[pins.cs] == LOW && levels[pins.dc] == HIGH);
  assert(levels[pins.sclk] == (sampleHigh ? HIGH : LOW));
  assert(modes[pin] == (sampleHigh ? INPUT : INPUT_PULLUP));
  assert(now >= 110 || !sampleHigh);
  const auto& data = replies.at(command);
  assert(static_cast<size_t>(readBits / 8) < data.size());
  int value = (data[readBits / 8] >> (7 - readBits % 8)) & 1;
  if (corruptMtpRepeat && command == 0xA2 && mtpReads == 3 && readBits == 4096 * 8)
    value ^= 1;
  ++readBits; ++samples;
  return value;
}
void delay(unsigned long ms) { now += ms; }
void delayMicroseconds(unsigned int) {}
unsigned long millis() { return now; }

int main() {
  using namespace freeink;
  using B = BoardConfig::Board;
  using V = DisplayControllerVerdict;
#if FREEINK_DEVICE_X4PRO
  // Full boot capture must include bytes beyond a uint8_t read length, match
  // the original header, and be repeatable. Export access must be read-only.
  reset(B::XteinkX4Pro, {0,0x0f,0x68,0,0}, false);
  auto& mtp = replies[0xA2];
  mtp.resize(4097);
  for (size_t i = 1; i < mtp.size(); ++i) mtp[i] = uint8_t(i * 37);
  mtp[1] = 0xA5;
  assert(detectXteinkDisplayController() == V::Uc81xxConfirmed);
  const auto captured = getXteinkPanelMtpSnapshot();
  assert(captured.verified && captured.size == 4096);
  for (size_t i = 0; i < captured.size; ++i) assert(captured.data[i] == mtp[i + 1]);
  const size_t commandCount = commands.size();
  assert(getXteinkPanelMtpSnapshot().data == captured.data);
  assert(commands.size() == commandCount && mtpReads == 3);
  assert(modes[pins.mosi] == INPUT && modes[pins.cs] == INPUT_PULLUP);
  corruptMtpRepeat = true;
  mtpReads = 0;
  assert(detectXteinkDisplayController() == V::Uc81xxConfirmed);
  assert(!getXteinkPanelMtpSnapshot().verified);
  std::puts("Verified full factory-memory capture checks passed");
#else
  uint8_t ver[5] = {9,9,9,9,9}, flg = 9;

  reset(B::XteinkX3, {0,0,0x66});
  assert(detectXteinkDisplayController(ver, &flg) == V::Uc81xxConfirmed);
  assert(ver[2] == 0x66 && ver[3] == 0 && ver[4] == 0 && flg == 0);
  assert(now == 111 && !getXteinkDisplayProbeDiag().busyTimedOut);
  checkX3Wire();

  // Prefix bytes and absent FLG/MTP must not gate a stock-recognized ID.
  reset(B::XteinkX3Uc8279, {0xff,0xff,0x66});
  replies.erase(0x71); replies.erase(0xA2);
  readyAt = 230;
  assert(detectXteinkDisplayController() == V::Uc81xxConfirmed);
  assert(now == 230 && !getXteinkDisplayProbeDiag().busyTimedOut);
  checkX3Wire();

  reset(B::XteinkX3, {0,0,0x66});
  readyAt = 10000;
  assert(detectXteinkDisplayController() == V::Uc81xxConfirmed);
  assert(now == 410 && getXteinkDisplayProbeDiag().busyTimedOut);
  checkX3Wire();

  for (uint8_t id : {uint8_t(0xff), uint8_t(0), uint8_t(0x68)}) {
    reset(B::XteinkX3, {0xff,0xff,id});
    assert(detectXteinkDisplayController() == (id == 0xff ? V::PrimaryAssumed : V::Inconclusive));
    checkX3Wire();
  }

  // Explicit X3 API must work before ACTIVE has been switched from X4.
  reset(B::XteinkX4, {0,0,0x66});
  pins = BoardConfig::XTEINK_X3.display;
  assert(detectX3DisplayController() == X3DisplayVerdict::Uc8279Confirmed);
  assert(BoardConfig::ACTIVE.board == B::XteinkX4);
  checkX3Wire();

  reset(B::XteinkX3, {0,0,0x66});
  assert(applyXteinkDisplayController());
  assert(BoardConfig::ACTIVE.displayController == BoardConfig::DisplayController::UC8279);
  assert(getXteinkDisplayProbeDiag().promoted);
  checkX3Wire();

  // X4 retains its five-byte/two-pass low-clock protocol and diagnostics.
  reset(B::XteinkX4, {0,0,1,0xff,0xff}, false);
  assert(detectXteinkDisplayController() == V::Uc81xxConfirmed);
  assert(commands == (std::vector<int>{0x71,0x70,0x71,0x70,0xA2}));
  assert(getXteinkDisplayProbeDiag().verBytesRead == 5);
  assert(getXteinkDisplayProbeDiag().mtpValid);

  // Stock panel table IDs, including the BOE 4.28 glass (0x40 UC8179,
  // 0x41/0x42 UC8279 D/E).
  for (uint8_t id : {uint8_t(0x01), uint8_t(0x40)}) {
    reset(B::XteinkX4, {0,0,id,0xff,0xff}, false);
    assert(applyXteinkDisplayController());
    assert(BoardConfig::ACTIVE.displayController == BoardConfig::DisplayController::UC8179);
  }
  for (uint8_t id : {uint8_t(0x02), uint8_t(0x41), uint8_t(0x42), uint8_t(0x68), uint8_t(0x69)}) {
    reset(B::XteinkX4, {0,0,id,0xff,0xff}, false);
    assert(applyXteinkDisplayController());
    assert(BoardConfig::ACTIVE.displayController == BoardConfig::DisplayController::UC8279);
    assert(BoardConfig::ACTIVE.displayControllerVariant == id);
  }
  reset(B::XteinkX3, {0,0,0x66});
  assert(detectXteinkDisplayController() == V::Uc81xxConfirmed);
  checkX3Wire();  // previous X4 MTP/status must not leak into the X3 snapshot
  std::puts("X3 stock protocol and X4 regression checks passed");
#endif
}
