#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include "../src/bus/EpdBus.h"

namespace freeink {
// Raw electrical transport for the 800x480/600-gate X4 Pro only. Owns no image
// storage or rendering policy. Caller excludes ordinary FreeInk display calls.
class Uc8279X4Transport {
 public:
  enum class Status : uint8_t { Off, PoweringOn, Ready, Refreshing, PoweringOff, Fault, Sleeping };
  enum class Result : uint8_t { Pending, Complete, AssertionTimeout, CompletionTimeout, Invalid };
  struct Window { uint16_t x, y, width, height; };
  struct Plane { const uint8_t* data; size_t size; };
  using Luts = std::array<std::array<uint8_t, 49>, 5>;
  explicit Uc8279X4Transport(EpdBus& bus) : bus_(bus) {}
  Status status() const { return status_; }
  uint32_t uploadedBytes() const { return uploaded_; }
  void reset() {
    bus_.reset(50);
    bytes(0x00, {0x37, 0x4D});
    bytes(0x61, {0x03, 0x20, 0x02, 0x58});
    bytes(0x65, {0, 0, 0, 0});
    bytes(0x03, {0x20}); bytes(0x30, {0x0E}); bytes(0xE1, {0x02});
    status_ = Status::Off; uploaded_ = 0;
  }
  bool powerOn(uint32_t now) {
    if (status_ != Status::Off) return false;
    issue(0x04, Status::PoweringOn, now); return true;
  }
  bool upload(uint8_t command, Plane plane, Window uploadWindow) {
    if (status_ != Status::Ready || (command != 0x10 && command != 0x13) ||
        plane.data == nullptr || plane.size != 48000 || !valid(uploadWindow) ||
        (uploadWindow.x % 8) || (uploadWindow.width % 8)) return false;
    const bool full = uploadWindow.x == 0 && uploadWindow.y == 0 &&
                      uploadWindow.width == 800 && uploadWindow.height == 480;
    if (full) bus_.cmd(0x92); else window(uploadWindow);
    bus_.cmd(command); bus_.beginTxn();
    // One internal-RAM staging row keeps PSRAM out of SPI driver constraints.
    std::array<uint8_t, 100> row{};
    if (full) {
      row.fill(0);  // selector 00 in unbonded gates; never visible
      for (uint16_t y = 0; y < 120; ++y) bus_.rawWriteBytes(row.data(), 100);
      uploaded_ += 12000;
    }
    const uint16_t count = uploadWindow.width / 8;
    for (uint16_t y = uploadWindow.y; y < uploadWindow.y + uploadWindow.height; ++y) {
      for (uint16_t x = 0; x < count; ++x) row[x] = plane.data[y * 100 + uploadWindow.x / 8 + x];
      bus_.rawWriteBytes(row.data(), count); uploaded_ += count;
    }
    bus_.endTxn(); bus_.cmd(0x11); bus_.cmd(0x92);
    return true;
  }
  bool activate(const Luts& luts, Window refreshWindow, bool verticalValidated, uint32_t now) {
    if (status_ != Status::Ready || !valid(refreshWindow) || refreshWindow.x != 0 ||
        refreshWindow.width != 800) return false;
    // PON reloads settings: every activation reasserts them AFTER power-on.
    bytes(0x50, {0xD7}); bytes(0x30, {0x0E}); bytes(0xE1, {0x02});
    for (uint8_t i = 0; i < 5; ++i) {
      bus_.cmd(0x20 + i); bus_.data(luts[i].data(), 49);
    }
    bus_.cmd(0x92);
    if (verticalValidated) window(refreshWindow);  // ALWAYS complete visible lines
    bytes(0x00, {0x37, 0x4D});
    issue(0x12, Status::Refreshing, now); return true;
  }
  bool activateOtp(bool fast, uint32_t now) {
    if (status_ != Status::Ready) return false;
    bytes(0x50, {uint8_t(fast ? 0xD7 : 0x97)});
    bytes(0xE0, {0x02}); bytes(0xE5, {uint8_t(fast ? 0x5A : 0x1E)});
    bus_.cmd(0x92);
    if (fast) window({0,0,800,480});
    bytes(0x00, {0x17,0x4D}); issue(0x12,Status::Refreshing,now); return true;
  }
  bool powerOff(uint32_t now) {
    if (status_ != Status::Ready) return false;
    issue(0x02, Status::PoweringOff, now); return true;
  }
  bool sleep() {
    if (status_ != Status::Off) return false;
    bytes(0x07, {0xA5}); status_ = Status::Sleeping; return true;
  }
  Result poll(uint32_t now) {
    if (status_ != Status::PoweringOn && status_ != Status::Refreshing && status_ != Status::PoweringOff)
      return status_ == Status::Fault ? Result::Invalid : Result::Complete;
    const uint32_t elapsed = now - started_;
    if (bus_.isBusy()) asserted_ = true;
    if (asserted_ && !bus_.isBusy()) {
      if (status_ == Status::PoweringOff) status_ = Status::Off;
      else status_ = Status::Ready;
      bus_.cmd(0x92);
      return Result::Complete;
    }
    // Missing BUSY assertion is not proof that a command completed.
    if (!asserted_ && elapsed >= 1000) { status_ = Status::Fault; return Result::AssertionTimeout; }
    if (elapsed >= 30000) { status_ = Status::Fault; return Result::CompletionTimeout; }
    return Result::Pending;
  }
 private:
  static bool valid(Window r) {
    return r.width && r.height && r.x < 800 && r.y < 480 &&
           r.width <= 800 - r.x && r.height <= 480 - r.y;
  }
  void bytes(uint8_t command, std::initializer_list<uint8_t> values) {
    bus_.cmd(command); for (auto value : values) bus_.data(value);
  }
  void window(Window r) {
    const uint16_t x1 = r.x + r.width - 1, y0 = r.y + 120, y1 = y0 + r.height - 1;
    bus_.cmd(0x91);
    bytes(0x90, {uint8_t(r.x >> 8), uint8_t(r.x), uint8_t(x1 >> 8), uint8_t(x1),
                 uint8_t(y0 >> 8), uint8_t(y0), uint8_t(y1 >> 8), uint8_t(y1), 1});
  }
  void issue(uint8_t command, Status next, uint32_t now) {
    bus_.cmd(command); started_ = now; asserted_ = bus_.isBusy(); status_ = next;
  }
  EpdBus& bus_;
  Status status_{Status::Off};
  uint32_t started_{0}, uploaded_{0};
  bool asserted_{false};
};
}  // namespace freeink
