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
  enum class Status : uint8_t {
    Off,
    PoweringOn,
    Ready,
    Refreshing,
    PoweringOff,
    Fault,
    Sleeping
  };
  enum class Result : uint8_t {
    Pending,
    Complete,
    AssertionTimeout,
    CompletionTimeout,
    Invalid
  };
  struct Window {
    uint16_t x, y, width, height;
  };
  struct Plane {
    const uint8_t* data;
    size_t size;
  };
  using Luts = std::array<std::array<uint8_t, 49>, 5>;
  enum class Event : uint8_t { Reset, Write, Upload, Issue, BusyOn, BusyOff,
                               AssertionTimeout, CompletionTimeout };
  struct TraceEvent { Event event; uint8_t command; uint32_t at, detail, pollGap; };
  using Clock = uint32_t (*)();
  explicit Uc8279X4Transport(EpdBus& bus, Clock clock = nullptr)
      : bus_(bus), clock_(clock) {}
  void clearTrace() { traceHead_ = traceSize_ = 0; }
  size_t traceSize() const { return traceSize_; }
  const TraceEvent& traceEvent(size_t index) const {
    return trace_[(traceHead_ + trace_.size() - traceSize_ + index) % trace_.size()];
  }
  Status status() const { return status_; }
  uint32_t uploadedBytes() const { return uploaded_; }
  void reset() {
    record(Event::Reset, 0);
    bus_.reset(50);
    bytes(0x00, {0x37, 0x4D});
    geometry();
    bytes(0x30, {0x0E});
    bytes(0xE1, {0x02});
    bytes(0xE0, {0x02});
    bytes(0xE5, {0x1E});
    bytes(0x50, {0x97});
    status_ = Status::Off;
    uploaded_ = 0;
  }
  bool powerOn(uint32_t now) {
    if (status_ != Status::Off) return false;
    // Cold external-LUT mode must not bypass the panel's OTP startup path.
    // Select OTP while PON loads panel defaults; activate() switches to the
    // external bank only after power-on completion. No display refresh here.
    bytes(0x00, {0x17, 0x4D});
    configure(false);
    issue(0x04, Status::PoweringOn, now);
    return true;
  }
  bool upload(uint8_t command, Plane plane, Window uploadWindow,
              uint8_t hiddenGateFill = 0) {
    if (status_ != Status::Ready || (command != 0x10 && command != 0x13) ||
        plane.data == nullptr || plane.size != 48000 || !valid(uploadWindow) ||
        (uploadWindow.x % 8) || (uploadWindow.width % 8))
      return false;
    const uint32_t before = uploaded_;
    const bool full = uploadWindow.x == 0 && uploadWindow.y == 0 &&
                      uploadWindow.width == 800 && uploadWindow.height == 480;
    if (full)
      bus_.cmd(0x92);
    else
      window(uploadWindow);
    bus_.cmd(command);
    record(Event::Write, command);
    auto transaction = bus_.beginTxn();
    // One internal-RAM staging row keeps PSRAM out of SPI driver constraints.
    std::array<uint8_t, 100> row{};
    if (full) {
      row.fill(hiddenGateFill);  // B/W: white like driver; AA: selector 00
      for (uint16_t y = 0; y < 120; ++y)
        transaction.writeBytes(row.data(), 100);
      uploaded_ += 12000;
    }
    const uint16_t count = uploadWindow.width / 8;
    for (uint16_t y = uploadWindow.y; y < uploadWindow.y + uploadWindow.height;
         ++y) {
      for (uint16_t x = 0; x < count; ++x)
        row[x] = plane.data[y * 100 + uploadWindow.x / 8 + x];
      transaction.writeBytes(row.data(), count);
      uploaded_ += count;
    }
    transaction.end();
    bus_.cmd(0x11);
    bus_.cmd(0x92);
    record(Event::Upload, command, uploaded_ - before);
    return true;
  }
  bool activate(const Luts& luts, Window refreshWindow, bool verticalValidated,
                uint32_t now, bool fullBw = false) {
    if (status_ != Status::Ready || !valid(refreshWindow) ||
        refreshWindow.x != 0 || refreshWindow.width != 800)
      return false;
    // PON reloads settings: every activation reasserts them AFTER power-on.
    // Full B/W compares OTP and external banks with the same surrounding
    // setup. Candidate AA retains its established CDI. No invented analog
    // voltages: PWR/VDCS/BTST continue to come from this panel's MTP.
    configure(!fullBw);
    for (uint8_t i = 0; i < 5; ++i) {
      bus_.cmd(0x20 + i);
      bus_.data(luts[i].data(), 49);
      record(Event::Upload, 0x20 + i, 49);
    }
    bus_.cmd(0x92);
    if (verticalValidated)
      window(refreshWindow);  // ALWAYS complete visible lines
    bytes(0x00, {0x37, 0x4D});
    issue(0x12, Status::Refreshing, now);
    return true;
  }
  bool activateOtp(bool fast, uint32_t now) {
    if (status_ != Status::Ready) return false;
    configure(fast, fast);
    bus_.cmd(0x92);
    if (fast) window({0, 0, 800, 480});
    bytes(0x00, {0x17, 0x4D});
    issue(0x12, Status::Refreshing, now);
    return true;
  }
  bool powerOff(uint32_t now) {
    // A fault may leave HV on. Permit a shutdown attempt only when BUSY is
    // idle; never reset a possibly active waveform to reclaim ownership.
    if (status_ != Status::Ready &&
        !(status_ == Status::Fault && !bus_.isBusy())) return false;
    issue(0x02, Status::PoweringOff, now);
    return true;
  }
  bool sleep() {
    if (status_ != Status::Off) return false;
    bytes(0x07, {0xA5});
    status_ = Status::Sleeping;
    return true;
  }
  Result poll(uint32_t now) {
    now_ = clock_ ? clock_() : now;
    now = now_;
    pollGap_ = now - lastPoll_;
    lastPoll_ = now;
    if (status_ != Status::PoweringOn && status_ != Status::Refreshing &&
        status_ != Status::PoweringOff)
      return status_ == Status::Fault ? Result::Invalid : Result::Complete;
    const uint32_t elapsed = now - started_;
    const bool busy = bus_.isBusy();
    if (busy && !asserted_) {
      asserted_ = true;
      record(Event::BusyOn, activeCommand_, elapsed);
    }
    if (asserted_ && !busy) {
      record(Event::BusyOff, activeCommand_, elapsed);
      if (status_ == Status::PoweringOn) {
        // PON can reload module defaults. Restore addressing and polarity
        // BEFORE either DTM upload, not only immediately before DRF.
        bytes(0x00, {0x17, 0x4D});
        geometry();
        configure(false);
      }
      if (status_ == Status::PoweringOff)
        status_ = Status::Off;
      else
        status_ = Status::Ready;
      bus_.cmd(0x92);
      return Result::Complete;
    }
    // Missing BUSY assertion is not proof that a command completed.
    if (!asserted_ && elapsed >= 1000) {
      record(Event::AssertionTimeout, activeCommand_, elapsed);
      status_ = Status::Fault;
      return Result::AssertionTimeout;
    }
    if (elapsed >= 30000) {
      record(Event::CompletionTimeout, activeCommand_, elapsed);
      status_ = Status::Fault;
      return Result::CompletionTimeout;
    }
    return Result::Pending;
  }

 private:
  void geometry() {
    bytes(0x61, {0x03, 0x20, 0x02, 0x58});
    bytes(0x65, {0, 0, 0, 0});
    bytes(0x03, {0x20});
  }
  void configure(bool partial, bool fastTemperature = false) {
    bytes(0x50, {uint8_t(partial ? 0xD7 : 0x97)});
    bytes(0x30, {0x0E});
    bytes(0xE1, {0x02});
    bytes(0xE0, {0x02});
    // External AA keeps the temperature setting used after full OTP prep;
    // its established CDI does not imply the OTP fast-temperature shortcut.
    bytes(0xE5, {uint8_t(fastTemperature ? 0x5A : 0x1E)});
  }
  static bool valid(Window r) {
    return r.width && r.height && r.x < 800 && r.y < 480 &&
           r.width <= 800 - r.x && r.height <= 480 - r.y;
  }
  void bytes(uint8_t command, std::initializer_list<uint8_t> values) {
    bus_.cmd(command);
    uint32_t packed = 0;
    for (auto value : values) {
      bus_.data(value);
      packed = (packed << 8) | value;
    }
    record(Event::Write, command, packed);
  }
  void window(Window r) {
    const uint16_t x1 = r.x + r.width - 1, y0 = r.y + 120,
                   y1 = y0 + r.height - 1;
    bus_.cmd(0x91);
    bytes(0x90,
          {uint8_t(r.x >> 8), uint8_t(r.x), uint8_t(x1 >> 8), uint8_t(x1),
           uint8_t(y0 >> 8), uint8_t(y0), uint8_t(y1 >> 8), uint8_t(y1), 1});
  }
  void issue(uint8_t command, Status next, uint32_t now) {
    bus_.cmd(command);
    now_ = started_ = clock_ ? clock_() : now;
    lastPoll_ = now_;
    pollGap_ = 0;
    activeCommand_ = command;
    record(Event::Issue, command);
    asserted_ = bus_.isBusy();
    if (asserted_) record(Event::BusyOn, command);
    status_ = next;
  }
  void record(Event event, uint8_t command, uint32_t detail = 0) {
    trace_[traceHead_] = {event, command, clock_ ? clock_() : now_, detail, pollGap_};
    traceHead_ = (traceHead_ + 1) % trace_.size();
    if (traceSize_ < trace_.size()) ++traceSize_;
  }
  EpdBus& bus_;
  Clock clock_;
  std::array<TraceEvent, 96> trace_{};
  size_t traceHead_{0}, traceSize_{0};
  uint32_t now_{0};
  uint32_t lastPoll_{0}, pollGap_{0};
  uint8_t activeCommand_{0};
  Status status_{Status::Off};
  uint32_t started_{0}, uploaded_{0};
  bool asserted_{false};
};
}  // namespace freeink
