#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace freeink {
struct Uc8279X4PanelPower {
  bool valid = false;
  std::array<uint8_t, 5> pwr{};
  std::array<uint8_t, 3> booster{};
  uint8_t vcom = 0, pll = 0;
};

// EXPERIMENTAL mapping of this observed 0x68 module's 30 C table. Its two
// complete boot reads agree (SHA256 78591ef00b383e73363002e6888f09e3809103a913d87
// ca8c2d1ab2bd245b1ef). The nine-byte prefix matches the vendor's X4 QY external
// blob shape; register payload layout and enable byte 0x43 follow the recovered
// UC8279 initializer in Uc8279X3Luts.h. This is NOT a general MTP decoder or a
// voltage measurement. Reject unfamiliar modules/data instead of guessing.
inline Uc8279X4PanelPower observedX4PanelPower(const uint8_t* mtp, size_t size,
                                            bool verified, uint8_t controllerId) {
  if (!verified || !mtp || size != 4096 || controllerId != 0x68 ||
      mtp[0] != 0xA5 || mtp[1] != 0xA5 || mtp[0x1A] != 0x68 ||
      mtp[11] != 3 || mtp[12] != 0x20 || mtp[13] != 2 || mtp[14] != 0x58)
    return {};
  constexpr std::array<uint8_t, 8> boundaries{5,10,15,20,80,95,100,127};
  for (size_t i = 0; i < boundaries.size(); ++i)
    if (mtp[0x28 + i] != boundaries[i]) return {};
  // Forced 30 C selects range 4 (20 < 30 <= 80). Eight populated blocks are
  // spaced 0x121 bytes apart, beginning at 0x33. First 9 bytes are settings.
  constexpr size_t table = 0x33 + 4 * 0x121;
  constexpr std::array<uint8_t, 9> header{9,0,0x78,0x78,0x14,0x24,0,0,0};
  for (size_t i = 0; i < header.size(); ++i)
    if (mtp[table + i] != header[i]) return {};
  if (mtp[4] != 0x25 || mtp[5] != 0x25 || mtp[6] != 0x3C) return {};
  return {true, {0x43, mtp[table+1], mtp[table+2], mtp[table+3], mtp[table+4]},
          {mtp[4], mtp[5], mtp[6]}, mtp[table+5], mtp[table]};
}
}  // namespace freeink
