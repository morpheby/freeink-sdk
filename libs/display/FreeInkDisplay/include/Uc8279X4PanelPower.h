#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace freeink {
using Uc8279X4FactoryRows = std::array<std::array<uint8_t, 49>, 5>;

struct Uc8279X4PanelPower {
  bool valid = false;
  std::array<uint8_t, 5> pwr{};
  std::array<uint8_t, 3> booster{};
  uint8_t vcom = 0, pll = 0;
};

// Profile from the observed 0x68 module. The five standard B/W tables
// share the power tuple verified on hardware at the former 30 C setting.
// Higher tables have different recipes or analogue codes and are excluded.
struct Uc8279X4FactoryProfile {
  Uc8279X4PanelPower power{};
  Uc8279X4FactoryRows rows{};
  uint8_t range = 0;
  int upperC = 0;
};

inline Uc8279X4FactoryProfile observedX4FactoryProfile(
    const uint8_t* mtp, size_t size, bool verified, uint8_t controllerId, int temperatureC) {
  if (!verified || !mtp || size != 4096 || controllerId != 0x68 ||
      mtp[0] != 0xA5 || mtp[1] != 0xA5 || mtp[0x1A] != 0x68 ||
      mtp[11] != 3 || mtp[12] != 0x20 || mtp[13] != 2 || mtp[14] != 0x58 ||
      temperatureC < -25 || temperatureC > 80)
    return {};
  constexpr std::array<uint8_t, 8> boundaries{5,10,15,20,80,95,100,127};
  for (size_t i = 0; i < boundaries.size(); ++i)
    if (mtp[0x28 + i] != boundaries[i]) return {};
  uint8_t range = 0;
  while (temperatureC > int(mtp[0x28 + range])) ++range;
  const size_t table = 0x33 + size_t(range) * 0x121;
  constexpr std::array<uint8_t, 9> header{9,0,0x78,0x78,0x14,0x24,0,0,0};
  for (size_t i = 0; i < header.size(); ++i)
    if (mtp[table + i] != header[i]) return {};
  if (mtp[4] != 0x25 || mtp[5] != 0x25 || mtp[6] != 0x3C) return {};
  Uc8279X4FactoryProfile profile;
  profile.range = range;
  profile.upperC = mtp[0x28 + range];
  profile.power = {true, {0x43, mtp[table+1], mtp[table+2], mtp[table+3], mtp[table+4]},
                   {mtp[4], mtp[5], mtp[6]}, mtp[table+5], mtp[table]};
  for (size_t row = 0; row < profile.rows.size(); ++row)
    for (size_t byte = 0; byte < profile.rows[row].size(); ++byte)
      profile.rows[row][byte] = mtp[table + header.size() + row * 49 + byte];
  return profile;
}

inline Uc8279X4PanelPower observedX4PanelPower(const uint8_t* mtp, size_t size,
                                            bool verified, uint8_t controllerId) {
  return observedX4FactoryProfile(mtp, size, verified, controllerId, 30).power;
}
}  // namespace freeink
