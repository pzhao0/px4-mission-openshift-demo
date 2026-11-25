#pragma once

#include <array>

namespace survey {

constexpr int kPasses = 6;
constexpr float kNorthStart = 22.f;
constexpr float kLaneSpacing = 4.4f;
constexpr float kEastStart = 2.f;
constexpr float kEastEnd = 30.f;
constexpr float kDown = -5.f;

constexpr float laneNorth(int lane) { return kNorthStart + lane * kLaneSpacing; }

constexpr std::array<float, 3> endpoint(int lane, bool entry)
{
  return {laneNorth(lane), (lane % 2 == 0) == entry ? kEastStart : kEastEnd, kDown};
}

}  // namespace survey
