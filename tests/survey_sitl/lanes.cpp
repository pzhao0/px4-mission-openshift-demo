#include "SurveyLanes.hpp"
#include <cassert>

int main()
{
  using namespace survey;
  for (int lane = 0; lane < kPasses; ++lane) {
    const auto entry = endpoint(lane, true), exit = endpoint(lane, false);
    assert(entry[0] == laneNorth(lane) && entry[0] == exit[0]);
    assert(entry[1] == (lane % 2 ? 30.f : 2.f));
    assert(exit[1] == (lane % 2 ? 2.f : 30.f));
    assert(entry[2] == -5.f && exit[2] == -5.f);
  }
}
