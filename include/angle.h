#ifndef KALMAN_FILTERS_CPP_ANGLE_H
#define KALMAN_FILTERS_CPP_ANGLE_H

#include "sample.h"
#include <math.h>

template <typename T>
void normalize(T& angle) {
  while (angle > 2*M_PI) {
    angle -= 2*M_PI;
  }
  while (angle < -2*M_PI) {
    angle += 2*M_PI;
  }
}

std::vector<float>
estimate_headings(const std::vector<Sample>& gt)
{
  std::vector<float> headings;

  for (int i = 1; i < gt.size(); ++i) {
    const auto &g = gt[i - 1];
    const auto &g1 = gt[i];
    auto x = g.data[0];
    auto y = g.data[1];
    auto x1 = g1.data[0];
    auto y1 = g1.data[0];
    auto heading = atan2(y1 - y, x1 - x + 1.0e-6); // avoid zero division
    normalize(heading);
    headings.emplace_back(heading);
  }

  return headings;
}

#endif //KALMAN_FILTERS_CPP_ANGLE_H
