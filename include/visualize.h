#ifndef KALMAN_FILTERS_CPP_VISUALIZE_H
#define KALMAN_FILTERS_CPP_VISUALIZE_H

#include "sample.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

void vis_meas(const std::vector<Sample>& m)
{
  auto get_x = [](const auto& m) {
    float x = 0;
    if (m.sensor == Sample::Sensor::LIDAR) {
      x = m.data[0];
    } else {
      auto rho = m.data[0];
      auto phi = m.data[1];
      x = rho * cos(phi);
    }
    return x;
  };

  auto get_y = [](const auto& m) {
    float y = 0;
    if (m.sensor == Sample::Sensor::LIDAR) {
      y = m.data[1];
    } else {
      auto rho = m.data[0];
      auto phi = m.data[1];
      y = rho * sin(phi);
    }
    return y;
  };

  std::vector<float> x, y;
  std::transform(m.begin(), m.end(), std::back_inserter(x), get_x);
  std::transform(m.begin(), m.end(), std::back_inserter(y), get_y);
  plt::scatter(x, y, /*s=*/3, {{"color", "blue"}});
}

void vis_gt(const std::vector<Sample>& gt)
{
  std::vector<float> x_gt, y_gt;
  std::transform(gt.begin(), gt.end(), std::back_inserter(x_gt), [](const auto& g){ return g.data[0]; });
  std::transform(gt.begin(), gt.end(), std::back_inserter(y_gt), [](const auto& g){ return g.data[1]; });
  plt::scatter(x_gt, y_gt, /*s=*/3, {{"color", "red"}});
}

#endif //KALMAN_FILTERS_CPP_VISUALIZE_H
