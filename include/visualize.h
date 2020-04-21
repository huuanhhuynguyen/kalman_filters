#ifndef KALMAN_FILTERS_CPP_VISUALIZE_H
#define KALMAN_FILTERS_CPP_VISUALIZE_H

#include "sample.h"
#include "fusion.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;


void vis_pred(const std::vector<Position>& p, std::string color = "green")
{
  std::vector<double> x, y;
  x.reserve(p.size());
  y.reserve(p.size());
  std::transform(p.begin(), p.end(), std::back_inserter(x), [](const auto& p){ return p.x; });
  std::transform(p.begin(), p.end(), std::back_inserter(y), [](const auto& p){ return p.y; });
  plt::scatter(x, y, /*s=*/3, {{"color", std::move(color)}});
}

void vis_gt(const std::vector<Sample>& gt)
{
  auto to_position = [](const auto& g) {
    auto x = g.data[0];
    auto y = g.data[2];
    return Position(x, y);
  };
  std::vector<Position> p;
  p.reserve(gt.size());
  std::transform(gt.begin(), gt.end(), std::back_inserter(p), to_position);

  vis_pred(p, "red");
}

void vis_meas(const std::vector<Sample>& meas)
{
  auto to_position = [](const auto& m) {
    double x = 0, y = 0;
    if (m.sensor == Sample::Sensor::LIDAR) {
      x = m.data[0];
      y = m.data[1];
    } else {
      auto rho = m.data[0];
      auto phi = m.data[1];
      x = rho * cos(phi);
      y = rho * sin(phi);
    }
    return Position(x, y);
  };

  std::vector<Position> p;
  p.reserve(meas.size());
  std::transform(meas.begin(), meas.end(), std::back_inserter(p), to_position);
  vis_pred(p, "blue");
}

#endif //KALMAN_FILTERS_CPP_VISUALIZE_H
