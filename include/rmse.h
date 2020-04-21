#ifndef KALMAN_FILTERS_CPP_RMSE_H
#define KALMAN_FILTERS_CPP_RMSE_H

#include "fusion.h"
#include "sample.h"

std::vector<double>
calculate_rmse(const std::vector<Position>& lhs, const std::vector<Position>& rhs)
{
  std::vector<double> res;
  for (int i = 1; i < lhs.size(); ++i) {
    auto x1 = lhs[i].x;
    auto y1 = lhs[i].y;
    auto x = rhs[i].x;
    auto y = rhs[i].y;
    auto rmse = sqrt( (x-x1)*(x-x1) + (y-y1)*(y-y1) );
    res.push_back(rmse);
  }
  return res;
}

#endif //KALMAN_FILTERS_CPP_RMSE_H
