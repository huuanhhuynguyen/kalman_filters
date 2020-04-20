#ifndef KALMAN_FILTERS_CPP_RMSE_H
#define KALMAN_FILTERS_CPP_RMSE_H

#include "fusion.h"
#include "sample.h"

std::vector<double>
calculate_rmse(std::vector<Sample> gt, std::vector<Position> prediction)
{
  std::vector<double> res;
  // prediction is only available from the second sample. Therefore, only
  // calculate rmse on the second sample onwards.
  for (int i = 1; i < gt.size(); ++i) {
    auto g = gt[i];
    auto p = prediction[i-1];
    auto x_gt = g.data[0];
    auto y_gt = g.data[1];
    auto x = p.x;
    auto y = p.y;
    auto rmse = sqrt( (x-x_gt)*(x-x_gt) + (y-y_gt)*(y-y_gt) );
    res.push_back(rmse);
  }
  return res;
}

#endif //KALMAN_FILTERS_CPP_RMSE_H
