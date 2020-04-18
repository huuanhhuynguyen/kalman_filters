#include <vector>
#include "sample.h"
#include "EKF"
#include "angle.h"
#include "visualize.h"

int main()
{

  // read measurement and ground truth data
  std::vector<Sample> measurement, gt;
  std::string file{"../data/sample-laser-radar-measurement-data-1.txt"};
  read(file, measurement, gt);

  // Initialize a concrete kalman filter, using Enum and factory pattern
      // Option 1: Linear KF
      // Option 2: EKF
      // Option 3: UKF

  // Update & Predict -> Prediction data


  // Calculate RMSE

  // Estimate heading angle of gt for visualisation
  //const auto headings = estimate_headings(gt);

  plt::figure();
  plt::xlim(3, 13);
  plt::ylim(-14, 1);

  // Visualize measurement
  vis_meas(measurement);

  // Visualize ground-truth
  vis_gt(gt);

  // Visualize prediction
  // Visualize RMSE as a chart

  plt::show();
  return 0;
}
