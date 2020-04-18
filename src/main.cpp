#include <vector>
#include "sample.h"
#include "angle.h"

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
  const auto headings = estimate_headings(gt);

  // Iterate over the time stamp
      // Visualize measurement
      // Visualize prediction
      // Visualize ground-truth
      // Visualize RMSE as a chart
      // Display as a video

  return 0;
}
