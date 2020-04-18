#include <vector>
#include "sample.h"
#include "KFFactory.h"
#include "angle.h"
#include "visualize.h"

int main()
{

  // read measurement and ground truth data
  std::vector<Sample> measurement, gt;
  std::string file{"../data/sample-laser-radar-measurement-data-2.txt"};
  read(file, measurement, gt);

  // Initialize a concrete kalman filter
  auto pKF = KFFactory::manufacture(KFType::EXTENDED,
                                    std::make_unique<VelocityModel>());

  // Update & Predict -> Prediction data
  std::vector<double> x_hat, y_hat;

  for (int i = 1; i < measurement.size(); ++i) {
    auto& m = measurement[i];
    double dt = double(measurement[i].t - measurement[i-1].t) / 1.0e6;
    if (m.sensor == Sample::Sensor::LIDAR) {
      auto x = m.data[0];
      auto y = m.data[1];
      VectorXd z(2);
      z << x, y;
      VectorXd u(1);
      u << 0;
      pKF->update(z, u, dt);
      auto X_hat = pKF->predict(u, dt);
      x_hat.emplace_back(X_hat[0]);
      y_hat.emplace_back(X_hat[1]);
    }
  }

  // Calculate RMSE

  // Estimate heading angle of gt for visualisation
  //const auto headings = estimate_headings(gt);

  plt::figure();
  //plt::xlim(3, 13);
  //plt::ylim(-14, 1);

  // Visualize measurement
  vis_meas(measurement);

  // Visualize ground-truth
  vis_gt(gt);

  // Visualize prediction
  vis_prediction(x_hat, y_hat);
  // Visualize RMSE as a chart

  plt::show();
  return 0;
}
