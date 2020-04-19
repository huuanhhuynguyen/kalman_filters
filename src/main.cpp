#include <vector>
#include "sample.h"
#include "KFFactory.h"
#include "fusion.h"
#include "angle.h"
#include "visualize.h"

int main()
{
  // read measurement and ground truth data
  std::vector<Sample> measurement, gt;
  std::string file{"../data/sample-laser-radar-measurement-data-1.txt"};
  read(file, measurement, gt);

  // Initialize kalman filters
  auto pLKF = KFFactory::manufacture(KFType::EXTENDED,
                                    std::make_unique<LaserModel>());
  auto pRKF = KFFactory::manufacture(KFType::EXTENDED,
                                     std::make_unique<RadarModel>());

  // Init intial state
  VectorXd X0(4);
  X0 << 1.5, 0, -0.1, 0;

  Fusion fusion(std::move(pLKF), std::move(pRKF), X0);

  // Update & Predict
  auto positions = fusion.process(measurement);

  // Calculate RMSE

  // Estimate heading angle of gt for visualisation
  //const auto headings = estimate_headings(gt);

  plt::figure();
  plt::xlim(3, 13);
  plt::ylim(-14, 1);
  //plt::xlim(0, 210);
  //plt::ylim(0, 40);

  // Visualize measurement
  vis_meas(measurement);

  // Visualize ground-truth
  vis_gt(gt);

  // Visualize prediction
  vis_prediction(positions);

  // Visualize RMSE as a chart

  plt::show();
  return 0;
}
