#include "sample.h"
#include "kf/KFFactory.h"
#include "model/linear.h"
#include "model/nl_model.h"
#include "fusion.h"
#include "rmse.h"
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
  X0 << 8.4, 0, 0.25, 0;

  Fusion fusion(std::move(pLKF), std::move(pRKF), X0);

  // Update & predict positions
  auto pred_pos = fusion.process(measurement);

  // Calculate RMSE
  std::vector<Position> gt_pos, meas_pos;
  std::transform(gt.begin(), gt.end(), std::back_inserter(gt_pos),
      [](const auto& g){ return Position(g.data[0], g.data[1]); });
  {
    auto get_pos = [](const auto& m){
      if (m.sensor == Sample::Sensor::LIDAR) {
        return Position(m.data[0], m.data[1]);
      } else {
        double rho = m.data[0];
        double phi = m.data[1];
        double x = rho * cos(phi);
        double y = rho * sin(phi);
        return Position(x, y);
      }
    };
    std::transform(measurement.begin(), measurement.end(), std::back_inserter(meas_pos), get_pos);
  }
  auto rmse_meas = calculate_rmse(gt_pos, meas_pos);
  auto mean_meas = std::accumulate( rmse_meas.begin(), rmse_meas.end(), 0.0) / rmse_meas.size();
  //TODO clean up
  auto rmse_pred = calculate_rmse(std::vector<Position>(gt_pos.begin(), gt_pos.end()-1), pred_pos);
  auto mean_pred = std::accumulate( rmse_pred.begin(), rmse_pred.end(), 0.0) / rmse_pred.size();


  // Estimate heading angle of gt for visualisation
  //const auto headings = estimate_headings(gt);

  plt::figure();
  plt::xlim(3, 13);
  plt::ylim(-14, 1);
  //plt::xlim(0, 210);
  //plt::ylim(0, 40);

  // Visualize measurement
  vis_meas(measurement);

  // Visualize prediction
  vis_pred(pred_pos);

  // Visualize ground-truth
  vis_gt(gt);

  // Visualize RMSE as a chart

  plt::show();
  return 0;
}

// TODO tranform back and forward between polar (rho, etc.) and cartesian (x,y) should be
// in a single place