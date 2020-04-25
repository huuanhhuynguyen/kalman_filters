#include "sample.h"
#include "kf/KFFactory.h"
#include "model/linear.h"
#include "model/nl_model.h"
#include "fusion.h"
#include "visualize.h"

int main()
{
  // read measurement and ground truth data
  std::vector<Sample> measurement, gt;
  std::string file{"../data/obj_pose-laser-radar-synthetic-input.txt"};
  read(file, measurement, gt);

  // Initialize kalman filters
  auto pLKF = KFFactory::manufacture(KFType::EXTENDED,
                                     std::make_unique<LaserModel>());
  auto pRKF = KFFactory::manufacture(KFType::EXTENDED,
                                     std::make_unique<RadarModel>());

  // Init intial state (untuned, just zeros)
  VectorXd X0 = VectorXd::Zero(4);

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

  // Visualize measurement, estimates & ground-truth
  plt::figure();
  //plt::xlim(3, 13);
  //plt::ylim(-14, 1);
  //plt::xlim(0, 210);
  //plt::ylim(0, 40);

  vis_meas(measurement);
  vis_pred(pred_pos);
  vis_gt(gt);

  plt::legend();

  plt::show();
  return 0;
}