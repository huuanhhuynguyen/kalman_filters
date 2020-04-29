#include "sample.h"
#include "kf/KFFactory.h"
#include "model/linear.h"
#include "model/nonlinear.h"
#include "fusion.h"
#include "visualize.h"

int main()
{
  // read measurement and ground truth data
  std::vector<Sample> measurement, gt;
  std::string file{"../data/3.txt"};
  read(file, measurement, gt);

  // Initialize uncertainty matrices
  int Sx = 4;
  MatrixXd Q_in = MatrixXd::Identity(Sx, Sx) * 15;  // laser and radar share the same process, thus same process uncertainty
  MatrixXd R_in_laser = MatrixXd::Identity(2, 2) * 0.01;
  MatrixXd R_in_radar = MatrixXd::Identity(3, 3) * 0.01;

  // Initialize extended kalman filters
  auto pLKF = KFFactory::manufacture_ekf(std::make_unique<LaserModel>(), Q_in, R_in_laser);
  auto pRKF = KFFactory::manufacture_ekf(std::make_unique<RadarModel>(), Q_in, R_in_radar);

  //auto pLKF = KFFactory::manufacture_ukf(std::make_unique<LaserModel>(), Q_in, R_in_laser);
  //auto pRKF = KFFactory::manufacture_ukf(std::make_unique<RadarModel>(), Q_in, R_in_radar);

  // Init intial state (untuned, just non-zeros), a correct KF should be able to
  // correct itself and converge to gt values.
  VectorXd X0 = VectorXd::Ones(Sx);

  Fusion fusion(std::move(pLKF), std::move(pRKF), X0);

  // Update & predict positions
  auto pred_pos = fusion.process(measurement);

  // Visualize measurement, estimates & ground-truth
  plt::figure();
  // for 1.txt
  //plt::xlim(3, 13);
  //plt::ylim(-14, 1);
  // for 2.txt
  //plt::xlim(0, 210);
  //plt::ylim(0, 40);
  // for 3.txt
  plt::xlim(-30, 25);
  plt::ylim(-12, 25);

  vis_meas(measurement);
  vis_pred(pred_pos);
  vis_gt(gt);

  plt::legend();

  plt::show();
  return 0;
}