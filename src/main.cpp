#include "sample.h"
#include "kf/KFFactory.h"
#include "model/linear.h"
#include "model/nonlinear.h"
#include "fusion.h"
#include "visualize.h"

enum Data : uint8_t {
  ONE, TWO, THREE,
};

enum FilterType : uint8_t {
  EXTENDED, UNSCENTED,
};

// App Configuration
Data data = Data::THREE;
FilterType type = FilterType::EXTENDED;

int main()
{
  // Choose data path
  std::string path;
  if (data == Data::ONE) {
    path = "../data/1.txt";
  } else if (data == Data::TWO) {
    path = "../data/2.txt";
  } else {
    path ="../data/3.txt";
  }

  // read measurement and ground truth data
  std::vector<Sample> measurement, gt;
  std::string file{path};
  read(file, measurement, gt);

  // Initialize uncertainty matrices
  int Sx = 4;
  MatrixXd Q_in = MatrixXd::Identity(Sx, Sx) * 15;  // laser and radar share the same process, thus same process uncertainty
  MatrixXd R_in_laser = MatrixXd::Identity(2, 2) * 0.01;
  MatrixXd R_in_radar = MatrixXd::Identity(3, 3) * 0.01;

  // Initialize kalman filters
  std::unique_ptr<IKalmanFilter> pLKF, pRKF;
  if (type == FilterType::EXTENDED) {
    pLKF = KFFactory::manufacture_ekf(std::make_unique<LaserModel>(), Q_in, R_in_laser);
    pRKF = KFFactory::manufacture_ekf(std::make_unique<RadarModel>(), Q_in, R_in_radar);
  } else {
    pLKF = KFFactory::manufacture_ukf(std::make_unique<LaserModel>(), Q_in, R_in_laser);
    pRKF = KFFactory::manufacture_ukf(std::make_unique<RadarModel>(), Q_in, R_in_radar);
  }

  // Initialize intial state (untuned, just non-zeros), a correct KF should be
  // able to correct itself and converge to gt values.
  VectorXd X0 = VectorXd::Ones(Sx);

  // Update and fuse prediction
  Fusion fusion(std::move(pLKF), std::move(pRKF), X0);
  auto pred_pos = fusion.process(measurement);

  // Visualize measurement, estimates & ground-truth
  plt::figure();

  if (data == Data::ONE) {
    plt::xlim(3, 13);
    plt::ylim(-14, 1);
  } else if (data == Data::TWO) {
    plt::xlim(0, 210);
    plt::ylim(0, 40);
  } else {
    plt::xlim(-30, 25);
    plt::ylim(-12, 25);
  }

  vis_meas(measurement);
  vis_pred(pred_pos);
  vis_gt(gt);

  plt::legend();
  plt::show();

  return 0;
}