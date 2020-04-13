#ifndef SELF_03_KALMAN_FILTERS_CPP_KF_H
#define SELF_03_KALMAN_FILTERS_CPP_KF_H

#include "Eigen/Dense"

namespace kf {

using namespace Eigen;

/**
 * Kalman Filter Interface
 * */
class IKalmanFilter {
public:
  virtual void update(const VectorXd& measurement,
                      const VectorXd* input, float dt) = 0;
  virtual VectorXd predict(float dt) = 0;
};

} // namespace kf

#endif //SELF_03_KALMAN_FILTERS_CPP_KF_H
