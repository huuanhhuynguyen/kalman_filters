#ifndef SELF_03_KALMAN_FILTERS_CPP_KF_H
#define SELF_03_KALMAN_FILTERS_CPP_KF_H

#include "Eigen/Dense"

using namespace Eigen;

/**
 * Kalman Filter Interface
 * */
class IKalmanFilter {
public:
  virtual void update(const VectorXd& z, const VectorXd& u, double dt) = 0;

  /// Returns the estimated state
  virtual VectorXd predict(const VectorXd& u, double dt) = 0;
};

#endif //SELF_03_KALMAN_FILTERS_CPP_KF_H
