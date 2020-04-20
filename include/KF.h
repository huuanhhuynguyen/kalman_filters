#ifndef SELF_03_KALMAN_FILTERS_CPP_KF_H
#define SELF_03_KALMAN_FILTERS_CPP_KF_H

#include "Eigen/Dense"

using namespace Eigen;

/**
 * Kalman Filter Interface
 * */
class IKalmanFilter {
public:
  virtual ~IKalmanFilter() = default;

  /**
   * Update Kalman Filter
   * @param z measurement
   * @param u input
   * @param dt time step
   */
  virtual void update(const VectorXd& z, const VectorXd& u, double dt) = 0;

  /**
   * Estimates the next state
   * @param u input
   * @param dt time step
   * @return next state
   */
  virtual VectorXd predict(const VectorXd& u, double dt) = 0;

  VectorXd X;  // Current State Vector
  MatrixXd P;  // Estimate Uncertainty (Covariance Matrix)
};

#endif //SELF_03_KALMAN_FILTERS_CPP_KF_H
