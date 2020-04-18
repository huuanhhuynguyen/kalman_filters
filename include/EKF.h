#ifndef KALMAN_FILTERS_CPP_EKF_H
#define KALMAN_FILTERS_CPP_EKF_H

#include "KF.h"

/**
 *  Linear Kalman Filter
 *  Equations: https://www.kalmanfilter.net/multiSummary.html
 *  Update equations:
 *      x = F * x + G * u
 *      P = F * P * Ft + Q
 *  Predict equations:
 *      K = P * Ht * (H * P * Ht + R).inv()
 *      x = x + K * (z - H * x)
 *      P = (I - K * H) * P * (I - K * H)t + K * R * Kt
 */
class EKF {
public:

  void update(const VectorXd& z, const VectorXd* u, float dt);
  VectorXd predict(float dt);

private:
  MatrixXd F;        // Transition Matrix
  MatrixXd G;        // Input Matrix
  MatrixXd P;        // Estimate Uncertainty (Covariance Matrix)
  MatrixXd Q;        // Process Uncertainty
  MatrixXd K;        // Kalman Gain
  MatrixXd R;        // Measurement Uncertainty
  MatrixXd H;        // Output Matrix
  static MatrixXd I; // Identity Matrix

  VectorXd x;        // Current State Vector

  void _set_F(float dt) {};
};

#endif //KALMAN_FILTERS_CPP_EKF_H
