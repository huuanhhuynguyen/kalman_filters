#ifndef KALMAN_FILTERS_CPP_EKF_H
#define KALMAN_FILTERS_CPP_EKF_H

#include "KF.h"
#include "model.h"
#include <memory>

/**
 *  Extended (or Linear) Kalman Filter
 *  Equations: https://www.kalmanfilter.net/multiSummary.html
 *  Update equations:
 *      x = F * x + G * u
 *      P = F * P * Ft + Q
 *  Predict equations:
 *      K = P * Ht * (H * P * Ht + R).inv()
 *      x = x + K * (z - H * x)
 *      P = (I - K * H) * P * (I - K * H)t + K * R * Kt
 *  Filter is linear or extended, depending on the input model to the filter.
 */
class EKF : public IKalmanFilter {
public:
  using MPtr = std::unique_ptr<IModel>;

  explicit EKF(MPtr pModel) : pM{std::move(pModel)} {
    auto Sz = pM->H().rows();
    auto Sx = pM->H().cols();
    auto Su = pM->G().cols();
    X = VectorXd::Zero(Sx);
    P = MatrixXd::Identity(Sx, Sx);
    Q = MatrixXd::Identity(Sx, Sx);
    R = MatrixXd::Identity(Sz, Sz);
    K = MatrixXd::Zero(Sx, Sz);
    I = MatrixXd::Identity(Sx, Sx);
  };

  void update(const VectorXd& z, const VectorXd& u, double dt) override;
  VectorXd predict(const VectorXd& u, double dt) override;

private:
  MPtr pM;

  MatrixXd P;  // Estimate Uncertainty (Covariance Matrix)
  MatrixXd Q;  // Process Uncertainty
  MatrixXd R;  // Measurement Uncertainty
  MatrixXd K;  // Kalman Gain
  MatrixXd I;  // Identity Matrix
};

#endif //KALMAN_FILTERS_CPP_EKF_H
