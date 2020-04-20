#ifndef KALMAN_FILTERS_CPP_EKF_H
#define KALMAN_FILTERS_CPP_EKF_H

#include "KF.h"
#include "model.h"
#include <memory>

/**
 *  Extended (or Linear) Kalman Filter
 *  Equations linear KF: https://www.kalmanfilter.net/multiSummary.html
 *  Equations extended KF: https://www.cse.sc.edu/~terejanu/files/tutorialEKF.pdf
 *  Update equations:
 *      x = f(x) + g(u)
 *      P = J_f * P * J_f.transpose() + Q
 *  Predict equations:
 *      K = P * J_h.t * (J_h * P * J_h.t + R).i
 *      x = x + K * (z - h(x))
 *      P = (I - K * J_h) * P * (I - K * J_h).t + K * R * K.t
 *  where .t and .i for transpose and inverse operations.
 *
 *  Filter is linear or extended, depending on the input model to the filter.
 */
class EKF : public IKalmanFilter {
public:
  using MPtr = std::unique_ptr<IModel>;

  explicit EKF(MPtr pModel) : pM{std::move(pModel)} {
    auto Sz = pM->J_h().rows();
    auto Sx = pM->J_h().cols();
    auto Su = pM->J_g().cols();
    X = VectorXd::Zero(Sx);
    P = MatrixXd::Identity(Sx, Sx);
    Q = MatrixXd::Identity(Sx, Sx) * 1; // * 0.001; //* 10;
    R = MatrixXd::Identity(Sz, Sz) * 0.02; // * 0.009; //* 0.02;
    K = MatrixXd::Zero(Sx, Sz);
    I = MatrixXd::Identity(Sx, Sx);
  };

  void update(const VectorXd& z, const VectorXd& u, double dt) override;
  VectorXd predict(const VectorXd& u, double dt) override;

private:
  MPtr pM;     // Model Pointer

  MatrixXd Q;  // Process Uncertainty
  MatrixXd R;  // Measurement Uncertainty
  MatrixXd K;  // Kalman Gain
  MatrixXd I;  // Identity Matrix
};

#endif //KALMAN_FILTERS_CPP_EKF_H
