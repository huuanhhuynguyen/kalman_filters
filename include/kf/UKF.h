#ifndef KALMAN_FILTERS_CPP_UKF_H
#define KALMAN_FILTERS_CPP_UKF_H

#include "KF.h"
#include "sigma_points.h"
#include "model/model.h"
#include <memory>

/**
 * Unscented Kalman Filter
 * Paper: https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf
 * Hints for programming UKF:
 * https://towardsdatascience.com/the-unscented-kalman-filter-anything-ekf-can-do-i-can-do-it-better-ce7c773cf88d
 */
class UKF : public IKalmanFilter {
public:
  using MPtr = std::unique_ptr<IModel>;

  explicit UKF(MPtr pModel) : pM{std::move(pModel)} {
    auto Sx = pM->Sx();
    auto Su = pM->Su();
    auto Sz = pM->Sz();
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
  MPtr pM;     // Model Pointer

  MatrixXd Q;  // Process Uncertainty
  MatrixXd R;  // Measurement Uncertainty
  MatrixXd K;  // Kalman Gain
  MatrixXd I;  // Identity Matrix
};

#endif //KALMAN_FILTERS_CPP_UKF_H
