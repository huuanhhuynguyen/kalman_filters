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

  explicit UKF(MPtr pModel,
               const MatrixXd& Q_in,
               const MatrixXd& R_in) : pM{std::move(pModel)}
  {
    auto Sx = pM->Sx();
    auto Su = pM->Su();
    auto Sz = pM->Sz();

    X = VectorXd::Zero(Sx);
    P = MatrixXd::Identity(Sx, Sx);
    Q = Q_in;
    R = R_in;
    K = MatrixXd::Zero(Sx, Sz);
    I = MatrixXd::Identity(Sx, Sx);

    n_sigma = 2*Sx + 1;
    sigma = MatrixXd::Zero(Sx, n_sigma);
    weights = compute_sigma_weights(Sx);
  }

  void update(const VectorXd& z, const VectorXd& u) override;
  VectorXd predict(const VectorXd& u, double dt) override;

private:
  MPtr pM;     // Model Pointer

  MatrixXd Q;  // Process Uncertainty
  MatrixXd R;  // Measurement Uncertainty
  MatrixXd K;  // Kalman Gain
  MatrixXd I;  // Identity Matrix

  int n_sigma;       // number of sigma points
  MatrixXd sigma;    // sigma points
  VectorXd weights;  // weights for sigma points
};

#endif //KALMAN_FILTERS_CPP_UKF_H
