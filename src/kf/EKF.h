#ifndef KALMAN_FILTERS_CPP_EKF_H
#define KALMAN_FILTERS_CPP_EKF_H

#include "KF.h"
#include "model/model.h"
#include <memory>

/**
 *  Extended (or Linear) Kalman Filter
 *  Equations linear KF: https://www.kalmanfilter.net/multiSummary.html
 *  Equations extended KF: https://www.cse.sc.edu/~terejanu/files/tutorialEKF.pdf
 *  Update equations:
 *      x = f(x, u, dt)
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
  using MPtr = std::unique_ptr<ILinearizableModel>;

  explicit EKF(MPtr pModel,
               const MatrixXd& Q_in,
               const MatrixXd& R_in) : pM{std::move(pModel)}
  {
    auto Sx = pM->Sx();
    // auto Su = pM->Su();
    auto Sz = pM->Sz();
    X = VectorXd::Zero(Sx);
    P = MatrixXd::Identity(Sx, Sx);
    Q = Q_in;
    R = R_in;
    K = MatrixXd::Zero(Sx, Sz);
    I = MatrixXd::Identity(Sx, Sx);
  };

  void update(const VectorXd& z, const VectorXd& u) override;
  VectorXd predict(const VectorXd& u, double dt) override;

private:
  MPtr pM;     // Model Pointer

  MatrixXd Q;  // Process Uncertainty
  MatrixXd R;  // Measurement Uncertainty
  MatrixXd K;  // Kalman Gain
  MatrixXd I;  // Identity Matrix
};

#endif //KALMAN_FILTERS_CPP_EKF_H
