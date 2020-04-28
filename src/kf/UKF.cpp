#include "kf/UKF.h"
#include <iostream>

void UKF::update(const VectorXd &z, const VectorXd &u, double dt)
{
  sigma = compute_sigma_points(X, P);

  // Z = h(sigma)
  MatrixXd Z(z.size(), n_sigma);
  for (int i = 0; i < n_sigma; ++i) {
    Z.col(i) = pM->h(sigma.col(i));
  }

  VectorXd z_hat = Z * weights;

  // Pzz = Sum_i(weights_i * (Z_i - z_hat) * (Z_i - z_hat).transpose())
  MatrixXd Pzz = R;
  for (int i = 0; i < n_sigma; ++i) {
    Pzz += weights(i) * (Z.col(i) - z_hat) * (Z.col(i) - z_hat).transpose();
  }

  // Pxz = Sum_i(weights_i * (sigma_i - X) * (Z_i - z_hat).transpose())
  MatrixXd Pxz = MatrixXd::Zero(X.size(), z.size());
  for (int i = 0; i < n_sigma; ++i) {
    Pxz += weights(i) * (sigma.col(i) - X) * (Z.col(i) - z_hat).transpose();
  }

  K = Pxz * Pzz.inverse();
  X = X + K * (z - z_hat);
  P = P - K * Pzz * K.transpose();
  std::cout << "P = \n" << P << std::endl;
}

VectorXd UKF::predict(const VectorXd &u, double dt)
{
  sigma = compute_sigma_points(X, P);

  // Update X
  X.fill(0.0);
  for (int i = 0; i < n_sigma; ++i) {
    X += weights(i) * pM->f(sigma.col(i), u, dt);
  }

  // Update P
  P = Q;
  for (int i = 0; i < n_sigma; ++i) {
    auto tmp = pM->f(sigma.col(i), u, dt) - X;
    P += weights(i) * tmp * tmp.transpose();
  }

  return X;
}





