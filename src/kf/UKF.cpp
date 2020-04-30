#include "kf/UKF.h"

void UKF::update(const VectorXd &z, const VectorXd &u)
{
  // Z = h(sigma)
  MatrixXd Z(z.size(), n_sigma);
  for (int i = 0; i < n_sigma; ++i) {
    Z.col(i) = pM->h(sigma.col(i));
  }

  VectorXd z_hat = Z * weights_m;

  // Pzz = R + Sum_i(weights_i * (Z_i - z_hat) * (Z_i - z_hat).transpose())
  MatrixXd Pzz = R;
  for (int i = 0; i < n_sigma; ++i) {
    Pzz += weights_c(i) * (Z.col(i) - z_hat) * (Z.col(i) - z_hat).transpose();
  }

  // Pxz = Sum_i(weights_i * (sigma_i - X) * (Z_i - z_hat).transpose())
  MatrixXd Pxz = MatrixXd::Zero(X.size(), z.size());
  for (int i = 0; i < n_sigma; ++i) {
    Pxz += weights_c(i) * (sigma.col(i) - X) * (Z.col(i) - z_hat).transpose();
  }

  K = Pxz * Pzz.inverse();
  X = X + K * (z - z_hat);
  P = P - K * Pzz * K.transpose();
}

VectorXd UKF::predict(const VectorXd &u, double dt)
{
  sigma = compute_sigma_points(X, P);

  // sigma_p1 = f(sigma)
  // X = Sum_i(weights_i * f(sigma_p1_i))
  X.fill(0.0);
  for (int i = 0; i < n_sigma; ++i) {
    sigma.col(i) = pM->f(sigma.col(i), u, dt);
    X += weights_m(i) * sigma.col(i);
  }

  // P = Q + Sum_i(weights_i * (sigma_p1_i - X) * (sigma_p1_i - X).transpose())
  P = Q;
  for (int i = 0; i < n_sigma; ++i) {
    auto tmp = sigma.col(i) - X;
    P += weights_c(i) * tmp * tmp.transpose();
  }

  return X;
}





