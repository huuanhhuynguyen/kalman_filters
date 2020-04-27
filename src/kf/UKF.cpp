#include "kf/UKF.h"

void UKF::update(const VectorXd &z, const VectorXd &u, double dt)
{
  const auto sigma = compute_sigma_points(X, P);
  const auto weights = compute_sigma_weights(X.rows());

  const int Sz = z.size();
  const int two_n_plus_1 = sigma.cols();

  MatrixXd Z(Sz, two_n_plus_1);
  for (unsigned int i = 0; i < two_n_plus_1; ++i) {
    Z.col(i) = pM->h(sigma.col(i));
  }

  VectorXd z_hat = VectorXd::Zero(z.size());
  for (unsigned int i = 0; i < two_n_plus_1; ++i) {
    z_hat += weights[i] * Z.col(i);
  }

  MatrixXd S = R;
  for (unsigned int i = 0; i < two_n_plus_1; ++i) {
    S += weights[i] * (Z.col(i) - z_hat) * (Z.col(i) - z_hat).transpose();
  }

  MatrixXd T = MatrixXd::Zero(X.size(), Sz);
  for (unsigned int i = 0; i < two_n_plus_1; ++i) {
    T += weights[i] * (sigma.col(i) - X) * (Z.col(i) - z_hat).transpose();
  }

  K = T * S.inverse();

  X = X + K * (z - z_hat);
  P = P - K * S * K.transpose();
}

VectorXd UKF::predict(const VectorXd &u, double dt)
{
  const auto sigma = compute_sigma_points(X, P);
  const auto weights = compute_sigma_weights(X.rows());

  // Update state X (aka. the new estimated mean)
  X.fill(0.0);
  for (unsigned int i = 0; i < 2*X.rows(); ++i) {
    X += weights[i] * pM->f(sigma.col(i), u, dt);
  }

  // Update covariance matrix P
  P = Q;
  for (unsigned int i = 0; i < 2*X.rows(); ++i) {
    auto tmp = pM->f(sigma.col(i), u, dt) - X;
    P += weights[i] * tmp * tmp.transpose();
  }

  return X;
}





