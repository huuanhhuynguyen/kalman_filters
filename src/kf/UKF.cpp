#include "kf/UKF.h"
#include <iostream>

void UKF::update(const VectorXd &z, const VectorXd &u, double dt)
{
  sigma = compute_sigma_points(X, P);

  const int Sz = z.size();
  const int two_n_plus_1 = sigma.cols();

  MatrixXd Z(Sz, two_n_plus_1);
  for (unsigned int i = 0; i < two_n_plus_1; ++i) {
    Z.col(i) = pM->h(sigma.col(i));
  }
  std::cout << "Z = \n" << Z << std::endl;

  VectorXd z_hat = VectorXd::Zero(z.size());
  for (unsigned int i = 0; i < two_n_plus_1; ++i) {
    z_hat += weights[i] * Z.col(i);
  }
  std::cout << "z_hat = \n" << z_hat << std::endl;

  MatrixXd Pzz = R;
  for (unsigned int i = 0; i < two_n_plus_1; ++i) {
    Pzz += weights[i] * (Z.col(i) - z_hat) * (Z.col(i) - z_hat).transpose();
  }
  std::cout << "Pzz = \n" << Pzz << std::endl;

  MatrixXd Pxz = MatrixXd::Zero(X.size(), Sz);
  for (unsigned int i = 0; i < two_n_plus_1; ++i) {
    Pxz += weights[i] * (sigma.col(i) - X) * (Z.col(i) - z_hat).transpose();
  }
  std::cout << "Pxz = \n" << Pxz << std::endl;

  K = Pxz * Pzz.inverse();
  std::cout << "K = \n" << K << std::endl;
  std::cout << "z = \n" << z << std::endl;
  std::cout << "z - z_hat= \n" << z - z_hat << std::endl;
  X = X + K * (z - z_hat);
  P = P - K * Pzz * K.transpose();

  std::cout << "P = \n" << P << std::endl;
}

VectorXd UKF::predict(const VectorXd &u, double dt)
{
  sigma = compute_sigma_points(X, P);

  // Update state X (aka. the new estimated mean)
  X.fill(0.0);
  for (unsigned int i = 0; i < 2*X.rows(); ++i) {
    X += weights[i] * pM->f(sigma.col(i), u, dt);
  }

  // Update covariance matrix P
  P = Q;
  for (unsigned int i = 0; i < 2*X.rows(); ++i) {
    auto x_diff = pM->f(sigma.col(i), u, dt) - X;
    P += weights[i] * x_diff * x_diff.transpose();
  }

  return X;
}





