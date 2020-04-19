#include "EKF.h"

void EKF::update(const VectorXd &z, const VectorXd& u, double dt)
{
  pM->update(dt, X);
  MatrixXd H = pM->H();

  K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
  X = X + K * (z - H * X);
  P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();
}

VectorXd EKF::predict(const VectorXd& u, double dt)
{
  pM->update(dt, X);
  MatrixXd F = pM->F();
  MatrixXd G = pM->G();

  X = F * X + G * u;
  P = F * P * F.transpose() + Q;
  return X;
}


