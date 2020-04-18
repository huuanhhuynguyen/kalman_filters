#include "EKF.h"
#include "iostream"

void LKF::update(const VectorXd &z, const VectorXd& u, double dt)
{
  pM->update(dt);
  K = P * pM->H.transpose() * (pM->H * P * pM->H.transpose() + R).inverse();
  X = X + K * (z - pM->H * X);
  P = (I - K * pM->H) * P * (I - K * pM->H).transpose() + K * R * K.transpose();
}

VectorXd LKF::predict(const VectorXd& u, double dt)
{
  pM->update(dt);
  X = pM->F * X + pM->G * u;
  P = pM->F * P * pM->F.transpose() + Q;
  return X;
}


