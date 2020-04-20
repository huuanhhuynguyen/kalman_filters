#include "EKF.h"
#include <iostream>

void EKF::update(const VectorXd &z, const VectorXd& u, double dt) {
  pM->update(dt, X);
  MatrixXd J_h = pM->J_h();

  K = P * J_h.transpose() * (J_h * P * J_h.transpose() + R).inverse();
  X = X + K * (z - pM->h(X));
  P = (I - K * J_h) * P * (I - K * J_h).transpose() + K * R * K.transpose();
}

VectorXd EKF::predict(const VectorXd& u, double dt)
{
  pM->update(dt, X);
  MatrixXd J_f = pM->J_f();

  X = pM->f(X) + pM->g(u);
  P = J_f * P * J_f.transpose() + Q;
  return X;
}


