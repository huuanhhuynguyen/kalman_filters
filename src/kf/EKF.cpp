#include "kf/EKF.h"

void EKF::update(const VectorXd &z, const VectorXd& u, double dt)
{
  MatrixXd J_h = pM->J_h(X);

  K = P * J_h.transpose() * (J_h * P * J_h.transpose() + R).inverse();
  X = X + K * (z - pM->h(X));
  P = (I - K * J_h) * P * (I - K * J_h).transpose() + K * R * K.transpose();
}

VectorXd EKF::predict(const VectorXd& u, double dt)
{
  MatrixXd J_f = pM->J_f(X);

  X = pM->f(X, u, dt);
  P = J_f * P * J_f.transpose() + Q;
  return X;
}


