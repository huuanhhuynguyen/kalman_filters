#ifndef KALMAN_FILTERS_CPP_LINEAR_MODEL_H
#define KALMAN_FILTERS_CPP_LINEAR_MODEL_H

#include "model.h"

class ILinearModel : public IModel {
public:
  VectorXd f(const VectorXd& X) const override { return F * X; }
  VectorXd g(const VectorXd& U) const override { return G * U; }
  VectorXd h(const VectorXd& X) const override { return H * X; }

protected:
  /** Linear models don't linearize anything */
  MatrixXd _linearize_F(const VectorXd& X0) override { return F; }
  MatrixXd _linearize_G(const VectorXd& X0) override { return G; }
  MatrixXd _linearize_H(const VectorXd& X0) override { return H; }
};

class LaserModel : public ILinearModel {
public:
  explicit LaserModel() {
    F = MatrixXd::Identity(4, 4);
    G = MatrixXd::Zero(4, 1);
    H = MatrixXd(2, 4);
    H << 1, 0, 0, 0,
         0, 0, 1, 0;
  }
private:

  void _update_dt(double dt) override {
    F = MatrixXd(4, 4);
    F << 1, dt, 0, 0,
         0,  1, 0, 0,
         0,  0, 1, dt,
         0,  0, 0, 1;
  }
};

#endif //KALMAN_FILTERS_CPP_LINEAR_MODEL_H
