#ifndef KALMAN_FILTERS_CPP_LINEAR_H
#define KALMAN_FILTERS_CPP_LINEAR_H

#include "model.h"

/**
 * Linear Model Interface
 *   x1 = F * x + G * u
 *   z  = H * x
 * Linearization of f(x) and g(x) simply returns F and G.
 */
class ILinearModel : public IModelEKF {
public:
  VectorXd f(const VectorXd& X, const VectorXd& U, double dt) const override {
    F = _update_F(dt);
    G = _update_G(dt);
    return F * X + G * U;
  }
  VectorXd h(const VectorXd& X) const override { return H * X; }

  MatrixXd J_f(const VectorXd& X0) const override { return F; }
  MatrixXd J_h(const VectorXd& X0) const override { return H; }

protected:
  mutable MatrixXd F;  // Transition Matrix
  mutable MatrixXd G;  // Input Matrix
  MatrixXd H;          // Output Matrix

  virtual MatrixXd _update_F(double dt) const = 0;
  virtual MatrixXd _update_G(double dt) const = 0;
};

class LaserModel : public ILinearModel {
public:
  /** This model assumes state X = [x, vx, y, vy] */
  explicit LaserModel() {
    F = MatrixXd::Identity(4, 4);
    G = MatrixXd::Zero(4, 1);
    H = MatrixXd(2, 4);
    H << 1, 0, 0, 0,
         0, 0, 1, 0;
  }

  int Sx() const override { return 4; }
  int Su() const override { return 1; }
  int Sz() const override { return 2; }

protected:
  MatrixXd _update_F(double dt) const override {
    MatrixXd F_new(4, 4);
    F_new << 1, dt, 0, 0,
             0,  1, 0, 0,
             0,  0, 1, dt,
             0,  0, 0, 1;
    return F_new;
  }

  /// G is a zero matrix (not dependent on dt)
  MatrixXd _update_G(double dt) const override { return G; }
};

#endif //KALMAN_FILTERS_CPP_LINEAR_H
