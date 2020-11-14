#ifndef KALMAN_FILTERS_CPP_NL_MODELS_H
#define KALMAN_FILTERS_CPP_NL_MODELS_H

#include "model.h"

/**
 * Radar model is non-linear due to measurement equations:
 * state X = [x, vx, y, vy]
 * measurement Z = [rho, phi, rho_dot]
 *   x = rho * cos(phi)
 *   y = rho * sin(phi)
 *   vx = dx/dt = rho_dot * cos(phi) - rho * sin(phi)
 *   vy = dy/dt = rho_dot * sin(phi) + rho * cos(phi)
 */
class RadarModel : public IModelEKF {
public:
  explicit RadarModel() {
    F = MatrixXd::Identity(4, 4);
    G = MatrixXd::Zero(4, 1);
  }

  VectorXd f(const VectorXd& X, const VectorXd& U, double dt) const override {
    F = _update_F(dt);
    return F * X + G * U;
  }

  VectorXd h(const VectorXd& X) const override;

  int Sx() const override { return 4; }
  int Su() const override { return 1; }
  int Sz() const override { return 3; }

  MatrixXd J_f(const VectorXd& X0) const override { return F; }

  /** Returns Jacobian matrix of h() at X0 */
  MatrixXd J_h(const VectorXd& X0) const override;

protected:
  mutable MatrixXd F;  // Transition Matrix
  mutable MatrixXd G;  // Input Matrix

  static MatrixXd _update_F(double dt) {
    MatrixXd F_new(4, 4);
    F_new << 1, dt, 0, 0,
             0,  1, 0, 0,
             0,  0, 1, dt,
             0,  0, 0, 1;
    return F_new;
  }
};

#endif //KALMAN_FILTERS_CPP_NL_MODEL_H
