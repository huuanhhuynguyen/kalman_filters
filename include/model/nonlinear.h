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
class RadarModel : public IModel {
public:
  explicit RadarModel() {
    F = MatrixXd::Identity(4, 4);
    G = MatrixXd::Zero(4, 1);
    H = MatrixXd::Zero(3, 4);
  }

  VectorXd f(const VectorXd& X, const VectorXd& U) const override { return F * X + G * U; }
  VectorXd h(const VectorXd& X) const override {
    double x  = X[0];
    double vx = X[1];
    double y  = X[2];
    double vy = X[3];
    double rho = sqrt(x*x + y*y);
    double phi = atan2(y, x);
    double rho_dot = (x * vx + y * vy) / std::max(rho, 1e-9);
    VectorXd z(3);
    z << rho, phi, rho_dot;
    return z;
  }

protected:
  void _update_dt(double dt) override {
    F = MatrixXd(4, 4);
    F << 1, dt, 0, 0,
         0,  1, 0, 0,
         0,  0, 1, dt,
         0,  0, 0, 1;
  }

  MatrixXd _linearize_F(const VectorXd& X0) override { return F; }
  MatrixXd _linearize_G(const VectorXd& X0) override { return G; }

  /** Returns Jacobian matrix of H at X0 */
  MatrixXd _linearize_H(const VectorXd& X0) override;

};

#endif //KALMAN_FILTERS_CPP_NL_MODEL_H
