#ifndef KALMAN_FILTERS_CPP_NL_MODEL_H
#define KALMAN_FILTERS_CPP_NL_MODEL_H

#include "Eigen/Dense"

using namespace Eigen;

/**
 * Generic (Linear or Non-Linear) Model Interface.
 * A model is mathematically represented as follows:
 *   x1 = f(x) + g(u)
 *   z  = h(x)
 * If the model is linear, then:
 *   x1 = F * x + G * u
 *   z  = H * x
 * If the model is non-linear, its linearized at a certain x0 is:
 *   x1 ~= f(x) + g(u)
 *   z  = J_h * x
 * where J_H is Jacobian matrix at x0 of the H matrix.
 *
 * J_f ana J_g are also needed for update Kalman Filter covariance matrix P.
 */
class IModel {
public:
  virtual ~IModel() = default;

  virtual VectorXd f(const VectorXd& X) const = 0;
  virtual VectorXd g(const VectorXd& U) const = 0;
  virtual VectorXd h(const VectorXd& X) const = 0;

  MatrixXd J_f() const { return F; }
  MatrixXd J_g() const { return G; }
  MatrixXd J_h() const { return H; }  // Output Matrix

  /** Needs to be called before reading the matrices */
  void update(double dt, const VectorXd& X0) {
    _update_dt(dt);
    F = _linearize_F(X0);
    G = _linearize_G(X0);
    H = _linearize_H(X0);
  }

protected:
  MatrixXd F;  // Transition Matrix
  MatrixXd G;  // Input Matrix
  MatrixXd H;  // Output Matrix

  /** Use dt to update member matrices */
  virtual void _update_dt(double dt) = 0;

  /** Linearize f() about X0 */
  virtual MatrixXd _linearize_F(const VectorXd& X0) = 0;

  /** Linearize g() about X0 */
  virtual MatrixXd _linearize_G(const VectorXd& X0) = 0;

  /** Linearize h() about X0 */
  virtual MatrixXd _linearize_H(const VectorXd& X0) = 0;
};

#endif //KALMAN_FILTERS_CPP_NL_MODEL_H
