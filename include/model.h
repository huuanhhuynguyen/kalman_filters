#ifndef KALMAN_FILTERS_CPP_MODEL_H
#define KALMAN_FILTERS_CPP_MODEL_H

#include "Eigen/Dense"

using namespace Eigen;

/**
 * Generic (Linear or Non-Linear) Model Interface.
 * A model is mathematically represented as follows:
 *  x1 = f(x) + g(u)
 *  z  = h(x)
 * If the model is linear, then:
 * x1 = F * x + G * u
 * z  = H * x
 * If the model is non-linear, its linearized at a certain x0 is:
 * x1 = J_F * x + J_G * u
 * z  = J_H * x
 * where J_K is Jacobian matrix at x0 of the K matrix, K = F, G, or H.
 */
class IModel {
public:
  virtual MatrixXd F() const = 0;  // Transition Matrix
  virtual MatrixXd G() const = 0;  // Input Matrix
  virtual MatrixXd H() const = 0;  // Output Matrix
  /// Needs to be called before reading the matrices
  virtual void update(double dt, const VectorXd& X0) = 0;
};

class ILinearModel : public IModel {
  /// Linear model is a special case, where no linearization is needed (i.e
  /// X0 is not needed)
  void update(double dt, const VectorXd& X0) override {
    _update(dt);
  }

protected:
  virtual void _update(double dt) = 0;
};

class LaserModel : public ILinearModel {
public:
  explicit LaserModel() {
    F_ = MatrixXd::Identity(4, 4);
    G_ = MatrixXd::Zero(4, 1);
    H_ = MatrixXd(2, 4);
    H_ << 1, 0, 0, 0,
          0, 0, 1, 0;
  }
  MatrixXd F() const override { return F_; }
  MatrixXd G() const override { return G_; }
  MatrixXd H() const override { return H_; }
private:
  MatrixXd F_, G_, H_;

  void _update(double dt) override {
    F_ = MatrixXd(4, 4);
    F_ << 1, dt, 0, 0,
        0,  1, 0, 0,
        0,  0, 1, dt,
        0,  0, 0, 1;
  }
};

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
    F_ = MatrixXd::Identity(4, 4);
    G_ = MatrixXd::Zero(4, 1);
    H_ = MatrixXd::Zero(3, 4);
  }

  MatrixXd F() const override { return F_; }
  MatrixXd G() const override { return G_; }
  MatrixXd H() const override { return H_; }

  void update(double dt, const VectorXd& X0) override {
    _update_transition(dt);
    H_ = _linearize_H(X0);
  }

private:
  MatrixXd F_, G_, H_;

  void _update_transition(double dt) {
    F_ = MatrixXd(4, 4);
    F_ << 1, dt, 0, 0,
          0,  1, 0, 0,
          0,  0, 1, dt,
          0,  0, 0, 1;
  }

  /// Returns jacobian matrix of H at X0
  static MatrixXd _linearize_H(const VectorXd& X0) {
    double x  = X0[0];
    double vx = X0[1];
    double y  = X0[2];
    double vy = X0[3];

    // equation r = sqrt(x*x + y*y)
    double c1 = x*x + y*y;
    double dr_dx = x / sqrt(c1);
    double dr_dy = y / sqrt(c1);
    double dr_dvx = 0;
    double dr_dvy = 0;

    // equation phi = arctan(y/x)
    double dp_dx = -y / c1;
    double dp_dy =  x / c1;
    double dp_dvx = 0;
    double dp_dvy = 0;

    // equation rho_dot = sqrt(vx*vx + vy*vy - x*x - y*y)
    double c2 = vx*vx + vy*vy - x*x - y*y;
    double drd_dx = -x / sqrt(c2);
    double drd_dy = -y / sqrt(c2);
    double drd_dvx = vx / sqrt(c2);
    double drd_dvy = vy / sqrt(c2);

    MatrixXd J_H = MatrixXd(3, 4);
    J_H << dr_dx,  dr_dy,  dr_dvx,  dr_dvy,
           dp_dx,  dp_dy,  dp_dvx,  dp_dvy,
           drd_dx, drd_dy, drd_dvx, drd_dvy;
    return J_H;
  }
};

//TODO save last dt to avoid recalculation
//TODO avoid inline big functions

#endif //KALMAN_FILTERS_CPP_MODEL_H
