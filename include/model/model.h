#ifndef KALMAN_FILTERS_CPP_NL_MODEL_H
#define KALMAN_FILTERS_CPP_NL_MODEL_H

#include "Eigen/Dense"

using namespace Eigen;

/**
 * Generic Model Interface.
 * A model is mathematically represented as follows:
 *   x1 = f(x, u, dt)
 *   z  = h(x)
 * where x = state, u = input, z = measurement
 */
class IModel {
public:
  virtual ~IModel() = default;

  virtual VectorXd f(const VectorXd& X, const VectorXd& U, double dt) const = 0;
  virtual VectorXd h(const VectorXd& X) const = 0;

  /** Sizes of the state, input and measurement vectors. */
  virtual unsigned int Sx() const = 0;
  virtual unsigned int Su() const = 0;
  virtual unsigned int Sz() const = 0;
};

/**
 * A model which is linearizable about X0 should have this interface.
 */
class ILinearizable {
public:
  virtual ~ILinearizable() = default;

  /// Returns linearized (Jacobian) matrix at X0
  virtual MatrixXd J_f(const VectorXd& X0) const = 0;
  virtual MatrixXd J_h(const VectorXd& X0) const = 0;
};

/**
 * All EKF models.
 */
class IModelEKF : public IModel, public ILinearizable {};





#endif //KALMAN_FILTERS_CPP_NL_MODEL_H
