#ifndef KALMAN_FILTERS_CPP_MODEL_H
#define KALMAN_FILTERS_CPP_MODEL_H

#include "Eigen/Dense"

using namespace Eigen;

class IModel {
public:
  MatrixXd F;        // Transition Matrix
  MatrixXd G;        // Input Matrix
  MatrixXd H;        // Output Matrix
  virtual void update(double dt) = 0;
};

class AccelerationModel : public IModel {
public:
  explicit AccelerationModel() {
    G = MatrixXd::Zero(6, 1);
    H = MatrixXd(4, 6);
    H << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0;
    F = MatrixXd::Identity(6, 6);
  }

  void update(double dt) override {
    float dt2 = dt * dt / 2;
    F = MatrixXd(6, 6);
    F << 1, dt, dt2, 0,  0,   0,
         0,  1,  dt, 0,  0,   0,
         0,  0,   1, 0,  0,   0,
         0,  0,   0, 1, dt, dt2,
         0,  0,   0, 0,  1,  dt,
         0,  0,   0, 0,  0,   1;
  }
};

class VelocityModel : public IModel {
public:
  explicit VelocityModel() {
    G = MatrixXd::Zero(4, 1);
    H = MatrixXd(2, 4);
    H << 1, 0, 0, 0,
         0, 0, 1, 0;
    F = MatrixXd::Identity(4, 4);
  }

  void update(double dt) override {
    F = MatrixXd(4, 4);
    F << 1, dt, 0, 0,
         0,  1, 0, 0,
         0,  0, 1, dt,
         0,  0, 0, 1;
  }
};

//TODO save last dt to avoid recalculation

#endif //KALMAN_FILTERS_CPP_MODEL_H
