#ifndef KALMAN_FILTERS_CPP_FUSION_H
#define KALMAN_FILTERS_CPP_FUSION_H

#include "sample.h"
#include "KF.h"
#include <memory>
#include <iostream>

struct Position {
  double x, y;
  explicit Position(double x_, double y_) : x{x_}, y{y_} {}
};

class Fusion {
public:
  using KFPtr = std::unique_ptr<IKalmanFilter>;

  explicit Fusion(KFPtr pLKF, KFPtr pRKF, VectorXd X0)
    : LaserKF{std::move(pLKF)},
      RadarKF{std::move(pRKF)},
      X{std::move(X0)}
  {
    auto Sx = X.rows();
    P = MatrixXd::Identity(Sx, Sx);
  }

  std::vector<Position> process(const std::vector<Sample>& measurement)
  {
    std::vector<Position> positions;
    VectorXd u(1);
    u << 0;

    for (int i = 1; i < measurement.size(); ++i)
    {
      auto& m = measurement[i];
      double dt = double(measurement[i].t - measurement[i-1].t) / 1.0e6;
      auto z = _get_z(m);

      if (m.sensor == Sample::Sensor::LIDAR) {
        LaserKF->X = X;
        LaserKF->P = P;
        LaserKF->update(z, u, dt);
        X = LaserKF->predict(u, dt);
        P = LaserKF->P;
      } else {
        RadarKF->X = X;
        RadarKF->P = P;
        RadarKF->update(z, u, dt);
        X = RadarKF->predict(u, dt);
        P = RadarKF->P;
      }
      positions.emplace_back(X[0], X[2]);
    }
    return positions;
  }

private:
  KFPtr LaserKF, RadarKF;
  VectorXd X;  // Current State
  MatrixXd P;  // Covariance Matrix
  MatrixXd Q;

  static VectorXd _get_z(const Sample& m) {
    if (m.sensor == Sample::Sensor::LIDAR) {
      auto x = m.data[0];
      auto y = m.data[1];
      VectorXd z(2);
      z << x, y;
      return z;
    } else {
      auto rho = m.data[0];
      auto phi = m.data[1];
      auto rho_dot = m.data[2];
      VectorXd z(3);
      z << rho, phi, rho_dot;
      return z;
    }
  }
};

#endif //KALMAN_FILTERS_CPP_FUSION_H
