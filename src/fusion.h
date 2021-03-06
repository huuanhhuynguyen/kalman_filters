#ifndef KALMAN_FILTERS_CPP_FUSION_H
#define KALMAN_FILTERS_CPP_FUSION_H

#include "sample.h"
#include "kf/KF.h"
#include <memory>
#include <iostream>
#include <utility>

struct Position {
  double x, y;
  explicit Position(double x_, double y_) : x{x_}, y{y_} {}
};

class Fusion {
public:
  using KF = IKalmanFilter;

  explicit Fusion(KF& pLKF, KF& pRKF, VectorXd  X0)
    : LaserKF{pLKF}, RadarKF{pRKF}, X{std::move(X0)}
  {
    P = MatrixXd::Zero(X.size(), X.size());
    MatrixXd Pxx = X * X.transpose();
    for (int i = 0; i < X.size(); ++i) {
      P(i, i) = Pxx(i, i);
    }
  }

  std::vector<Position> process(const std::vector<Sample>& measurement)
  {
    std::vector<Position> positions;
    VectorXd u(1);
    u << 0;

    for (int i = 1; i < measurement.size(); ++i)
    {
      const auto& m = measurement[i];
      double dt = double(measurement[i].t - measurement[i-1].t) / 1.0e6;
      const auto& z = m.data;

      if (m.sensor == Sample::Sensor::LIDAR) {
        LaserKF.X = X;
        LaserKF.P = P;
        LaserKF.update(z, u);
        X = LaserKF.predict(u, dt);
        P = LaserKF.P;
      } else {
        RadarKF.X = X;
        RadarKF.P = P;
        RadarKF.update(z, u);
        X = RadarKF.predict(u, dt);
        P = RadarKF.P;
      }
      positions.emplace_back(X[0], X[2]);
    }
    return positions;
  }

private:
  IKalmanFilter& LaserKF;
  IKalmanFilter& RadarKF;
  VectorXd X;  // Current State
  MatrixXd P;  // Covariance Matrix
};

#endif //KALMAN_FILTERS_CPP_FUSION_H
