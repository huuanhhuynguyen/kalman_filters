#ifndef KALMAN_FILTERS_CPP_KFFACTORY_H
#define KALMAN_FILTERS_CPP_KFFACTORY_H

#include "EKF.h"
#include "UKF.h"
#include <memory>

class KFFactory {
public:
  using KFPtr = std::unique_ptr<IKalmanFilter>;

  static KFPtr manufacture_ekf(std::unique_ptr<ILinearizableModel> pM,
                               const MatrixXd& Q_in,
                               const MatrixXd& R_in) {
    return std::make_unique<EKF>(std::move(pM), Q_in, R_in);
  }

  static KFPtr manufacture_ukf(std::unique_ptr<IModel> pM,
                               const MatrixXd& Q_in,
                               const MatrixXd& R_in) {
    return std::make_unique<UKF>(std::move(pM), Q_in, R_in);
  }
};

#endif //KALMAN_FILTERS_CPP_KFFACTORY_H
