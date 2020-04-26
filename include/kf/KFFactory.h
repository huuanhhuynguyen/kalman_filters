#ifndef KALMAN_FILTERS_CPP_KFFACTORY_H
#define KALMAN_FILTERS_CPP_KFFACTORY_H

#include "EKF.h"
#include <memory>

enum class KFType : uint8_t {
  EXTENDED, UNSCENTED,
};

class KFFactory {
public:
  using KFPtr = std::unique_ptr<IKalmanFilter>;
  using ModelPtr = std::unique_ptr<IModel>;
  using EKF_MPtr = std::unique_ptr<IModelEKF>;

  static KFPtr manufacture(KFType type, EKF_MPtr pM) {
    switch (type) {
      case KFType::UNSCENTED:
      case KFType::EXTENDED:
      {
        //EKF_MPtr pModelEKF(dynamic_cast<IModelEKF*>(pM.get()));
        //return std::make_unique<EKF>(std::move(pModelEKF));
        return std::make_unique<EKF>(std::move(pM));
      }
    }
  }
};

#endif //KALMAN_FILTERS_CPP_KFFACTORY_H
