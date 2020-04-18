#ifndef KALMAN_FILTERS_CPP_KFFACTORY_H
#define KALMAN_FILTERS_CPP_KFFACTORY_H

#include "EKF.h"
#include <memory>

enum class KFType : uint8_t {
  LINEAR, EXTENDED, UNSCENTED
};

class KFFactory {
public:
  using KFPtr = std::unique_ptr<IKalmanFilter>;
  using ModelPtr = std::unique_ptr<IModel>;

  static KFPtr manufacture(KFType type, ModelPtr pModel) {
    switch (type) {
      case KFType::LINEAR:
      case KFType::EXTENDED:
      case KFType::UNSCENTED:
      default:
        return std::make_unique<LKF>(std::move(pModel));
    }
  }
};

#endif //KALMAN_FILTERS_CPP_KFFACTORY_H
