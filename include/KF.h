#ifndef SELF_03_KALMAN_FILTERS_CPP_KF_H
#define SELF_03_KALMAN_FILTERS_CPP_KF_H

class IState {
public:
    /// returns the number of elements in the state vector
    virtual void len() const noexcept = 0;
    /// returns the state vector
    virtual void data() const = 0;
};

class IKalmanFilter {
public:
    virtual void update(const IState& measurement, float dt) = 0;
    virtual void predict(float dt) = 0;
};

#endif //SELF_03_KALMAN_FILTERS_CPP_KF_H
