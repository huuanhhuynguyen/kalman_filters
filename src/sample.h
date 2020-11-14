#ifndef KALMAN_FILTERS_CPP_SAMPLE_H
#define KALMAN_FILTERS_CPP_SAMPLE_H

#include <vector>
#include <fstream>
#include "Eigen/Dense"

using Timestamp = long long;

struct Sample {
  Timestamp t;

  enum class Sensor : uint8_t {
    RADAR,
    LIDAR
  } sensor;

  Eigen::VectorXd data;
};

void read(const std::string& filename,
          std::vector<Sample>& measurement,
          std::vector<Sample>& gt);

#endif //KALMAN_FILTERS_CPP_SAMPLE_H
