#include "sample.h"

namespace {
void _read_lidar_measurement(std::ifstream& iss, Sample& sample)
{
  sample.sensor = Sample::Sensor::LIDAR;

  float x, y;
  iss >> x;
  iss >> y;
  sample.data = Eigen::VectorXd(2);
  sample.data << x, y;

  iss >> sample.t ;
}

void _read_radar_measurement(std::ifstream& iss, Sample& sample)
{
  sample.sensor = Sample::Sensor::RADAR;

  float rho, phi, rho_dot;
  iss >> rho;
  iss >> phi;
  iss >> rho_dot;
  sample.data = Eigen::VectorXd(3);
  sample.data << rho, phi, rho_dot;

  iss >> sample.t;
}

void _read_gt(std::ifstream& iss, Sample& sample_gt)
{
  float x_gt, y_gt, vx_gt, vy_gt;
  iss >> x_gt;
  iss >> y_gt;
  iss >> vx_gt;
  iss >> vy_gt;
  sample_gt.data = Eigen::VectorXd(4);
  sample_gt.data << x_gt, vx_gt, y_gt, vy_gt;
}
} // unnamed namespace

void read(const std::string& filename,
          std::vector<Sample>& measurement,
          std::vector<Sample>& gt)
{
  using std::ifstream;
  ifstream iss(filename.c_str(), ifstream::in);

  std::string line;
  Sample sample, gt_sample;
  while (std::getline(iss, line))
  {
    std::string sensor;
    iss >> sensor;
    if (sensor == "L")
    {
      _read_lidar_measurement(iss, sample);
      gt_sample.sensor = Sample::Sensor::LIDAR;
    } else {
      _read_radar_measurement(iss, sample);
      gt_sample.sensor = Sample::Sensor::RADAR;
    }
    _read_gt(iss, gt_sample);

    measurement.push_back(sample);
    gt.push_back(gt_sample);
  }
}