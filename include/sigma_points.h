#ifndef KALMAN_FILTERS_CPP_SIGMA_POINTS_H
#define KALMAN_FILTERS_CPP_SIGMA_POINTS_H

#include <vector>
#include <numeric>
#include "Eigen/Dense"

using namespace Eigen;

/**
 * Compute sigma points from a multivariate Gaussian distribution.
 * @param muy mean vector of the distribution
 * @param cov covariance matrix of the distribution
 * @return sigma points
 *
 * Note: lambda is chosen as 3 - n, where n is size of muy (i.e. system dimension).
 */
MatrixXd compute_sigma_points(const VectorXd& muy, MatrixXd cov);

/**
 * Return weights of sigma points of a n-dimension Gaussian distribution.
 */
std::vector<double> compute_sigma_weights(int n);

#endif //KALMAN_FILTERS_CPP_SIGMA_POINTS_H
