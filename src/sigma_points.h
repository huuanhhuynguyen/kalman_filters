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
 */
MatrixXd compute_sigma_points(const VectorXd& muy, MatrixXd cov);

/**
 * Return weights of sigma points of a n-dimension Gaussian distribution.
 */
void compute_sigma_weights(int n, VectorXd& weights_m, VectorXd& weights_c);

#endif //KALMAN_FILTERS_CPP_SIGMA_POINTS_H
