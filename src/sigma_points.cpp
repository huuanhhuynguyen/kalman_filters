#include "sigma_points.h"
#include <iostream>

namespace {
  // configuration parameters for unscented transformation
  int _kappa = 100;
  int _beta = 2;
  double _alpha = 1e-3;
}  // unamed namespace

MatrixXd compute_sigma_points(const VectorXd& muy, MatrixXd cov)
{
  if (cov.rows() != cov.cols()) {
    throw(std::invalid_argument("Error: 'cov' must be a square matrix."));
  }

  if (muy.size() != cov.rows()) {
    throw(std::invalid_argument("Error: 'muy' and 'cov' must have the same number of rows."));
  }

  for (int i = 0; i < cov.rows(); ++i) {
    if (cov(i, i) <= 0.0) {
      std::cerr << "Error: diagonal elements of 'cov' must be non-negative" << std::endl;
    }
  }

  // cov = sqrt( (n + lambda) * cov )
  // Note: square-root of a matrix A is the vector S if A = S * S.transpose()
  {
    const int n = cov.rows();
    const double lambda = (_alpha * _alpha) * (n + _kappa) - n;
    cov *= (n + lambda);
    const MatrixXd sqrt = cov.llt().matrixL();
    if ((cov - sqrt*sqrt.transpose()).norm() > 1e-6) {
      std::cerr << "Error: Estimation of square root of covariance matrix 'cov'"
                   "has a large error." << std::endl;
    }
    cov = sqrt;
  }

  // Compute sigma points
  const int n = cov.rows();
  MatrixXd sigma(n, 2*n+1);
  sigma.col(0) = muy;
  for (int i = 1; i <= n; ++i) {
    sigma.col(i) = muy + cov.col(i-1);
  }
  for (int i = n+1; i <= 2*n; ++i) {
    sigma.col(i) = muy - cov.col(i-n-1);
  }

  // output validation
  VectorXd mean = VectorXd::Zero(n);
  for (int i = 0; i <= 2*n; ++i) {
    mean += sigma.col(i);
  }
  mean /= (2 * n + 1);
  if ((mean - muy).norm() > 1e-6) {
    std::cerr << "Mean of all sigma points must equal 'muy'." << std::endl;
  }

  return sigma;
}

void compute_sigma_weights(int n, VectorXd& weights_m, VectorXd& weights_c)
{
  if (n <= 0) {
    throw(std::invalid_argument("'n' must be a positive number."));
  }

  const double lambda = (_alpha * _alpha) * (n + _kappa) - n;
  weights_m(0) = lambda / (n + lambda);
  for (int i = 1; i <= 2*n; ++i) {
    weights_m(i) = 0.5 / (n + lambda);
  }

  // output validation
  const double sum = weights_m.sum();
  if (abs(sum - 1.0) > 1.0e-9) {
    // Don't throw exception because the function is called in UKF constructor
    std::cerr << "Sigma weights must sum up to 1!" << std::endl;
  }

  weights_c = weights_m;
  weights_c(0) += (1 - _alpha*_alpha + _beta);
}