#include "sigma_points.h"
#include <iostream>

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
      throw(std::invalid_argument("Error: diagonal elements of 'cov' must be non-negative"));
    }
  }

  // cov = sqrt( (n + lambda) * cov )
  // Note: square-root of a matrix A is the vector S if A = S * S.transpose()
  {
    const int n = cov.rows();
    const int lambda = 3 - n;
    cov *= (n + lambda);
    SelfAdjointEigenSolver<MatrixXd> estimator(cov);
    const auto sqrt = estimator.operatorSqrt();
    //const MatrixXd sqrt = cov.llt().matrixL();
    if ((cov - sqrt*sqrt.transpose()).norm() > 1e-9) {
      throw(std::logic_error("Error: Either the calculation of square-root of "
                            "covariance matrix 'cov' is wrong or the given "
                            "'cov' has a problem. In the second case, check"
                            "your UKF equations or UKF model pointer member."))
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
  if ((mean - muy).norm() > 1e-9) {
    throw(std::logic_error("Mean of all sigma points must equal 'muy'."));
  }

  return sigma;
}

VectorXd compute_sigma_weights(int n)
{
  VectorXd weights(2*n+1);

  const double lambda = 3 - n;
  weights(0) = lambda / (n + lambda);
  for (int i = 1; i <= 2*n; ++i) {
    weights(i) = 0.5 / (n + lambda);
  }

  // output validation
  const double sum = weights.sum();
  if (abs(sum - 1.0) > 1.0e-9) {
    // Don't throw exception because the function is called in UKF constructor
    std::cerr << "Sigma weights must sum up to 1!" << std::endl;
  }

  return weights;
}