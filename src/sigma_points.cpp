#include "sigma_points.h"

MatrixXd compute_sigma_points(const VectorXd& muy, MatrixXd cov)
{
  if (muy.size() != cov.rows()) {
    throw(std::invalid_argument("muy and cov must have the same number of rows."));
  }

  // cov = sqrt( (n + lambda) * cov )
  // Note that: square-root of matrix A is vector S if A = S * S.transpose()
  {
    const int n = cov.rows();
    const int lambda = 3 - n;
    cov *= (n + lambda);
    SelfAdjointEigenSolver<MatrixXd> estimator(cov);
    cov = estimator.operatorSqrt();
  }

  // Compute sigma points
  const int n = cov.rows();
  MatrixXd sigma(n, 2*n+1);
  sigma.col(0) = muy;
  for (unsigned int i = 1; i <= n; ++i) {
    sigma.col(i) = muy + cov.col(i-1);
  }
  for (unsigned int i = n+1; i <= 2*n; ++i) {
    sigma.col(i) = muy - cov.col(i-n-1);
  }

  return sigma;
}

std::vector<double> compute_sigma_weights(int n)
{
  std::vector<double> weights;
  weights.reserve(2*n + 1);

  const double lambda = 3 - n;
  weights.push_back(lambda / (n + lambda));
  for (unsigned int i = 0; i <= 2*n; ++i) {
    double val = 0.5 / (n + lambda);
    weights.push_back(val);
  }

  // output validation
  double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
  if (abs(sum - 1.0) < 1.0e-9) {
    throw(std::logic_error("Sigma weights must sum up to 1!"));
  }

  return weights;
}