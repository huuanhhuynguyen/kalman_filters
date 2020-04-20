#include "nl_model.h"

MatrixXd RadarModel::_linearize_H(const VectorXd &X0)
{
  double x  = X0[0];
  double vx = X0[1];
  double y  = X0[2];
  double vy = X0[3];

  // equation r = sqrt(x*x + y*y)
  double c1 = x*x + y*y;
  double dr_dx = x / sqrt(c1);
  double dr_dy = y / sqrt(c1);
  double dr_dvx = 0;
  double dr_dvy = 0;

  // equation phi = arctan(y/x)
  double dp_dx = -y / c1;
  double dp_dy =  x / c1;
  double dp_dvx = 0;
  double dp_dvy = 0;

  // equation rho_dot = (x * vx + y * vy) / sqrt(x*x + y*y)
  double c2 = c1 * sqrt(c1);
  double drd_dx = y * (vx*y - vy*x) / c2;
  double drd_dy = x * (vy*x - vx*y) / c2;
  double drd_dvx = x / sqrt(c1);
  double drd_dvy = y / sqrt(c1);

  MatrixXd J_H(3, 4);
  J_H << dr_dx,  dr_dvx,  dr_dy,  dr_dvy,
         dp_dx,  dp_dvx,  dp_dy,  dp_dvy,
        drd_dx, drd_dvx, drd_dy, drd_dvy;
  return J_H;
}

//TODO avoid zero division