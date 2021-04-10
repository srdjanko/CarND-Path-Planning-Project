//
// Created by srdjan on 3/25/21.
//

#ifndef PATH_PLANNING_STATE_SPACE_H
#define PATH_PLANNING_STATE_SPACE_H

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace controller
{
  double ApplyRateChange(double du, double max_change_rate);
  double ApplySaturation(double u, const double saturation_min, const double saturation_max);

  // Quick and simple implementation of the discrete state space model.
  class StateSpace
  {
  public:

    StateSpace();

    void Initialize(const MatrixXd &A, const MatrixXd &B, const MatrixXd &C, const MatrixXd &D = MatrixXd());

    const VectorXd& CalculateOutput(const VectorXd &u);

    const void UpdateStates(const VectorXd &u);

    static void C2D_Euler(const MatrixXd &Ac, const MatrixXd &Bc, MatrixXd &Ad, MatrixXd &Bd, double Ts);

//  private:
    MatrixXd A_;
    MatrixXd B_;
    MatrixXd C_;
    MatrixXd D_;
    VectorXd x_;
    VectorXd y_;
  };

}

#endif //PATH_PLANNING_STATE_SPACE_H
