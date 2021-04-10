//
// Created by srdjan on 3/25/21.
//

#include "state_space.h"
#include "iostream"

namespace controller
{
  StateSpace::StateSpace()
  {}

  void StateSpace::Initialize(const MatrixXd &A, const MatrixXd &B, const MatrixXd &C, const MatrixXd &D)
  {
    A_ = A;
    B_ = B;
    C_ = C;

    if (D.rows() == 0)
    {
      D_ = MatrixXd::Zero(C.rows(), B.cols());
    }
    else
    {
      D_ = D;
    }

    x_ = VectorXd::Zero(A_.rows());
    y_ = VectorXd::Zero(C_.rows());
  }

  const VectorXd &StateSpace::CalculateOutput(const VectorXd &u)
  {
    y_ = C_ * x_ + D_ * u;
    return y_;
  }

  const void StateSpace::UpdateStates(const VectorXd &u)
  {
    x_ = A_ * x_ + B_ * u;
  }

  void StateSpace::C2D_Euler(const MatrixXd &Ac, const MatrixXd &Bc, MatrixXd &Ad, MatrixXd &Bd, double Ts)
  {
    Ad = MatrixXd::Identity(Ac.rows(), Ac.rows()) + Ts * Ac;
    Bd = Ts * Bc;
  }

  double ApplyRateChange(double du, double max_change_rate)
  {
    if (du > max_change_rate)
    {
      du = max_change_rate;
    }
    else if (du < -max_change_rate)
    {
      du = -max_change_rate;
    }

    return du;
  }

  double ApplySaturation(double u, const double saturation_min, const double saturation_max)
  {
    if (u > saturation_max)
    {
      u = saturation_max;
    }
    else if (u < saturation_min)
    {
      u = saturation_min;
    }

    return u;
  }
}
