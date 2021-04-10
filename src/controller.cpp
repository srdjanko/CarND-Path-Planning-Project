//
// Created by ksrdjan on 3/21/2021.
//

#include "controller.h"
#include "parameters.h"

namespace controller
{
  namespace
  {
    // Calculated from the model and characteristic polynomial
    constexpr double kv = 2.7273;
    constexpr double ks = 0.606;

    constexpr double Td = 1;
    constexpr double kdv = 2 / Td;
    constexpr double kds = 1/(Td*Td);
  }

  bool Controller::InitializeController()
  {
    // The controller is state-space regulator of the double integrator process
    // since s_2_dot = u, where u is the acceleration input. By using the rate-change
    // and saturation filtering, we can exactly control the amount of jerk and acceleration
    // to be in the given range. The controller for d coordinate is similar with somewhat
    // different limit values (see parameters.h).

    d_u_prev_ = 0;
    s_u_prev_ = 0;

    // Car model, double integrator
    MatrixXd A(2, 2);
    A << 0, 1,
         0, 0;

    MatrixXd B(2, 1);
    B << 0,
         1;

    StateSpace::C2D_Euler(A, B, A, B, Ts);
    auto C = MatrixXd::Identity(2, 2);

    // S and D model are identical, simple second order mechanical systems
    // (double integrator with acceleration as input)
    s_model_.Initialize(A, B, C);
    d_model_.Initialize(A, B, C);
  }

  // Regulator for s can accept just the speed, or both the speed and position setpoints.
  // Using both setpoints (when regulating s) ensures we will achieve given position even
  // if it is moving with constant speed.
  void Controller::regulate_s(double delta_dist, double speed_set_point, bool regulate_speed)
  {
    VectorXd u(1);
    u << delta_dist * ks * (regulate_speed ? 0 : 1) + speed_set_point;

    // Apply speed limit saturation
    u << ApplySaturation(u[0], 0, Speed_limit_max);

    // Calculate speed feedback control
    u << (u[0] - s_model_.x_[1]) * kv;

    // Apply rate and saturation limits to speed feedback -> jerk limits
    auto du = u[0] - s_u_prev_;
    du = ApplyRateChange(du, Jerk_limit_max * Ts);
    u << u[0] + du;

    // Apply saturation for acceleration
    u << ApplySaturation(u[0], -Acc_limit_max, Acc_limit_max);

    // Update states
    s_u_prev_ = u[0];
    s_model_.UpdateStates(u);
  }

  // Regulator for d just regulates position
  void Controller::regulate_d(double dist)
  {
    VectorXd u(1);
    u << (dist - d_model_.x_[0]) * kds;

    // Apply speed limit saturation
    u << ApplySaturation(u[0], -Speed_limit_d_max, Speed_limit_d_max);

    // Calculate speed feedback control
    u << (u[0] - d_model_.x_[1]) * kdv;

    // Apply rate and saturation limits to speed feedback -> jerk limits
    auto du = u[0] - d_u_prev_;
    du = ApplyRateChange(du, Jerk_limit_d_max * Ts);
    u << u[0] + du;

    // Apply saturation for acceleration
    u << ApplySaturation(u[0], -Acc_limit_d_max, Acc_limit_d_max);

    // Update states
    d_u_prev_ = u[0];
    d_model_.UpdateStates(u);
  }
}
