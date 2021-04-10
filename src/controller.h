//
// Created by ksrdjan on 3/21/2021.
//

#ifndef PATH_PLANNING_CONTROLLER_H
#define PATH_PLANNING_CONTROLLER_H

#include "state_space.h"
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;

namespace controller
{
  class Controller
  {
  public:
    bool InitializeController();
    void regulate_s(double delta_dist, double speed_set_point, bool regulate_speed);
    void regulate_d(double dist);

    // Car model, basically double-integrator
    StateSpace s_model_;
    StateSpace d_model_;

    // previous value to compare for rate change
    double s_u_prev_;
    double d_u_prev_;
    double max_s;
  };
}

#endif //PATH_PLANNING_CONTROLLER_H
