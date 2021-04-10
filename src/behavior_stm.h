//
// Created by srdjan on 4/4/21.
//

#ifndef PATH_PLANNING_BEHAVOIRSTM_H
#define PATH_PLANNING_BEHAVOIRSTM_H

#include "controller.h"
#include <vector>

class BehaviorStm
{
  enum class States
  {
    Follow,
    Keep_speed
  };

  using CarData = std::vector<double>;
  using CarDataIt = std::vector<CarData>::const_iterator;

public:
  BehaviorStm(double max_s, double offset);

  void Cycle(const std::vector<CarData> &sensor_fusion, bool predict = true);

  double GetS();
  double GetD();
  void SetModel(double s, double s_dot, double d, double target_d);

private:
  void find_closest_cars_from_lane(const std::vector<CarData> &sensor_fusion, int lane_number, CarDataIt &closest_behind,
                                   CarDataIt &closest_ahead, double &min_dist_behind, double &min_dist_ahead);

  double lane_score(CarDataIt closest_behind, CarDataIt closest_ahead, CarDataIt end);
  double calculate_car_speed(const CarData &car_data);
  double calculate_distance(double s1, double s2);
  double wrap_s(double s);

  controller::Controller ctrl_;
  States current_state_;
  double target_d_;
  double target_position_;
  double target_speed_;
  double max_s_;

  double offset_;
  double first_lane_d_;
  double second_lane_d_;
  double third_lane_d_;
};


#endif //PATH_PLANNING_BEHAVOIRSTM_H
