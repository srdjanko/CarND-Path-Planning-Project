//
// Created by srdjan on 4/4/21.
//

#include "behavior_stm.h"
#include "parameters.h"

namespace
{
  constexpr double max_value = std::numeric_limits<double>::max();
}

BehaviorStm::BehaviorStm(double max_s, double offset) : current_state_(States::Keep_speed), target_d_(0), max_s_(max_s), offset_(offset)
{
  first_lane_d_ = Lane_length / 2 - offset;
  second_lane_d_ = first_lane_d_ + Lane_length;
  third_lane_d_ = second_lane_d_ + Lane_length;

  ctrl_.InitializeController();
}

void BehaviorStm::Cycle(const std::vector<CarData> &sensor_fusion, bool predict)
{
  auto sensor_fusion_end = sensor_fusion.end();

  switch (current_state_)
  {
    case States::Follow:
    {
      // Regulate according to previous plan
      ctrl_.regulate_d(target_d_ == third_lane_d_ ? third_lane_d_ - 0.2 : target_d_);
      target_position_ = wrap_s(target_position_ + target_speed_ * 0.02);
      auto distance = calculate_distance(target_position_, ctrl_.s_model_.x_[0]);

      ctrl_.regulate_s(distance, target_speed_, false);
      ctrl_.s_model_.x_[0] = wrap_s(ctrl_.s_model_.x_[0]);

      if (!predict)
      {
        CarDataIt closest_behind, closest_ahead;
        double min_dist_behind, min_dist_ahead;
        find_closest_cars_from_lane(sensor_fusion, target_d_, closest_behind, closest_ahead,
                                    min_dist_behind, min_dist_ahead);

        if (min_dist_ahead > Fall_behind_distance)
        {
          // If we are falling behind the lead car, change state to Keep_speed
          current_state_ = States::Keep_speed;
        }
        else if (closest_ahead != sensor_fusion_end)
        {
          // Check leading car
          const auto closest_ahead_speed = calculate_car_speed(*closest_ahead);
          target_position_ = wrap_s((*closest_ahead)[5] - Following_distance);
          target_speed_ = closest_ahead_speed;

          if (closest_ahead_speed < Lane_change_speed_threshold)
          {
            // We only change lane if the leading car is too slow

            // Try lane change maneuver
            double left_lane_score = 0, right_lane_score = 0;
            CarDataIt closest_ahead_left, closest_ahead_right;

            if (target_d_ != first_lane_d_)
            {
              find_closest_cars_from_lane(sensor_fusion, target_d_ - Lane_length, closest_behind, closest_ahead_left,
                                          min_dist_behind, min_dist_ahead);

              left_lane_score = lane_score(closest_behind, closest_ahead_left, sensor_fusion_end);

              if (left_lane_score < closest_ahead_speed)
              {
                left_lane_score = 0;
              }
            }

            if (target_d_ != third_lane_d_)
            {
              find_closest_cars_from_lane(sensor_fusion, target_d_ + Lane_length, closest_behind, closest_ahead_right,
                                          min_dist_behind, min_dist_ahead);

              right_lane_score = lane_score(closest_behind, closest_ahead_right, sensor_fusion_end);

              if (right_lane_score < closest_ahead_speed)
              {
                right_lane_score = 0;
              }
            }

            double best_score = 0;
            double target_overtaking_d = 0;
            CarDataIt best_target;

            // Pick the better score of the neighbouring lanes
            if (right_lane_score > left_lane_score)
            {
              best_target = closest_ahead_right;
              best_score = right_lane_score;
              target_overtaking_d = target_d_ + Lane_length;
            }
            else
            {
              best_target = closest_ahead_left;
              best_score = left_lane_score;
              target_overtaking_d = target_d_ - Lane_length;
            }

            if (best_score > 0)
            {
              target_d_ = target_overtaking_d;

              if (best_score == max_value)
              {
                // If we have max score there is no car in front so we can just
                // maintain the speed
                current_state_ = States::Keep_speed;
              }
              else
              {
                // Follow the car in the chosen lane
                target_position_ = wrap_s((*best_target)[5] - Following_distance);
                target_speed_ = calculate_car_speed(*best_target);
              }
            }
          }
        }
      }
    }
      break;

    case States::Keep_speed:
    {
      ctrl_.regulate_d(target_d_ == third_lane_d_ ? third_lane_d_ - 0.2 : target_d_);
      ctrl_.regulate_s(0, Speed_limit_max, true);

      ctrl_.s_model_.x_[0] = wrap_s(ctrl_.s_model_.x_[0]);

      if (!predict)
      {
        CarDataIt closest_ahead, closest_behind;
        double min_dist_behind, min_dist_ahead;
        find_closest_cars_from_lane(sensor_fusion, target_d_, closest_behind, closest_ahead, min_dist_behind,
                                    min_dist_ahead);

        if (closest_ahead != sensor_fusion.end() and min_dist_ahead < Start_following_distance)
        {
          current_state_ = States::Follow;
          target_position_ = wrap_s((*closest_ahead)[5] - Following_distance);
          target_speed_ = calculate_car_speed(*closest_ahead);
        }
      }
    }
      break;
  }
}

double BehaviorStm::GetS()
{
  return ctrl_.s_model_.x_[0];
}

double BehaviorStm::GetD()
{
  return ctrl_.d_model_.x_[0];
}

void BehaviorStm::SetModel(double s, double s_dot, double d, double target_d)
{
  ctrl_.s_model_.x_ << s, s_dot;
  ctrl_.d_model_.x_ << d, 0;
  target_d_ = target_d;
}

void BehaviorStm::find_closest_cars_from_lane(const std::vector<CarData> &sensor_fusion, int target_d,
                                              CarDataIt &closest_behind,
                                              CarDataIt &closest_ahead, double &min_dist_behind, double &min_dist_ahead)
{
  // Check if there is a leading vehicle in front
  auto same_lane = [&](const CarData &car_data)
  {
    return (car_data[6] > target_d - Lane_length / 2.0 + offset_) and (car_data[6] < target_d + Lane_length / 2.0 + offset_);
  };

  auto i = sensor_fusion.begin(), end = sensor_fusion.end();
  // Find the closest cars in the lane
  min_dist_ahead = max_value;
  min_dist_behind = max_value;
  closest_ahead = end;
  closest_behind = end;

  // Iterate all the cars in the that are in given lane, to determine the
  // closest from the behind and in front.
  while (i != end)
  {
    i = std::find_if(i, end, same_lane);
    if (i != end)
    {
      auto dist = calculate_distance((*i)[5], ctrl_.s_model_.x_[0]);

      if (dist > 0 and dist < min_dist_ahead)
      {
        min_dist_ahead = dist;
        closest_ahead = i;
      }
      else if (dist < 0 and -dist < min_dist_behind)
      {
        min_dist_behind = -dist;
        closest_behind = i;
      }

      ++i;
    }
  }
}

double BehaviorStm::lane_score(CarDataIt closest_behind, CarDataIt closest_ahead, CarDataIt end)
{
  double behind_limit, ahead_limit;

  // This method is meant for scoring candidate lanes during lane change maneuver.
  // The higher the score the better. Scoring is provided by looking the closest
  // car behind our position, and the closest car ahead. The final score will be
  // the worse of the two.

  auto our_current_s = ctrl_.s_model_.x_[0];
  auto our_current_s_dot = ctrl_.s_model_.x_[1];

  if (closest_behind == end)
  {
    // For there are no cars behind, we set maximum score.
    behind_limit = max_value;
  }
  else
  {
    // Check if we have enough "time difference" from car in behind
    auto car_speed = calculate_car_speed(*closest_behind);
    auto car_distance = calculate_distance(our_current_s, (*closest_behind)[5]);

    // Modify car distance to actual distance, taking car length into account
    car_distance -= Car_length;

    // We assume here that vehicle behind will at least not attempt to accelerate if it detect we are
    // trying to merge in front.
    auto speed_diff = car_speed - our_current_s_dot;
    double time_diff;

    if(car_distance <= 0)
    {
      time_diff = 0;
    }
    else if(speed_diff <= 0)
    {
      time_diff = max_value;
    }
    else
    {
      time_diff = car_distance / speed_diff;
    }

    // If we have enough of "time difference" from the car behind taking into account
    // our relative speeds and positions, then we set max_value, else we set 0.
    behind_limit = time_diff >= Time_difference ? max_value : 0;
  }

  if (closest_ahead == end or calculate_distance((*closest_ahead)[5], our_current_s) > Lookahead_limit)
  {
    // If there are no cars infront or the car is further away than Lookahead_limit
    // we set maximum score.
    ahead_limit = max_value;
  }
  else
  {
    auto car_speed = calculate_car_speed(*closest_ahead);
    auto car_distance = calculate_distance((*closest_ahead)[5], ctrl_.s_model_.x_[0]);

    // If the car if distant enough then set the cars speed as the score, else set 0.
    ahead_limit = car_distance < Min_ahead_distance ? 0 : car_speed;
  }

  // As a return value we can have:
  // 0 -> no overtaking, (>0) -> we have car ahead with some speed, (max) -> no car ahead, can overtake
  return std::min(behind_limit, ahead_limit);
}

double BehaviorStm::calculate_car_speed(const CarData &car_data)
{
  return std::sqrt(std::pow(car_data[3], 2) + std::pow(car_data[4], 2));
}

double BehaviorStm::calculate_distance(double s1, double s2)
{
  auto distance = std::abs(s1 - s2);

  if(distance > max_s_ / 2.0)
  {
    distance = s1 > s2 ? distance - max_s_ : max_s_ - distance;
  }
  else
  {
    distance = s1 - s2;
  }

  return distance;
}

// Wrap the s to the to the range [0, max_s]
double BehaviorStm::wrap_s(double s)
{
  bool s_is_negative = s < 0;
  s = s_is_negative ? -s : s;

  while(s > max_s_)
  {
    s -= max_s_;
  }

  if(s_is_negative)
  {
    s = max_s_ - s;
  }

  return s;
}
