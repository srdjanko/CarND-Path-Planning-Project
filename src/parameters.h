//
// Created by srdjan on 4/4/21.
//

#ifndef PATH_PLANNING_PARAMETERS_H
#define PATH_PLANNING_PARAMETERS_H

#define A_limit                     10.0
#define J_limit                     10.0
#define Max_speed                   20
#define Ts                          0.02

#define Acc_limit_max               A_limit * 0.6
#define Jerk_limit_max              J_limit * 0.6
#define Speed_limit_max             Max_speed
#define Acc_limit_d_max             A_limit * 0.5
#define Jerk_limit_d_max            J_limit * 0.3
#define Speed_limit_max             Max_speed
#define Speed_limit_d_max           8
#define Time_difference             4
#define Car_length                  5
#define Lane_length                 4
#define Min_ahead_distance          8
#define Start_following_distance    20
#define Following_distance          15
#define Fall_behind_distance        30
#define Lane_change_speed_threshold 0.95 * Speed_limit_max
#define Lookahead_limit             60

#endif //PATH_PLANNING_PARAMETERS_H
