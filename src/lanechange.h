#ifndef LANECHANGE_H
#define LANECHANGE_H

using std::map;
using std::string;
using std::vector;

float calculate_cost();

int lane_change(vector<vector<double>> &sensor_fusion, double car_s, double car_d, double car_speed, double ref_vel, int lane, double prev_size, double sample_time, bool too_close1);

#endif  // LANECHANGE_H