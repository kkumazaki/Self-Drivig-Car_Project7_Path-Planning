#include "lanechange.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>

using std::string;
using std::vector;

int lane_change(vector<vector<double>> &sensor_fusion, double car_s, double car_d, double car_speed, double ref_vel, int lane, double prev_size, double sample_time, bool too_close1){
  // Minimum velocity for lane change
  double lc_vel = 20; //(mph)

  // Transfer (mph) to (m/s)
  double mph2ms = 3600.0/1609.0;

  vector<int> cnt = {0, 0, 0};
  vector<double> average_vel = {0.0, 0.0, 0.0};
  vector<double> lane_cost = {0.0, 1.0, 2.0};

  vector<double> traffic_flow = {49.5, 45.0, 40.0};


  double weight_density = 5.0;    
  double weight_velofity = 10.0;
  double weight_deceleration = 10.0;
  double weight_lanes = 3.0;

  double total_cost = 0.0;

  for (int i = 0; i < sensor_fusion.size(); i++){
    float d = sensor_fusion[i][6];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt(vx*vx + vy*vy);
    double check_car_s = sensor_fusion[i][5];

    if ((check_car_s > car_s) && (check_car_s - car_s < 90.0 ) && (d < 4.0)){ 
      cnt[0] += 1;
      average_vel[0] += check_speed;
    } else if ((check_car_s > car_s) && (check_car_s - car_s < 90.0 ) && (d < 8.0)){
      cnt[1] += 1;
      average_vel[1] += check_speed;
    } else if ((check_car_s > car_s) && (check_car_s - car_s < 90.0 ) && (d < 12.0)){
      cnt[2] += 1;
      average_vel[2] += check_speed;
    }

  }

  // calculate average traffic velocity at i=0: left, i=1: center, i=2: right
  for (int i = 0; i < 3; i++){
    if (cnt[i]>0){
      average_vel[i] = average_vel[i]/cnt[i];
    } else{
      average_vel[i] = traffic_flow[i] / mph2ms;
    }
  }

  std::cout << "cnt_left: " << cnt[0]  << " cnt_center: " << cnt[1]  << " cnt_right: " << cnt[2]  << std::endl;
  std::cout << "average_vel_left: " << average_vel[0]  << " average_vel_center: " << average_vel[1]  << " average_vel_right: " << average_vel[2]  << std::endl;
  //std::cout << "car_speed: " << car_speed << " ref_vel: " << ref_vel << std::endl;

  double total_cnt = cnt[0] + cnt[1] + cnt[2];

  // [Cost Function]
  for (int i = 0; i < 3; i++){
    // [Cost Function 1. Set the weighted cost on each lane]
    lane_cost[i] = lane_cost[i] * weight_lanes;

    // [Cost Function 2. Set the weighted cost on traffic density]
    if (total_cnt > 0.1){
      lane_cost[i] += weight_density * cnt[i]/total_cnt;
    }

    // [Cost Function 3. Set the weighted cost on average velocity]
    lane_cost[i] += weight_velofity * (49.5 - average_vel[i]*mph2ms);
  }
 
  // [Cost Function 4. Set the weighted cost if the preceding vehicle is too slow]
  if(too_close1 == true){
    lane_cost[lane] += weight_deceleration;
  }

  std::cout << "Cost[0]: " << lane_cost[0] << " Cost[1]: " << lane_cost[1] << " Cost[2]: " << lane_cost[2] << std::endl; 

  double min_cost = 10000.;
  int index = lane; // default: no lane change

  // [Minimum Cost Function]
  for (int i = 0; i < 3; i++){
    if (lane_cost[i] < min_cost){
      min_cost = lane_cost[i];
      index = i;
    }
  }

  // Plan lane change if there's no other vehicles around the lc area
  // Vehicle State: Lane Keep --> Prepare for Lane Change
  for (int i = 0; i < sensor_fusion.size(); i++){
    float d = sensor_fusion[i][6];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt(vx*vx + vy*vy);
    double check_car_s = sensor_fusion[i][5];

    // Left lane chagne
    if (index - lane < 0 && ref_vel > lc_vel){
      //Check the car in the left lane
      if (d > (2 + 4*lane - 6) && d < (2 + 4*lane - 2)){ 
        //Predict the future s position of the car
        check_car_s += ((double)prev_size * sample_time * check_speed);

        // If the car in behind us is far from our car, we can chane lane
        if ((abs(check_car_s - car_s) > 50)){
          lane -= 1;
        }
      }                   
    } 

    // Right lane chagne
    if (index - lane > 0 && ref_vel > lc_vel){
      //Check the car in the left lane
      if (d > (2 + 4*lane + 2) && d < (2 + 4*lane + 6)){   
        //Predict the future s position of the car
        check_car_s += ((double)prev_size * sample_time * check_speed);

        // If the car in behind us is far from our car, we can chane lane
        if ((abs(check_car_s - car_s) > 50)){
          lane += 1;
        }                        
      } 
    }
  }
 
  return lane;  
}