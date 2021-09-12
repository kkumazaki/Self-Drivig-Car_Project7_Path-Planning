#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "lanechange.cpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

/* [Step2: Smoothing the Path]*/ 
// Set the initial lane (0: Left, 1: Center, 2: Right)
int lane = 1;

// Set the initial velocity
//double ref_vel = 49.5; //(mph)
double ref_vel = 0.0; //(mph) /* [Step4] */

// Sampling time of control
double sample_time = 0.02; //(sec)

// Transfer (mph) to (m/s)
double mph2ms = 3600.0/1609.0;

// Minimum velocity for lane change
//double lc_vel = 40; //(mph) // after add lanechange.h

// Threshold velocity of preceding vehicle which is too slow
//double slow_vel = 35; //(mph) // after add lanechange.h

// Time to reject the next lane chagne to avoid too many lane changes
int stable_count = 1 * 50; // (sec) /(count/sec)
int lc_count = 0; // (count)

/* [Step2: End]*/

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          //auto sensor_fusion = j[1]["sensor_fusion"];
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          /* [Step2: Smoothing the Path]*/ 

          // Previous path size
          int prev_size = previous_path_x.size();

          /* [Step3: Sensor Fusion]*/
          if (prev_size > 0){
            car_s = end_path_s;
          }

          bool too_close1 = false; // Flag for deceleration at regular distance (preceding vehicle)
          bool too_close2 = false; // Flag for strong deceleration at short distance (preceding vehicle)
          bool too_close3 = false; // Flag for strong acceleration at short distance (behind vehicle)
          bool too_close4 = false; // Flag for strong deceleration for possible other car's Lane Change
          bool too_close5 = false; // Flag for strong deceleration for sudden deceleration (preceding vehicle)

          // Find ref_v to use
          for (int i = 0; i < sensor_fusion.size(); i++){
            // Other cars
            float d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];

            // Check the car in the same lane
            if (d < (2 + 4*lane + 2) && d > (2 + 4*lane -2)){ 
              // Predict the future s position of the car
              check_car_s += ((double)prev_size * sample_time * check_speed);

              // If the car in front of us is close to our car, we should decelerate
              //if((check_car_s > car_s) && ((check_car_s - car_s) < 30)){
              if((check_car_s > car_s) && ((check_car_s - car_s) < 40)){ // too avoid collision when preceding vehicle suddenly stops or other vehicle cutf in
                /*[Step4] Improve smoothness of deceleration and acceleration*/
                //ref_vel = 29.5; 
                too_close1 = true;
              }
              //if((check_car_s > car_s) && ((check_car_s - car_s) < 15)){
              if((check_car_s > car_s) && ((check_car_s - car_s) < 20)){ // too avoid collision when preceding vehicle suddenly stops or other vehicle cutf in
                too_close2 = true;
              }
              if((check_car_s < car_s) && ((check_car_s - car_s) > -30)){
                too_close3 = true;
              }
            }
            // If the car in the left lane tries to Lane Change to my lane, need deceleration
            if (d > (2 + 4*lane - 6) && d < (2 + 4*lane - 2)){
              std::cout << "Left vehicle's d: " << d << std::endl; 
              if((check_car_s > car_s) && ((check_car_s - car_s) < 40) && d > (1.5 + 4*lane - 2)){
                too_close4 = true;
              }
            }

            // If the car in the right lane tries to Lane Change to my lane, need deceleration
            if (d > (2 + 4*lane + 2) && d < (2 + 4*lane + 6)){   
              std::cout << "Right vehicle's d: " << d << std::endl; 
              if((check_car_s > car_s) && ((check_car_s - car_s) < 40) && d < (2.5 + 4*lane + 2)){
                too_close4 = true;
              }
            }
            if((check_car_s > car_s) && ((check_car_s - car_s) < 100) && (check_speed - ref_vel/mph2ms) < -15){
              too_close5 = true;
            }

          }

          // Debug
          //std::cout << "too_close1: " << too_close1 << std::endl;
          //std::cout << "too_close2: " << too_close2 << std::endl;
          //td::cout << "too_close3: " << too_close3 << std::endl;
          std::cout << "too_close4: " << too_close4 << " too_close5: " << too_close5 << std::endl;

          /*[Step5] Lane Change*/
          // Save the current lane information
          int current_lane = lane;

          // Vehicle State
          int vehicle_state = 0; // 0: Lane Keep, 1: Prepare for Lane Change, 2: Lane Change

          lane = lane_change(sensor_fusion, car_s, car_d, car_speed, ref_vel, lane, prev_size, sample_time, too_close1);

          std::cout << "target lane: " << lane << std::endl;

          // Judge Lane change one by one after keeping stable time
          // Vehicle State: Prepare for Lane Change
          if (lane != current_lane){
            vehicle_state = 1; // Prepare for Lane Change
            lc_count += 1;
            if (lc_count < stable_count){
              lane = current_lane;
              //std::cout << "Waiting for stable count." << std::endl;
            } else{
              //std::cout << "After stable count." << std::endl;
              if(lane < current_lane){
                lane = current_lane - 1; // Reject direct lane change from Right to Left
              } else {
                lane = current_lane + 1; // Reject direct lane change from Left to Right
              }
              lc_count = 0;
            } 
          }
          else{
            std::cout << "Terget lane is equal to Current lane." << std::endl;
            lc_count = 0;
          }

          std::cout << "lc_count: " << lc_count << std::endl;

          // Execute Lane change if it's safe to avoid collision
          // Vehicle State: Prepare for Lane Change --> Lane Change
          for (int i = 0; i < sensor_fusion.size(); i++){
            // Other cars
            float d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];

            // Left lane change
            if (lane < current_lane){
              
              if (d > (2 + 4*current_lane - 6) && d < (2 + 4*current_lane - 2)){ 
                // Predict the future s position of the car
                check_car_s += ((double)prev_size * sample_time * check_speed);

                //Debug
                std::cout << "Other vehicle speed: " << check_speed * mph2ms << std::endl;
                std::cout << "Other vehicle speed difference: " << (ref_vel/mph2ms - check_speed) << std::endl;
                std::cout << "Other vehicle position difference: " << (check_car_s - car_s) << std::endl;
                std::cout << "TTC: " << abs((check_car_s - car_s)/(ref_vel/mph2ms - check_speed)) << std::endl;

                // If the car is close to our car or speed difference is large, reject lane change
                //if((abs(check_car_s - car_s) < 30) && (abs(check_speed * mph2ms - ref_vel) > 15)){
                if(abs(check_car_s - car_s) < 30.0 || (abs((check_car_s - car_s)/(ref_vel/mph2ms - check_speed)) < 3.0)){
                  lane = current_lane;
                  std::cout << "Reject Lane Change to avoid collision." << std::endl;
                } else {
                  vehicle_state = 2; // Lane Change
                }
              }
            }
            // Right lane change 
            else if (lane > current_lane){
              
              if (d > (2 + 4*current_lane + 2) && d < (2 + 4*current_lane + 6)){ 
                // Predict the future s position of the car
                check_car_s += ((double)prev_size * sample_time * check_speed);

                //Debug
                std::cout << "Other vehicle speed: " << check_speed * mph2ms << std::endl;
                std::cout << "Other vehicle speed difference: " << (ref_vel/mph2ms - check_speed) << std::endl;
                std::cout << "Other vehicle position difference: " << (check_car_s - car_s) << std::endl;
                std::cout << "TTC: " << (check_car_s - car_s)/(ref_vel/mph2ms - check_speed) << std::endl;

                // If the car is close to our car or speed difference is large, reject lane change
                if(abs(check_car_s - car_s) < 30.0 || (abs((check_car_s - car_s)/(ref_vel/mph2ms - check_speed)) < 3.0)){
                  lane = current_lane;
                  std::cout << "Reject Lane Change to avoid collision." << std::endl;
                } else {
                  vehicle_state = 2; // Lane Change
                }
              }
            }
          }

          std::cout << "Final Lane: " << lane << std::endl;

          if (vehicle_state == 0){
            std::cout << "Vehicle State: Lane Keep"<< std::endl;
          }  else if (vehicle_state == 1){
            std::cout << "Vehicle State: Prepare for Lane Change"<< std::endl;
          }  else {
            std::cout << "Vehicle State: Lane Change"<< std::endl;
          }

          std::cout << "------------------------------" << std::endl;

          /*[Step5] End*/

          if (too_close1){
            ref_vel -= 0.2; // regular deceleration
          }
          else if(too_close2){
            ref_vel -= 0.4; // strong deceleration
          }
          else if(too_close4){
            ref_vel -= 0.3; // strong deceleration
          }
 
          else if(ref_vel < 49.5){
            ref_vel += 0.3;
            if (too_close3){
              ref_vel += 0.1; // strong acceleration
            }
          }
          else if(too_close5){
            ref_vel -= 0.4; // strong deceleration
          }
          /* [Step4: End]*/

          /* [Step3: End]*/

          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // Later we will interpolate these waypoints with a spline and fill it in with more points
          vector<double> ptsx;
          vector<double> ptsy;

          // Refference x, y, yaw states
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // If there's no previous path at beginning, follow the path before 1 step
          if(prev_size < 2){
            //Use two points that make tha path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // Else, use the previous path information as starting reference
          else{
            // Redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // Use two points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // In Frenet add evenly 30m spaed points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // Shift the x, y positions for spline
          for (int i = 0; i<ptsx.size(); i++){
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }

          // Create a spline
          tk::spline s;
          s.set_points(ptsx, ptsy);

          //vector<double> next_x_vals;
          //vector<double> next_y_vals;

          // Start with all of the previous path points from last time
          for (int i = 0; i<previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
          double x_add_on = 0;

          // Fill up the rest of our path planner after filling in with previous points.
          // Output 50 points.
          for (int i = 1; i <= 50-previous_path_x.size(); i++){
            double N = (target_dist/(sample_time*ref_vel / mph2ms));
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // Rotate back to normal after rotating it ealier
            x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }


 
          /*[Step2: End]*/

          /* [Step1: Just run by keeping center lane.]*/
          /*
          double dist_inc = 0.5;
          for (int i = 0; i < 50; i++){
            double next_s = car_s + i*dist_inc;
            double next_d = 6;
            vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }
          */
          /* [Step1: End]*/

          /* TODO: End */

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}