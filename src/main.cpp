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

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

#define LANE_WIDTH_M (4)
#define MINIMAL_GAP_M (30)
#define MAX_VELOCITY_MPH (49.5)
#define MAX_VELOCITY_KMPH (MAX_VELOCITY_MPH * 1.61)
#define MIN_DISTANCE_CAR (15)
#define COST_LANE_CHANGE (0.04)
#define COST_COLLISION (5.0)
#define COST_LEAVE_LANE (10.0)
#define NUMBER_LANES (3)
#define NUMBER_ACTIONS (3)


enum behavior {Keep_Lane = 0, Lane_Change_Left = 1, Lane_Change_Right = 2};



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
  
  //write to file
  std::freopen( "output.txt", "w", stdout );

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
  
  // start in lane 1
  int lane = 1;
  
  // reference velocity
  double ref_vel = 0; //mph  
  

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) 
  {
                                   
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') 
    {
      auto s = hasData(data);

      if (s != "") 
      {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") 
        {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          car_speed = car_speed / 2.24;

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

		  int prev_size = previous_path_x.size();

		  if(prev_size > 0)
		  {
			  car_s = end_path_s;
		  }
		  
		  bool too_close = false;          
           
          // initial costs for an action in a certain lane
          float cost[NUMBER_LANES][NUMBER_ACTIONS] = {
            {0.0, COST_LEAVE_LANE, COST_LANE_CHANGE},
            {0.0, COST_LANE_CHANGE, COST_LANE_CHANGE},
            {0.0, COST_LANE_CHANGE, COST_LEAVE_LANE}
          };
          
          float dist_closes_car[NUMBER_LANES] = {99999.0, 99999.0, 99999.0};
          int id_closest_car[NUMBER_LANES] = {-1, -1, -1};	// id of the clostest cars on each lane
          float cost_lane[NUMBER_LANES] = {0.0, 0.0, 0.0};	// How good is a lane? 
          
          
		  // lane 0 is left most lane
          for(int i = 0; i < sensor_fusion.size(); ++i)
		  {
			double vx = sensor_fusion[i][3]; // x velocity of car i
			double vy = sensor_fusion[i][4]; // y velocity
			double check_speed = sqrt(vx*vx + vy*vy);	// m/s
			double check_car_s = sensor_fusion[i][5];  // s coordinate  
            check_car_s += ((double)prev_size * 0.02 * check_speed);
            float d = sensor_fusion[i][6];	// d value of car i, What lane is car i in?
            //float min_distance_car = abs(check_speed - car_speed) * 1.5 + MIN_DISTANCE_CAR;
            
            float min_distance_car_front = (car_speed - check_speed) * 1.5 + MIN_DISTANCE_CAR;
            if(min_distance_car_front < MIN_DISTANCE_CAR) min_distance_car_front = MIN_DISTANCE_CAR;
            
            float min_distance_car_back = (check_speed - car_speed) * 2.5 + MIN_DISTANCE_CAR;
            if(min_distance_car_back < MIN_DISTANCE_CAR) min_distance_car_back = MIN_DISTANCE_CAR;
            
            float distance = check_car_s - car_s;
                       
            int check_lane = -1;
            if( d < (2+4 * 0 + 2) && d > (2+4*0 - 2) ) check_lane = 0;
            if( d < (2+4 * 1 + 2) && d > (2+4*1 - 2) ) check_lane = 1;
            if( d < (2+4 * 2 + 2) && d > (2+4*2 - 2) ) check_lane = 2;
            
            std::cout << i <<":  car_s: " << car_s << "  car_speed: " << car_speed << "  check_speed: " << check_speed << "  check_car_s: " << check_car_s << "  d: " << d << "  check_lane: " << check_lane
              		  << "  min_distance_car_front: " << min_distance_car_front << "  min_distance_car_back: " << min_distance_car_back << "  distance: " << distance << std::endl;
            
            
            // costs per lane
            if(distance > 0 && distance < dist_closes_car[check_lane])
            {
              dist_closes_car[check_lane] = distance;
              id_closest_car[check_lane] = i;
              if(distance < min_distance_car_front)
              {
                // cannot change to this lane
                cost_lane[check_lane] = COST_COLLISION; 
              }
              else
              {
              	cost_lane[check_lane] = 1.0 - check_speed/MAX_VELOCITY_KMPH - distance/1000.0; 
              }
            }
            
            if( d < (2+4 * lane + 2) && d > (2+4*lane - 2) )	//my lane?
			{  				  
              if( (check_car_s > car_s) && (check_car_s - car_s) < 30 ) // car in front?
              {
                if(car_speed > check_speed)
                {
                  too_close = true;
                }
                if( (check_car_s < car_s + 1.0) && (check_car_s > car_s) )
                {
                	std::cout << "***** CRASH *******" << std::endl;
                }
              }              
            }            
            
            // cost lane change right
            if( d < (2+4 * (lane + 1) + 2) && d > (2+4*(lane+1) - 2) && (lane + 1) <= 2  )	// car in right lane?
            {             
              if(   ( check_car_s < car_s && (check_car_s + min_distance_car_back) > car_s )  	// car behind
                 || ( check_car_s > car_s && (check_car_s - min_distance_car_front) < car_s ) ) // car in front
              {
                //right lane not free                
                cost[lane][Lane_Change_Right] += COST_COLLISION;
              }
            }

            
            // cost lane change left
            if( d < (2+4 * (lane - 1) + 2) && d > (2+4*(lane-1) - 2) && (lane - 1) >= 0  )	//car in left lane?
            {
              if(   ( check_car_s < car_s && (check_car_s + min_distance_car_back) > car_s )  
                 || ( check_car_s > car_s && (check_car_s - min_distance_car_front) < car_s ) )
              {
                //right lane not free                
                cost[lane][Lane_Change_Left] += COST_COLLISION;
              }
            }
          }

          std::cout<<"id_closest_car: ";
          for(float c: id_closest_car)std::cout << c << "  "; 
          std::cout<<std::endl;
          
          std::cout<<"costs lane: ";
          for(float c: cost_lane)std::cout << c << "  "; 
          std::cout<<std::endl;
          
          std::cout<<"costs change: ";
          for(float c: cost[lane])std::cout << c << "  "; 
          std::cout<<std::endl;
          
          // map action and current lane to next lane
          int action_to_lane[3][3];
          action_to_lane[0][Keep_Lane] = 0;
          action_to_lane[0][Lane_Change_Right] = 1;
          action_to_lane[0][Lane_Change_Left] = -1;
          action_to_lane[1][Keep_Lane] = 1;
          action_to_lane[1][Lane_Change_Right] = 2;
          action_to_lane[1][Lane_Change_Left] = 0;
          action_to_lane[2][Keep_Lane] = 2;
          action_to_lane[2][Lane_Change_Right] = -1;
          action_to_lane[2][Lane_Change_Left] = 1;
          
          // What action do we need to get to from lane1 to lane2?
          int target_lane_to_action[3][3];
          target_lane_to_action[0][0] = Keep_Lane;
          target_lane_to_action[0][1] = Lane_Change_Right;
          target_lane_to_action[0][2] = Lane_Change_Right;
          target_lane_to_action[1][0] = Lane_Change_Left;
          target_lane_to_action[1][1] = Keep_Lane;
          target_lane_to_action[1][2] = Lane_Change_Right;
          target_lane_to_action[2][0] = Lane_Change_Left;
          target_lane_to_action[2][1] = Lane_Change_Left;
          target_lane_to_action[2][2] = Keep_Lane;
          
          //total costs
          float total_cost[] = {9999.0, 9999.0, 9999.0}; // costs for lane and action 
          for(int target_lane = 0; target_lane < 3; ++target_lane)
          {
            int action = target_lane_to_action[lane][target_lane];
            float cost_target_lane = 9999;
            
            cost_target_lane = cost_lane[target_lane];
            
            if(cost_target_lane + cost[lane][action] < total_cost[action])
            	total_cost[action] = cost_target_lane + cost[lane][action];
          }
          
          /*          
          //total costs
          float total_cost[] = {0.0, 0.0, 0.0}; // costs for lane and action 
          for(int action = 0; action < 3; ++action)
          {
          	int target_lane = action_to_lane[lane][action];
            float cost_target_lane = 9999;
            
            if(target_lane != -1)
            {
              cost_target_lane = cost_lane[action_to_lane[lane][action]];
            }
            
            total_cost[action] = cost_target_lane + cost[lane][action];
          }*/
          
          std::cout<<"total costs: ";
          for(float c: total_cost) std::cout << c << "  "; 
          std::cout<<std::endl;
          
          int action_min_cost = -1;
          float min_cost = 99999.0;
          for(int action = 0; action < 3; ++action)
          {
            if(total_cost[action] < min_cost)
            {
              min_cost = total_cost[action];
              action_min_cost = action;
            }
            std::cout<<"action: " << action << "  action_min_cost: " << action_min_cost << "   min_cost: " << min_cost << std::endl;
          }
          
          
          switch(action_min_cost)
          {
            case Keep_Lane:                        
              if( too_close )
              {              
                ref_vel -= .448;       
              }
              else if(ref_vel < MAX_VELOCITY_MPH)
              {                
                ref_vel += .224;	// from Q&A
              }  
              break;
            case Lane_Change_Left:
              lane = lane -1;
              break;
            case Lane_Change_Right:
              lane = lane +1;
          }
              
          
		  vector<double> ptsx;
		  vector<double> ptsy;
		  
		  double ref_x = car_x;
		  double ref_y = car_y;
		  double ref_yaw = deg2rad(car_yaw);
		  
          std::cout << "ref_vel: " << ref_vel << "  prev_size: " << prev_size << "   lane: " << lane << std::endl;
		  if(prev_size < 2) //no previous point
		  {
			  double prev_car_x = car_x - cos(car_yaw);
			  double prev_car_y = car_y - sin(car_yaw);
				
			  ptsx.push_back(prev_car_x);
			  ptsx.push_back(car_x);
			  
			  ptsy.push_back(prev_car_y);
			  ptsy.push_back(car_y);
		  }
		  else
		  {
			  ref_x = previous_path_x[prev_size - 1];
			  ref_y = previous_path_y[prev_size - 1];
			  
			  double ref_x_prev = previous_path_x[prev_size - 2];
			  double ref_y_prev = previous_path_y[prev_size - 2];
			  ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
			  
			  ptsx.push_back(ref_x_prev);
			  ptsx.push_back(ref_x);
			  
			  ptsy.push_back(ref_y_prev);
			  ptsy.push_back(ref_y);			  
		  }
			  
		  vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		  vector<double> next_wp2 = getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

		  ptsx.push_back(next_wp0[0]);
		  ptsx.push_back(next_wp1[0]);
		  ptsx.push_back(next_wp2[0]);
		  
		  ptsy.push_back(next_wp0[1]);
		  ptsy.push_back(next_wp1[1]);
		  ptsy.push_back(next_wp2[1]);
		  
		  for( int i = 0; i<ptsx.size(); ++i)
		  {
			  double shift_x = ptsx[i]-ref_x;
			  double shift_y = ptsy[i]-ref_y;
			  
			  ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
			  ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
			  
		  }
		  
		  tk::spline s;
		  s.set_points(ptsx, ptsy);
		  
		  vector<double> next_x_vals;
          vector<double> next_y_vals;
		  
		  for( int i = 0; i < previous_path_x.size(); ++i)
		  {
			  next_x_vals.push_back(previous_path_x[i]);
			  next_y_vals.push_back(previous_path_y[i]);
		  }
		  
		  double target_x = 30;
		  double target_y = s(target_x);
		  double target_dist = sqrt((target_x * target_x) + (target_y * target_y)); // d
		  
		  double x_add_on = 0;
		  
		  for(int i = 1; i <= 50 - previous_path_x.size(); ++i)
		  {
			  double N = target_dist /( 0.02 * ref_vel / 2.24);
			  double x_point = x_add_on + target_x/N;
			  double y_point = s(x_point);
			  
			  x_add_on = x_point;
			  
			  double x_ref = x_point;
			  double y_ref = y_point;
			  
			  x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
			  y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
			  
			  x_point += ref_x;
			  y_point += ref_y;
			  
			  next_x_vals.push_back(x_point);
			  next_y_vals.push_back(y_point);
			  
		  }
          
//			std::cout << "next_x_vals: ";
//           for(int i = 0; i < next_x_vals.size(); ++i)
//           {
//             std::cout << next_x_vals[i] << ", ";
//           }
//           std::cout << std::endl;
          
//           std::cout << "next_y_vals: ";
//           for(int i = 0; i < next_y_vals.size(); ++i)
//           {
//             std::cout << next_y_vals[i] << ", ";
//           }
//           std::cout << std::endl;
          
		  json msgJson;
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