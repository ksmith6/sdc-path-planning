#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#include "Eigen-3.3/Eigen/LU"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Global constants
const double MAX_S = 6945.554; // meters
const double LANE_WIDTH = 4.0; // meters

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

/*
  ===== Prototype Lane Following Behavior ======
  This is my first attempt at tracking a particular lane while commanding a constant velocity in Frenet coordinates.
  The resulting trajectories are very jerky as the trajectory is not smoothed at all.
*/
vector<vector<double>> laneFollowing(double car_s, vector<double> previous_path_x, vector<double> previous_path_y, 
                                      vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, bool verbose)
{
  
  double target_lane = 1.0;
  double target_d = LANE_WIDTH * (0.5 + target_lane);
  //vector<double> next_d_vals;
  //vector<double> next_s_vals;
  double dist_inc = 0.5;

  int path_size = previous_path_x.size();

  if (verbose) {cout << "Adding " << path_size << " elements from previous path." << endl << "Previous elements: " << endl;}


  vector<double> next_x_vals;
  vector<double> next_y_vals;
  // Copy previous path points 
  for(int i = 0; i < path_size; i++) {

      if (verbose) { cout << "#" << i << "(" << previous_path_x[i] << "," << previous_path_y[i] << ")" << endl; }
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
  }

  // Get last state from previous points, if it exists.
  double prev_s;
  if (path_size == 0) {
    prev_s = car_s;
  } else {
    double y1 = previous_path_y[path_size-1];
    double y2 = previous_path_y[path_size-2];
    double x1 = previous_path_y[path_size-1];
    double x2 = previous_path_y[path_size-2];
    double prev_yaw = atan2(y1-y2, x1-x2); 
    vector<double> sd = getFrenet(previous_path_x[path_size-1], previous_path_y[path_size-1], prev_yaw, map_waypoints_x, map_waypoints_y);
    prev_s = sd[0];
  }

  // Generate new points that simply follow the centerline of the target lane.  No smoothing is performed.
  if (verbose) { cout << "New points:" << endl; }
  for(int i = 1; i < 50 - path_size; i++)
  {
      // Ensure wraparound when crossing the MAX_S boundary.
      double target_s = fmod(prev_s + dist_inc*i, MAX_S);
      vector<double> xy = getXY(target_s, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
      if (verbose) {
        cout << "target_s = " << target_s << "m | target_d = " << target_d << "m  | ";
        cout << "xy[0] = " << xy[0] << " | xy[1] = " << xy[1] << endl;
      }
      next_x_vals.push_back(xy[0]);
      next_y_vals.push_back(xy[1]);
  }

  if (verbose) { cout << "------------" << endl << endl; }
  return {next_x_vals, next_y_vals};
} 

/*
  Initial demonstration code for driving in a straight line in the simulator.
*/
vector<vector<double>> straightLine(double car_x, double car_y, double car_yaw) {
  double dist_inc = 0.1;

  vector<double> next_x_vals;
  vector<double> next_y_vals;
  for(int i = 0; i < 50; i++)
  {
    next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
    next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
  }
  return {next_x_vals, next_y_vals};
}

/*
  Initial demonstration code for driving in a circle in the simulator.  
*/
vector<vector<double>> drive_in_a_circle(double car_x, double car_y, double car_yaw, vector<double> previous_path_x, vector<double> previous_path_y) {
  
  double pos_x;
  double pos_y;
  double angle;
  int path_size = previous_path_x.size();

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  for(int i = 0; i < path_size; i++)
  {
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
  }

  if(path_size == 0)
  {
      pos_x = car_x;
      pos_y = car_y;
      angle = deg2rad(car_yaw);
  }
  else
  {
      pos_x = previous_path_x[path_size-1];
      pos_y = previous_path_y[path_size-1];

      double pos_x2 = previous_path_x[path_size-2];
      double pos_y2 = previous_path_y[path_size-2];
      angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
  }

  double dist_inc = 0.5;
  for(int i = 0; i < 50-path_size; i++)
  {    
      next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
      next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
      pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
      pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
  }
  return {next_x_vals, next_y_vals};
}

/*
  Utility function to compute the 6th order polynomial coefficents for a jerk-minimal trajectory between two points over some duration T.
*/
vector<double> min_jerk_trajectory(double x_i, double v_i, double a_i, double x_f, double v_f, double a_f, double T) {

  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
          3*T*T, 4*T*T*T,5*T*T*T*T,
          6*T, 12*T*T, 20*T*T*T;
    
  MatrixXd B = MatrixXd(3,1);     
  B << x_f-(x_i + v_i*T + 0.5*a_i*T*T),
          v_f - (v_i + a_i*T),
          a_f - a_i;
          
  MatrixXd Ai = A.inverse();
  
  MatrixXd C = Ai*B;
  
  vector <double> result = {x_i, v_i, .5*a_i};
  for(int i = 0; i < C.size(); i++)
  {
      result.push_back(C.data()[i]);
  }
  
  return result;
}


/*
  Function to determine if it is safe to switch from current lane to next lane.
*/
bool safeToEnterLane(double car_s, double car_speed, int targetLane, vector<vector<double>> sensor_fusion) {
  
  bool isSafeToEnter = true;
  for (int i=0; i<sensor_fusion.size(); i++) {
    float d = sensor_fusion[i][6];

    // Is the other car in our targeted lane?
    bool in_target_lane = d < LANE_WIDTH*(targetLane+1) && d > LANE_WIDTH*(targetLane);

    // Fetch the other car's s-coordinate in Frenet.
    double check_car_s = sensor_fusion[i][5];

    // Compute the relative range.
    double relRange = check_car_s-car_s;

    if (in_target_lane && fabs(relRange) < 20) {
      /* If there's a car within 20 meters of our current position in 
      the target lane, don't switch lanes. */
      isSafeToEnter = false;
    } 
  }  

  return isSafeToEnter;
}


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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  int lane = 1;

  double ref_vel = 0.0; // mph
  const double MAX_SPEED = 49.5;

  h.onMessage([&ref_vel, &MAX_SPEED, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

            int prev_size = previous_path_x.size();

            if (prev_size > 0) {
              car_s = end_path_s;
            }

            double M_TO_MPH = 2.24;
            bool too_close = false;


            double speed_constraint = MAX_SPEED; // mph
            for (int i=0; i<sensor_fusion.size(); i++) {
              float d = sensor_fusion[i][6];
              bool in_my_lane = d<(2+4*lane+2) && d > (2+4*lane-2);
              if (in_my_lane) {
                // car is in my lane.
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];

                check_car_s += ((double) prev_size)*0.02*check_speed;
                bool inFront = check_car_s > car_s;
                double relRange = check_car_s-car_s;

                /* If there's a car in front of us and the 
                separation distance is less than 50 meters, then begin 
                tracking its speed (if it's slower than us). */
                if (inFront && relRange < 50) {
                  // Start slowing down to match leader speed
                  speed_constraint = min(speed_constraint, check_speed * M_TO_MPH);
                  
                  /* If we're more than 3mph below the max speed, the car in front has become 
                  an obstacle, so let's try to pass it. */
                  if (MAX_SPEED - speed_constraint > 3) {
                    too_close = true;
                  }
                }
              }
            }

            if (too_close) {

              // Check our lane to see if we can pass on left or right.
              bool canPassLeft = lane > 0;
              bool canPassRight = lane < 2;

              bool safeToPassLeft;

              /* If a left-pass is feasible, can we safely enter the left lane? */
              if (canPassLeft) {
                // Is it safe to pass in nextmost left lane given current traffic conditions?
                safeToPassLeft = safeToEnterLane(car_s, car_speed, lane-1, sensor_fusion);
              } else {
                // Already is left lane, so cannot pass on left.
                safeToPassLeft = false;
              }

              /* If a right-pass is feasible, can we safely enter the right lane? */
              bool safeToPassRight;
              if (canPassRight) {
                // Is it safe to pass in nextmost right lane given current traffic conditions?
                safeToPassRight = safeToEnterLane(car_s, car_speed, lane+1, sensor_fusion);
              } else {
                safeToPassRight = false;
              }


              if (canPassLeft && safeToPassLeft) {
                // Pass on the left, since it is feasible and safe.
                lane -= 1;
              } else if (canPassRight && safeToPassRight) {
                // Pass on the right, since it is feasible and safe.
                lane += 1;
              } else {
                // Not yet safe to pass, so just maintain safe speed in current lane.

                if (ref_vel >= speed_constraint) {
                  ref_vel -= 0.244;
                }

              }

            } else {
              // No obstacles in path.  Just maintain speed.
              if (ref_vel < speed_constraint) {
                ref_vel += 0.224;
              }
            }


            /*
            The algorithm below is reproduced from the Udacity Walkthrough video.

            I had a tough time building my own from scratch without introducing 
            hiccups in the path that would cause jerk and acceleration violations.

            Therefore, I abandoned that work and just reproduced the Udacity 
            version to have something functional.

            */
            vector<double> ptsx, ptsy;
            double ref_x = car_x;
            double ref_y = car_d;
            double ref_yaw = (car_yaw);
          	
            /* Is this a cold start? */
            if (prev_size < 2) {
              // Cold Start Logic

              // Generate two anchors to constrain spline to honor current yaw.

              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            } else {
              // Warm Start Logic
              // Generate two anchors from the end of the previous path.
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];

              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];

              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
            }

            // Sample distant waypoints 
            vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
            // Add waypoints to anchor set.
            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);


            // Translate and rotate the anchor points to help spline.h
            for (int i=0; i<ptsx.size(); i++) {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
              ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
            }

            // Create spline
            tk::spline s;

            s.set_points(ptsx, ptsy);

            vector<double> next_x_vals;
            vector<double> next_y_vals;


            /* Re-use the previous points from the previous path */
            for (int i=0; i<previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            /* Generate new points from the spline based on the anchors */
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);

            double x_add_on = 0.0;
            for (int i=1; i<=50 - previous_path_x.size(); i++) {
              double N = (target_dist) / (0.02 * ref_vel /2.24);
              double x_point = x_add_on + target_x / N;
              double y_point = s(x_point);
              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // rotate back
              x_point = x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw);

              // shift back.
              x_point += ref_x;
              y_point += ref_y;

              // Add to final path vectors
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
















































































