#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "json.hpp"
#include "spline.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

//***************************************************//
//User Defined Parameters Definition
//***************************************************//
double mph2ms(double x) {return x*1609.34/3600;}

int dflag = 0;
// Debug level
int dflag_general = 1;
int dflag_cost = 2;
int dflag_cost_details = 3;
int dflag_sensor_details = 3;
int dflag_fitting = 5;
int dflag_getXY_short = 7;
int dflag_getXY_full = 8;

int path_count=0;
int iter_count=0;
double max_s = 6945.554;
double car_lane_width = 4.0;

double t_inc = 3;
double t_n = 150;
double c = 2.0;
double c_d = 2.0;
double car_speed_max = 20.5;
double car_s_init = 1.249392e+02;
double car_d_init = 6.164833e+00;
int max_prev_path_size = t_n;

double car_v_init_global = 0.0;
double car_v_end_global = car_speed_max;

double  car_d_init_global = car_d_init;
double car_d_end_global = car_d_init;

bool lane_keeping_flag = true;
double lane_keeping_buffer = 5.0;
double lane_keeping_buffer_v = 1.0;

bool lane_changing_flag = true;
double lane_change_buffer = 25.0;

vector<double> s_history;
vector<double> d_history;

#include "map.h"
#include "trajectories.h"
#include "sensor.h"
#include "behaviors.h"
#include "costs.h"

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

	if(angle > pi()/2)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}
  if (dflag >= dflag_general) {cout << setw(25) << "getFrenet - prev_wp: " << prev_wp << " " <<"next_wp: " << next_wp << endl;}
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
	//for(int i = 0; i < prev_wp; i++)
	//{
	//	frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	//}
  frenet_s = maps_s[prev_wp];
	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;
  if (dflag >= dflag_getXY_full) {cout << setw(25) << "getXY - s:  "<< s << " " << maps_s[prev_wp+1] << " " << (int)(maps_s.size()-1) << endl;}
	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
    if (dflag >= dflag_getXY_full) {cout << setw(25) << "getXY - s:  "<< s << " " << maps_s[prev_wp+1] << " " << (int)(maps_s.size()-1) << endl;}
	}

	int wp2 = (prev_wp+1)%maps_x.size();
  if (dflag >= dflag_getXY_short) {cout << setw(25) << "getXY - wp:  "<< prev_wp << " " << wp2 << endl;}

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double x = maps_x[prev_wp]+seg_s*cos(heading);
	double y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading - pi()/2;

  x += d*cos(perp_heading);
  y += d*sin(perp_heading);

	return {x,y};

}

vector<double> getXY_spline(double s, double d, tk::spline &s_x, tk::spline &s_y, tk::spline &s_dx, tk::spline &s_dy)
{
  double x = s_x(s) + d*s_dx(s);
  double y = s_y(s) + d*s_dy(s);

	return {x,y};
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  map_read_in(map_waypoints_x, map_waypoints_y, map_waypoints_s,
    map_waypoints_dx, map_waypoints_dy);

  tk::spline s_x, s_y, s_dx, s_dy;
  spline_fitting(s_x, s_y, s_dx, s_dy, map_waypoints_s,
    map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);

  /*vector<double> map_waypoints_x_tmp;
  vector<double> map_waypoints_y_tmp;
  vector<double> map_waypoints_s_tmp;
  vector<double> map_waypoints_dx_tmp;
  vector<double> map_waypoints_dy_tmp;*/

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,
     &s_x, &s_y, &s_dx, &s_dy]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
            //car_d = 6;
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];
            car_speed = mph2ms(car_speed);

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            //***************************************************//
            //Parameters Definition
            //***************************************************//
          	json msgJson;
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            double prev_x, prev_y, rev_yaw_rad;
            double prev_s, prev_d;
            double prev_speed, prev_speed2;
            double prev_a;

            int path_size = previous_path_x.size();
            int prev_path_size;

            vector <double> end;
            vector <double> start;

            int sensor_list_size;
            vector<vector<double>> sensor_car_list_left;
            vector<vector<double>> sensor_car_list_mid;
            vector<vector<double>> sensor_car_list_right;

            vector <double> tmp_status;
            double car_s_tmp;
            //***************************************************//
            //Read and Initialize Previous States
            //***************************************************//
            if (path_size > max_prev_path_size){
              prev_path_size = max_prev_path_size;
            }
            else{
              prev_path_size = path_size;
            }
            if (s_history.size()>2){
              prev_s = s_history[s_history.size()-1];
              prev_speed = (s_history[s_history.size()-1] - s_history[s_history.size()-2])/(t_inc/t_n);
              prev_speed2 = (s_history[s_history.size()-2] - s_history[s_history.size()-3])/(t_inc/t_n);
              prev_a = (prev_speed - prev_speed2)/(t_inc/t_n);
              prev_d = d_history[d_history.size()-1];

              car_s_tmp = s_history[(int)t_n - prev_path_size - 1]; //corrected current car_s
              s_history.erase( s_history.begin(), s_history.begin() + (int)t_n - prev_path_size );
              d_history.erase( d_history.begin(), d_history.begin() + (int)t_n - prev_path_size );
            }
            else{
              prev_s = car_s_init;
              prev_speed = 0.0;
              prev_a = 0.0;
              prev_d = car_d_init;
            }

            iter_count++;

            cout << scientific;
            cout << left;
            if (dflag>=dflag_general)
            {cout << setw(25) << "==================================================================" << endl;
            cout << setw(25) << "Start Iteration: " << iter_count << endl;
            cout << setw(25) << "==================================================================" << endl;
            //cout << setw(25) << "pseudo t:  "<< t_current << endl;
            cout << setw(25) << "current status :  "<< car_s << " " << car_d << " " << car_d_end_global << " " << car_speed << " " << car_v_end_global << endl;}
            //<< car_x << " " << car_y << " "   << endl;}

            //***************************************************//
            //Read Sensor Fusion Data
            //***************************************************//
            sensor_processing(sensor_fusion, sensor_car_list_left, sensor_car_list_mid, sensor_car_list_right, car_s);

            //***************************************************//
            //Behavior Planning
            //***************************************************//
            double v_init = car_v_init_global; double v_end = car_v_end_global;
            double d_init = car_d_init_global; double d_end = car_d_end_global;
            int next_signal;

            next_signal = next_state(car_s, car_d, car_speed, sensor_car_list_left, sensor_car_list_mid, sensor_car_list_right);

            cout << setw(25) <<"==================================================================" << endl;
            //cout << setw(25) << "NEXT SIGNAL: " << next_signal << " FLAGS " << lane_keeping_flag << " " << lane_changing_flag << endl;
            if (lane_changing_flag==false){
              cout << "Current Status: " << "Changing Lane" << endl;
              if (next_signal<0){cout << "Direction: " << "Left" << endl;}
              else if (next_signal==0){cout << "Direction: " << "Straight" << endl;}
              else {cout << "Direction: " << "Right" << endl;}
            }
            else if (lane_keeping_flag==false){
              cout << "Current Status: " << "Adjusting Speed" << endl;
              cout << "Current speed: " << car_speed << " Speed target: " << car_v_end_global << endl;
            }
            else{
              cout << "Current Status: " << "Keeping Lane" << endl;
              cout << "Current speed: " << car_speed << " Speed target: " << car_v_end_global << endl;
            }
            if ( (next_signal == 0) && (lane_changing_flag==true) ){
              if (car_d> 2.0*car_lane_width){lane_keeping(sensor_car_list_right, car_s, prev_s, lane_keeping_buffer, v_init, v_end, car_speed, lane_keeping_flag);}
              else if(car_d > car_lane_width){lane_keeping(sensor_car_list_mid, car_s, prev_s, lane_keeping_buffer, v_init, v_end, car_speed, lane_keeping_flag);}
              else {lane_keeping(sensor_car_list_left, car_s, prev_s, lane_keeping_buffer, v_init, v_end, car_speed, lane_keeping_flag);}
            }
            else{ lane_changing(car_s, prev_s, v_init, v_end, car_speed, car_d, d_init, d_end, next_signal, lane_changing_flag);}

            double prev_path_size_kept = s_history.size();

            //***************************************************//
            //Re-Evaluate Previous State after Behavior Planning
            //***************************************************//
            //cout << setw(25) << "all path points in the prev list: " << endl;
            //for (int i = 0; i<path_size; i++) cout << previous_path_x[i] << ' ' << previous_path_y[i] << ' ' << endl;
            if (s_history.size()>2){
              prev_s = s_history[s_history.size()-1];
              prev_speed = (s_history[s_history.size()-1] - s_history[s_history.size()-2])/(t_inc/t_n);
              prev_speed2 = (s_history[s_history.size()-2] - s_history[s_history.size()-3])/(t_inc/t_n);
              prev_a = (prev_speed - prev_speed2)/(t_inc/t_n);
              prev_d = d_history[d_history.size()-1];
            }
            else{
              prev_s = car_s_init;
              prev_speed = 0.0;
              prev_a = 0.0;
              prev_d = car_d_init;
            }
            start = {prev_s, prev_speed, prev_a}; //end = {car_s_next, car_speed_next, car_a_next};

            //***************************************************//
            //Generate Trajectories
            //***************************************************//
            if (dflag>=dflag_general)
            {cout << setw(25) <<"==================================================================" << endl;
            cout << setw(25) << "Trajectories Generation" << endl;
            cout << setw(25) << "==================================================================" << endl;}

            trajectories_generation(start, end, s_history, v_init, v_end, prev_d, d_history, d_init, d_end, max_prev_path_size - prev_path_size_kept,
              t_inc, t_n, car_speed_max, c, c_d);

            //cout << "current speed: "  << car_speed << endl;
            //cout << setw(25) << "==================================================================" << endl;
            //***************************************************//
            //Output Next Path States
            //***************************************************//
            for(int i = 0; i < prev_path_size_kept; i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            path_count += max_prev_path_size - prev_path_size_kept;
            //cout << setw(25) << "current time: " << 0.02*(path_count-150) << " added path size:  "<< int(t_n - prev_path_size_kept) << " total added path size:  " << path_count << endl;

          	for(int i = 0; i < max_prev_path_size - prev_path_size_kept; i++)
          	{
              tmp_status = getXY_spline(s_history[prev_path_size_kept+i], d_history[prev_path_size_kept+i], s_x, s_y, s_dx, s_dy);
              //cout << "index: " << s_history.size() << " " << max_prev_path_size << " " << prev_path_size << " " << i << endl;
              if (dflag>=dflag_general)
              {cout << setw(25) << "pushed in s, d:  "<< s_history[prev_path_size_kept+i] << " "<< d_history[prev_path_size_kept+i] << " "
              << tmp_status[0] << " " << tmp_status[1] << endl;}
              next_x_vals.push_back(tmp_status[0]);
          	  next_y_vals.push_back(tmp_status[1]);
          	}
            //cout << setw(25) << "==================================================================" << endl;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;
            //***************************************************//

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
