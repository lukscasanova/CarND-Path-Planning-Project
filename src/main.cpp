#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

#include "json.hpp"

#include "spline.h"
#include "map_spline.h"
#include "car.h"
#include "trajectory.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


/**************************************************************************************/


vector<double> smoothTrajectory(vector<double> traj){

  int traj_size = traj.size();

  for(int i = 3; i < traj_size-3; ++i){
    double temp = (4*traj[i-3] + 3*traj[i-2] + 2*traj[i-1] + traj[i] + 2*traj[i+1] + 3*traj[i+1] + 4*traj[i+3])/19;
    traj[i] = temp;
  }

  return traj;
}



/***************************************************************************************/

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

  // Calculate using previous wp
	int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// double heading2 = atan2((maps_y[wp3]-maps_y[wp2]),(maps_x[wp3]-maps_x[wp2]));

  // std::cout << "heading 1: " << heading << std::endl;
  // std::cout << "heading 2: " << heading2 << std::endl;
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);
  // double interpolate_factor = seg_s/(maps_s[wp2]-maps_s[prev_wp]);

  // double heading_2;

  // std::cout << "interpolate_factor1: " << interpolate_factor << std::endl;

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  // heading = heading2*interpolate_factor + heading*(1-interpolate_factor);
  // heading = atan2(sin(heading), cos(heading));    

  // std::cout << "interpolated heading: " << heading << std::endl;
	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

  // std::cout << "x,y: " << x << ", " << y << std::endl;


	return {x,y};

}






void averageTrajectories(vector<double>& avx, vector<double>& avy, vector<double> new_x, vector<double> new_y, 
  vector<double> old_x, vector<double> old_y){
  
  int smaller_size = old_x.size()/3;
  int bigger_size = new_x.size();
  avx.clear();
  avy.clear();  
  cout << "smaller_size: " << smaller_size << endl;
  cout << "bigger_size: " << bigger_size << endl;

  for(int i = 0; i < smaller_size; ++i){
    // avx.push_back( (new_x[i]+old_x[i])/2);
    // avy.push_back( (new_y[i]+old_y[i])/2);
    avx.push_back(old_x[i]);
    avy.push_back(old_y[i]);
  }
  for(int i = smaller_size; i < bigger_size; ++i){
    double dx = new_x[i]-new_x[i-1];
    double dy = new_y[i]-new_y[i-1];

    avx.push_back(avx[i-1]+dx);
    avy.push_back(avy[i-1]+dy);

  }
}


Trajectory generateTrajectory(Car start, Car end, Trajectory previous, double time_interval, int stitchPoint, MapSpline ms){
  
  double delta_t = 0.02;
 
  int path_size = previous.x_vals.size();
  int start_index=0;

  if(path_size==0){
    previous.createJerkMinimized(start, end, time_interval, ms);
  }else{
    previous.extendJerkMinimized(start.s, stitchPoint, end, time_interval, ms);
  }

  previous.x_vals = smoothTrajectory(previous.x_vals);
  previous.y_vals = smoothTrajectory(previous.y_vals);

  cout << "previous size xy: " << previous.x_vals.size() << endl;

  return previous;
}




typedef struct planner_state{
  double last_s;
  double last_d;
  double last_speed;
  bool initialized;
} State;

int main() {
  uWS::Hub h;


  Car car;
  Trajectory traj;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &car, &traj](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            Trajectory previous;
            



            // int base_wp = getPrevWp(car_s, map_waypoints_s)-1;

            // vector<double> wp_s, wp_x, wp_y, wp_dx, wp_dy;
            // for(int i = -1; i <= 20; ++i){
            //   wp_s.push_back(map_waypoints_s[base_wp+i]);
            //   wp_x.push_back(map_waypoints_x[base_wp+i]);
            //   wp_y.push_back(map_waypoints_y[base_wp+i]);
            //   wp_dx.push_back(map_waypoints_dx[base_wp+i]);
            //   wp_dy.push_back(map_waypoints_dy[base_wp+i]);
            // }

            // tk::spline wp_x_spline, wp_y_spline, wp_dx_spline, wp_dy_spline;

            // wp_x_spline.set_points(wp_s, wp_x);
            // wp_y_spline.set_points(wp_s, wp_y);
            // wp_dx_spline.set_points(wp_s, wp_dx);
            // wp_dy_spline.set_points(wp_s, wp_dy);

            MapSpline ms;
            ms.create(car_s, map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);


            // car.updateState(car_s, car_d);
            car.printState();

            double goal_s, goal_d;
            double pos_x, pos_y, angle;
            double pos_s, pos_d;
            int start_index;            
            int path_size = previous_path_x.size();

            std::cout << "path_size: " << path_size << std::endl; 


            cout << "small path_size" << endl;
            // goal_s = pos_s + 10;

            goal_d = 2 + ( ( ( ((int)pos_s) / 400 ) + 1 ) % 3) * 4;

  
            
            std::cout << "curr:" << car_s   << ", " << car_d    << std::endl;

            std::cout << "car_speed: " << car_speed << endl;

  
            double T = 3.0;


            // Trajectory traj;

            Car start, end;

            start.s   = car_s;
            start.s_v = 10.0;
            start.s_a = 0;

            start.d   = 6;
            start.d_v = 0;
            start.d_a = 0;

            end.s   = car_s+ 30;
            end.s_v = 10.0;
            end.s_a = 0;

            end.d   = 6;
            end.d_v = 0;
            end.d_a = 0;

            Trajectory out;

            if(path_size<100){

              traj = generateTrajectory(start, end, traj, T, 10, ms);
              // out = traj;
              next_x_vals = traj.x_vals;
              next_y_vals = traj.y_vals;


  
              // std::vector<double> previous_path_s, previous_path_d;

              // cout << "Previous path xy" << endl;
              // for(int i = 0 ; i < previous_path_y.size(); i++){
              //   cout << "xy: " << previous_path_x[i] << ", " << previous_path_y[i] << endl;
              //   auto sd = getFrenet(previous_path_x[i], previous_path_y[i], deg2rad(car_yaw), map_waypoints_x, map_waypoints_y);
              //   previous_path_s.push_back(sd[0]);
              //   previous_path_d.push_back(sd[1]);
              // }

              // // traj.vals_s = previous_path_s;
              // // traj.vals_d = previous_path_d;
              
              // // S,D TRAJ TO XY

              // double delta_t = 0.02;
              // // vector<double> dxy;
              // // auto xy = splineGetXY(car_s, car_d, wp_x_spline, wp_y_spline, wp_dx_spline, wp_dy_spline);

              // // dxy.push_back(car_x-xy[0]);
              // // dxy.push_back(car_y-xy[1]);

              // int stitchPoint = 10;

              // if(path_size==0){

              //   traj.createJerkMinimized(start, end, T);
              // }else{
              //   traj.extendJerkMinimized(car_s, stitchPoint, end, T);
              //   // end.s+=50;
              //   // traj.extendJerkMinimized(car_s, 60, end, 2*T);
              // }

              // int i;
              // for(i = 0; i < stitchPoint && i < path_size; ++i){
              //   next_x_vals.push_back(previous_path_x[i]);
              //   next_y_vals.push_back(previous_path_y[i]);
              // }

              // for(; i<traj.vals_s.size(); ++i){
              //   // double t = i * delta_t;

              //   double s = traj.vals_s[i];
              //   double d = traj.vals_d[i];
              //   // double d = evaluate(coeff_d, t);

              //   // cout << "s,d: " << s << ", " << d << endl;

              //   auto xy = splineGetXY(s,d, wp_x_spline, wp_y_spline, wp_dx_spline, wp_dy_spline);
              //   // // auto xy = getXY(s,d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

              //   next_x_vals.push_back(xy[0]);
              //   next_y_vals.push_back(xy[1]);

              // }
              
              // next_x_vals = smoothTrajectory(next_x_vals);
              // next_y_vals = smoothTrajectory(next_y_vals);
              // cout << "out path xy" << endl;
              
              // for(int i = 0 ; i < next_x_vals.size(); i++){
              //   cout << "xy: " << next_x_vals[i] << ", " << next_y_vals[i] << endl;
              // }

            }else{
              // out = traj;
              for(int i = 0; i < path_size; ++i){
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
              }
            }

            
            // if(path_size>0){
            //   averageTrajectories(next_x_vals, next_y_vals,  next_x_vals, next_y_vals, previous_path_x, previous_path_y);
            // }
            // }else{
            //   avx = next_x_vals;
            //   avy = next_y_vals;
            // }
            // for(size_t m = 0 ; m < next_x_vals.size(); m++){
            //   cout << "x,y: " << next_x_vals[m] << ", " << next_y_vals[m] << endl;
            // }
            // }


            std::cout << "out path size: " << next_x_vals.size();
            
           

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
















































































