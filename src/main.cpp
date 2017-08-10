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

#define DELTA_T 0.02


/**************************************************************************************/


vector<double> smoothTrajectory(vector<double> traj){

  int traj_size = traj.size();
  vector<double> new_traj;
  new_traj.push_back(traj[0]);
  // new_traj.push_back(traj[1]);

  for(int i = 1; i < traj_size-1; ++i){
    double temp = (traj[i-1] + traj[i] + traj[i+1])/3;
    new_traj.push_back(temp);
  }
  new_traj.push_back(traj[traj_size-1]);

  return new_traj;
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

  previous.x_vals = smoothTrajectory(previous.x_vals);
  previous.y_vals = smoothTrajectory(previous.y_vals);

  previous.x_vals = smoothTrajectory(previous.x_vals);
  previous.y_vals = smoothTrajectory(previous.y_vals);

  previous.x_vals = smoothTrajectory(previous.x_vals);
  previous.y_vals = smoothTrajectory(previous.y_vals);



  // // cout << "previous size xy: " << previous.x_vals.size() << endl;

  return previous;
}

void calcVelAccJerk(Trajectory traj, double& minVel, double& maxVel, 
  double& maxAcc, double& maxJerk, double& maxAngVel, double& maxAngAcc){
  
  maxAcc = 0;
  maxJerk = 0;
  maxVel = 0;
  minVel = 9999;
  maxAngVel=0;
  maxAngAcc=0;
  
  double acc, vel, jerk, ang_vel, angle, ang_acc;
  double last_vel=0;
  double last_acc=0;
  double last_angle=0;
  double last_ang_vel=0;

  for(int i = 1; i < traj.x_vals.size()-2; i++){
    double diff_x = traj.x_vals[i]-traj.x_vals[i-1];
    double diff_y = traj.y_vals[i]-traj.y_vals[i-1];
    double diff_s = (traj.s_vals[i]-traj.s_vals[i-1])/DELTA_T;

    vel = sqrt(diff_x*diff_x + diff_y*diff_y)/DELTA_T;
    angle = atan2(diff_y, diff_x);

    if(vel > maxVel) maxVel = vel;
    if(diff_s < minVel) minVel = diff_s;

    if(i>1){
      acc = (vel - last_vel)/DELTA_T;
      if(abs(acc) > maxAcc) maxAcc = abs(acc);

      ang_vel = atan2(sin(angle-last_angle), cos(angle-last_angle))/DELTA_T;
      if(abs(ang_vel) > maxAngVel) maxAngVel = abs(ang_vel);
    }

    if(i>2){
      jerk = (acc - last_acc)/DELTA_T;
      if(abs(jerk) > maxJerk) maxJerk = abs(jerk);

      ang_acc = (ang_vel - last_ang_vel)/DELTA_T;
      if(abs(ang_acc) > maxAngAcc) maxAngAcc = abs(ang_acc);
    }

    last_vel = vel;
    last_acc = acc;
    last_angle = angle;
    last_ang_vel = ang_vel;
  }

  // printf("minVel:%lf, maxVel: %lf, maxAcc: %lf, maxJerk: %lf, maxAngVel: %lf, maxAngAcc: %lf\n",
    // minVel, maxVel, maxAcc, maxJerk, maxAngVel, maxAngAcc);
}


bool testInLane(double d){
  while(d>4) d-=4;
  return (d>1 && d<3);
}

int getLane(double d){
  return (int) d/4.0;
}

bool testInStreet(double d){
  return (d>1 && d<11);
}
void calcLanesParameters(Trajectory traj, int& laneSwitches, int& outsideLane, int& outsideStreet){
  laneSwitches=0;
  outsideLane=0;
  outsideStreet=0;

  int last_lane = getLane(traj.d_vals[0]);
  for(int i = 0; i < traj.d_vals.size(); ++i){
    double d = traj.d_vals[i];
    int curr_lane = getLane(d);
    
    if(curr_lane!= last_lane){
      laneSwitches++;
      last_lane=curr_lane;
    }

    if(!testInLane(d)){
      outsideLane++;
    }
    if(!testInStreet(d)){
      // cout << "d: " << d << endl;
      outsideStreet++;
    }
  }

  // printf("laneSwitches: %d, outsideLane: %d, outsideStreet: %d\n", laneSwitches, outsideLane, outsideStreet);
}


double scoreTrajectory(Trajectory traj){

  double score=0;

  double KINEMATIC_WEIGHT=1;
  double LANE_WEIGHT=1;

  /*************** SCORE KINEMATICS *******************/
  double maxAcc, maxJerk, maxVel, minVel, maxAngVel, maxAngAcc;
  calcVelAccJerk(traj, minVel, maxVel, maxAcc, maxJerk, maxAngVel, maxAngAcc);


  double ACC_LIMIT = 5, JERK_LIMIT =30, VEL_LIMIT=22, ANG_VEL_LIMIT=0.4, ANG_ACC_LIMIT=10000;
  double ACC_WEIGHT= 400, JERK_WEIGHT=100, MAXVEL_WEIGHT=-200, MINVEL_WEIGHT=-150, ANG_VEL_WEIGHT=1000, ANG_ACC_WEIGHT=20;

  if(maxAcc>ACC_LIMIT){
    ACC_WEIGHT = 1000000;
  }

  if(maxJerk>JERK_LIMIT){
    maxJerk = 1000000;
  }

  if(maxVel > VEL_LIMIT){
    MAXVEL_WEIGHT = 1000000;
  }

  if(minVel < 0){
    MINVEL_WEIGHT = -100000;
  }

  if(maxAngVel>ANG_VEL_LIMIT){
    ANG_VEL_WEIGHT = 10000;
  }


  score += KINEMATIC_WEIGHT*(maxAcc*ACC_WEIGHT + maxJerk*JERK_WEIGHT 
    + maxVel*MAXVEL_WEIGHT+minVel*MINVEL_WEIGHT + maxAngVel*ANG_VEL_WEIGHT + maxAngAcc*ANG_ACC_WEIGHT);

  /*********************************************************/

  /********************* SCORE LANES **********************/

  int laneSwitches, outsideLane, outsideStreet;
  calcLanesParameters(traj, laneSwitches, outsideLane, outsideStreet);

  double SWITCH_WEIGHT=0, OUTSIDE_LANE_WEIGHT=10, OUTSIDE_STREET_WEIGHT=10000;
  int OUTSIDE_LANE_LIMIT = 100;

  if(outsideLane>OUTSIDE_LANE_LIMIT) OUTSIDE_LANE_WEIGHT = 100;

  score+= LANE_WEIGHT*(SWITCH_WEIGHT*laneSwitches + OUTSIDE_LANE_WEIGHT * outsideLane + OUTSIDE_STREET_WEIGHT * outsideStreet);

  /********************************************************/

  // cout << "score: " << score << endl;
  return score;

}

double dist(double x1, double y1, double x2, double y2){
  return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

bool collides(Trajectory traj, std::vector<Car> cars, double time){
  double time_step = 0.25;
  for(double i=time_step; i <= time; i+=time_step){
    int traj_index = i/DELTA_T;
    for(int j=0; j < cars.size(); j++){
      double ego_x, ego_y, car_x, car_y;
      ego_x = traj.x_vals[traj_index];
      ego_y = traj.y_vals[traj_index];

      car_x = cars[j].x+cars[j].x_v*i/2;
      car_y = cars[j].y+cars[j].y_v*i/2;

      if(dist (ego_x, ego_y, car_x, car_y) < 4+2*i){
        // test same lane
        double ego_d, car_d;
        ego_d = traj.d_vals[traj_index];
        car_d = cars[j].d;
        if(abs(ego_d-car_d)<2.5){
          // cout << "predicted collision"<< endl;
          // cout << "T: " << time_step << endl;
          // printf("ego xy: %lf, %lf x ", ego_x, ego_y);
          // printf("car xy: %lf, %lf\n", car_x, car_y);
          return true;
        }        
      }
    }
  }
  return false;
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
	double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}





typedef struct planner_state{
  double last_s;
  double last_d;
  double last_speed;
  bool initialized;
} State;

int main() {
  uWS::Hub h;

  Trajectory previous;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &previous](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

            std::vector<Car> cars;

            for(int i =0; i < sensor_fusion.size(); ++i){
              cout << sensor_fusion[i] << endl;
              if(sensor_fusion[i][6]>0){
                Car c;
                // using s and d to store xy values;
                c.x = sensor_fusion[i][1];
                c.y = sensor_fusion[i][2];
                c.x_v = sensor_fusion[i][3];
                c.y_v = sensor_fusion[i][4];
                c.s = sensor_fusion[i][5];
                c.d = sensor_fusion[i][6];
                cars.push_back(c);
              }
            }

            cout << "BP 1" << endl;
            std::cout << "curr:" << car_s   << ", " << car_d    << std::endl;


          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            MapSpline ms;
            ms.create(car_s, map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);

            cout << "BP 2" << endl;

            int path_size = previous_path_x.size();


            Car start, end;

            start.s   = car_s;
            start.s_v = 0.0;
            start.s_a = 0;

            start.d   = car_d;
            start.d_v = 0;
            start.d_a = 0;

            Trajectory out;

            if(path_size<150){
              std::vector<Trajectory> trajs;
              double T = 4;
              int stitchPoint = 30;

              // Generate Several Trajectories
              // for(int i = 0; i < 1; i++){

              for(int j = 0; j < 10; j++){ // 5 velocities
                end.s_v = 2.5*j;
                for(int m = 0; m <= 10; m++){ // 10 endpoints
                  end.s = start.s + 5+ m*10;
                  for(int n = 0; n < 2; n++){ // 3 lanes 
                    end.d = 2+n*4;
                    Trajectory temp = generateTrajectory(start, end, previous, T,stitchPoint, ms);
                    // scoreTrajectory(temp);
                    trajs.push_back(temp);
                  }
                }
              }
              // }

              cout << "generated " << trajs.size() << "trajectories" << endl;
              // Eliminate collision trajectorie
              std::vector<Trajectory> collision_free_trajs;
              for(int i = 0; i < trajs.size(); ++i){
                if(!collides(trajs[i], cars, T)){
                  collision_free_trajs.push_back(trajs[i]);
                }
              }
              cout << "collision_free_trajs:  " << collision_free_trajs.size() << "trajectories" << endl;


              // Score and choose the best

              


              if(collision_free_trajs.size()>0){

                double bestScore = scoreTrajectory(collision_free_trajs[0]);
                int index = 0;
                
                for(int i = 1; i < collision_free_trajs.size(); ++i){
                  double score = scoreTrajectory(collision_free_trajs[i]);
                  if(score < bestScore){
                    bestScore = score;
                    index = i;
                  }
                }


                cout << "chosen trajetory: " << endl;
                scoreTrajectory(collision_free_trajs[index]); 
                collision_free_trajs[index].print();

                previous = collision_free_trajs[index];

                next_x_vals = collision_free_trajs[index].x_vals;
                next_y_vals = collision_free_trajs[index].y_vals;   
              }else{
                printf("repeating last path");
                for(int i = 0; i < path_size; ++i){
                  next_x_vals.push_back(previous_path_x[i]);
                  next_y_vals.push_back(previous_path_y[i]);
                }
              }
             
            }else{
              for(int i = 0; i < path_size; ++i){
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
              }
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
















































































