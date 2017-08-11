#ifndef TRAJECTORY_HEADER
#define TRAJECTORY_HEADER

#include "car.h"
#include "map_spline.h"
#include <vector>
#include <iostream>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

vector<double> JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    
    MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
          3*T*T, 4*T*T*T,5*T*T*T*T,
          6*T, 12*T*T, 20*T*T*T;
    
  MatrixXd B = MatrixXd(3,1);     
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
          end[1]-(start[1]+start[2]*T),
          end[2]-start[2];
          
  MatrixXd Ai = A.inverse();
  
  MatrixXd C = Ai*B;
  
  vector <double> result = {start[0], start[1], .5*start[2]};
  for(int i = 0; i < C.size(); i++)
  {
      result.push_back(C.data()[i]);
  }
  
    return result;
    
}


double evaluate(std::vector<double> coeff, double t){
  
  double result = coeff[0];
  double t0 = t;
  for(size_t i = 1 ; i < coeff.size(); ++i){
    result += coeff[i]*t;
    t*=t0;
  }

  return result;
}

class Trajectory{
public:

	void createJerkMinimized(Car start, Car end, double time_interval, MapSpline ms){

		start.s = ms.correct_s(start.s);
		end.s = ms.correct_s(end.s);
		std::vector<double> start_s, start_d, end_s, end_d;
        start_s = {start.s, start.s_v, start.s_a};

        end_s 	= {end.s , end.s_v, end.s_a};

        start_d = {start.d, start.d_v, start.d_a};
        end_d   = {end.d , end.d_v, end.d_a};

        std::vector<double> coeff_s = JMT(start_s, end_s, time_interval);
        std::vector<double> coeff_d = JMT(start_d, end_d, time_interval);

        double delta_t = 0.02;
		
		s_vals.clear();
		d_vals.clear();

        for(int i = 0; i < (int) time_interval / delta_t ;++i){
        	double t = i * delta_t;

        	double s = evaluate(coeff_s, t);
        	double d = evaluate(coeff_d, t);

            s_vals.push_back(s);
            d_vals.push_back(d);

            auto xy = ms.getXY(s,d);
            x_vals.push_back(xy[0]);
            y_vals.push_back(xy[1]);
        }

        //debug
        // for (int i = 0; i < s_vals.size(); ++i){
        // 	cout << "s,d: " << s_vals[i] << ", " << d_vals[i] << endl;
        // }

	}

	void extendJerkMinimized(double curr_s, int stitchPoint, Car end, double time_interval, MapSpline ms){


		// check loop
		double s_correction = 0;
		if(s_vals[0]>end.s){
			if(!ms.transition){
				s_correction = -6945.554;
			}
		}

		if(s_vals.size() == 0){
			Car start;
			createJerkMinimized(start, end, time_interval, ms);
			return;
		}

		if(stitchPoint<2){
			stitchPoint=2;
		}

		int start_index=-1;
		curr_s = ms.correct_s(curr_s);
		while(curr_s>s_vals[++start_index]+s_correction);
		stitchPoint+=start_index;

		if(stitchPoint>s_vals.size()){
			stitchPoint=s_vals.size()-1;
		}

		double start_vel_s = (s_vals[stitchPoint] - s_vals[stitchPoint-1])/0.02;
		double start_vel_d = (d_vals[stitchPoint] - d_vals[stitchPoint-1])/0.02;

		double start_acc_s = (s_vals[stitchPoint] - s_vals[stitchPoint-1]*2 + s_vals[stitchPoint-2])/(0.0004);
		double start_acc_d = (d_vals[stitchPoint] - d_vals[stitchPoint-1]*2 + d_vals[stitchPoint-2])/(0.0004);


		double start_pos_s = s_vals[stitchPoint-1];
		double start_pos_d = d_vals[stitchPoint-1];

		
		start_pos_s += s_correction;
		end.s = ms.correct_s(end.s);

		std::vector<double> start_s, start_d, end_s, end_d;
        start_s = {start_pos_s, start_vel_s, start_acc_s};
        end_s 	= {end.s , end.s_v, end.s_a};

        start_d = {start_pos_d, start_vel_d, start_acc_d};
        end_d   = {end.d , end.d_v, end.d_a};

        std::vector<double> coeff_s = JMT(start_s, end_s, time_interval);
        std::vector<double> coeff_d = JMT(start_d, end_d, time_interval);

        double delta_t = 0.02;

        std::vector<double> new_s_vals, new_d_vals;
        std::vector<double> new_x_vals, new_y_vals;


        // Remove points already visited and copy over old path until stitch point
        for(int i = start_index; i < stitchPoint-1; ++i){
        	double s = s_vals[i] + s_correction;
			double d = d_vals[i];


			new_x_vals.push_back(x_vals[i]);
			new_y_vals.push_back(y_vals[i]);

        	new_s_vals.push_back(ms.correct_s(s));
        	new_d_vals.push_back(d);


        }

        // Add points from new trajectory
        for(int i = 0; i < (int) time_interval/delta_t; ++i ){
        	double t = i*delta_t;

        	double s = evaluate(coeff_s, t);
        	double d = evaluate(coeff_d, t);

        	new_s_vals.push_back(s);
        	new_d_vals.push_back(d);

        	auto xy = ms.getXY(s,d);

        	new_x_vals.push_back(xy[0]);
			new_y_vals.push_back(xy[1]);
        }

        // Create single spline from old and new parts
        tk::spline x_spline, y_spline;
        std::vector<double> x_points, y_points, t_points;

        for(int i =0; i < new_x_vals.size(); i+=10){
        	t_points.push_back(i);
        	x_points.push_back(new_x_vals[i]);
        	y_points.push_back(new_y_vals[i]);
        }

        x_spline.set_points(t_points, x_points);
        y_spline.set_points(t_points, y_points);

		x_vals.clear();
		y_vals.clear();    

		// Create final XY trajectory from spline
        for(int i =0; i < new_x_vals.size(); i++){
        	x_vals.push_back(x_spline(i));
        	y_vals.push_back(y_spline(i));

        }

        d_vals = new_d_vals;
        s_vals = new_s_vals;

	}

	void print(){
		for(int i =0; i < s_vals.size(); ++i){
			printf("x,y,s,d: %lf, %lf, %lf, %lf\n", x_vals[i], y_vals[i], s_vals[i], d_vals[i]);
		}
	}
	
	std::vector<double> s_vals;
	std::vector<double> d_vals;

	std::vector<double> x_vals;
	std::vector<double> y_vals;
private:

	double start_s;
	double start_d;
};

#endif /* TRAJECTORY_HEADER */
