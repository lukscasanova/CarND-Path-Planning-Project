#ifndef MAP_SPLINE_HEADER
#define MAP_SPLINE_HEADER
#include "spline.h"
#include <vector>

using namespace std;
class MapSpline {
public:
	vector<double> getXY(double s, double d)
	{

	  double x = wp_x_spline(s);
	  double y = wp_y_spline(s);
	  double dx = wp_dx_spline(s);
	  double dy = wp_dy_spline(s);

	  x += d*dx;
	  y += d*dy;

	  return {x,y};

	}

	
	void create(double s,
		vector<double> map_waypoints_s,
		vector<double> map_waypoints_x,
		vector<double> map_waypoints_y,
		vector<double> map_waypoints_dx,
		vector<double> map_waypoints_dy){

		int base_wp = getPrevWp(s, map_waypoints_s)-1;

	    vector<double> wp_s, wp_x, wp_y, wp_dx, wp_dy;
	    for(int i = -1; i <= 20; ++i){
	      wp_s.push_back(map_waypoints_s[base_wp+i]);
	      wp_x.push_back(map_waypoints_x[base_wp+i]);
	      wp_y.push_back(map_waypoints_y[base_wp+i]);
	      wp_dx.push_back(map_waypoints_dx[base_wp+i]);
	      wp_dy.push_back(map_waypoints_dy[base_wp+i]);
	    }

        wp_x_spline.set_points(wp_s, wp_x);
        wp_y_spline.set_points(wp_s, wp_y);
        wp_dx_spline.set_points(wp_s, wp_dx);
        wp_dy_spline.set_points(wp_s, wp_dy);
	}

private:

	int getPrevWp(double s, vector<double> maps_s){
		int prev_wp = -1;

		while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
		{
			prev_wp++;
		}
		return prev_wp;
	}

	tk::spline wp_x_spline;
	tk::spline wp_y_spline;
	tk::spline wp_dx_spline;
	tk::spline wp_dy_spline;

};

#endif