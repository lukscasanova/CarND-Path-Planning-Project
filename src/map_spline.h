#ifndef MAP_SPLINE_HEADER
#define MAP_SPLINE_HEADER
#include "spline.h"
#include <vector>

using namespace std;
class MapSpline {
public:

	/**
	 * @brief      Gets Cartesian coordinates from Frenet coordinates
	 *
	 * @param[in]  s     forward position on the road
	 * @param[in]  d     lateral position on the road
	 *
	 * @return     The xy.
	 */
	vector<double> getXY(double s, double d)
	{
		s= correct_s(s);
		double x = wp_x_spline(s);
		double y = wp_y_spline(s);
		double dx = wp_dx_spline(s);
		double dy = wp_dy_spline(s);

		x += d*dx;
		y += d*dy;

		return {x,y};
	}

	/**
	 * @brief      Applies a correction to s value if we are in transition area
	 *
	 * @param[in]  s     s value
	 *
	 * @return     corrected s value
	 */
	double correct_s(double s){
		if(transition && s < 3000){
	    		s+=6945.554;
		}
		return s;
	}

	/**
	 * @brief      Creates the spline
	 *
	 * @param[in]  s                 current s position
	 * @param[in]  map_waypoints_s   The map waypoints s
	 * @param[in]  map_waypoints_x   The map waypoints x
	 * @param[in]  map_waypoints_y   The map waypoints y
	 * @param[in]  map_waypoints_dx  The map waypoints dx
	 * @param[in]  map_waypoints_dy  The map waypoints dy
	 */
	void create(double s,
		vector<double> map_waypoints_s,
		vector<double> map_waypoints_x,
		vector<double> map_waypoints_y,
		vector<double> map_waypoints_dx,
		vector<double> map_waypoints_dy){

		transition = false;

		int size = map_waypoints_s.size();

		int base_wp = (getPrevWp(s, map_waypoints_s)-3 + size)%size;
	    vector<double> wp_s, wp_x, wp_y, wp_dx, wp_dy;
	    for(int i = 0; i <= 10; ++i){
	    	double s = map_waypoints_s[(base_wp+i)%size];
	    	if(base_wp+i>=size){
	    		s+=6945.554;
	    		transition=true;
	    	}
	    	wp_s.push_back(s);
			wp_x.push_back(map_waypoints_x[(base_wp+i)%size]);
			wp_y.push_back(map_waypoints_y[(base_wp+i)%size]);
			wp_dx.push_back(map_waypoints_dx[(base_wp+i)%size]);
			wp_dy.push_back(map_waypoints_dy[(base_wp+i)%size]);

		}
        wp_x_spline.set_points(wp_s, wp_x);
        wp_y_spline.set_points(wp_s, wp_y);
        wp_dx_spline.set_points(wp_s, wp_dx);
        wp_dy_spline.set_points(wp_s, wp_dy);
	}

	bool transition;


private:

	/**
	 * @brief      Gets the previous waypoint
	 *
	 * @param[in]  s       s value
	 * @param[in]  maps_s  The waypoints s values
	 *
	 * @return     The previous wp.
	 */
	int getPrevWp(double s, vector<double> maps_s){
		int prev_wp = -1;
		int size = maps_s.size();
		while(s > maps_s[prev_wp+1] && prev_wp < size-1)
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