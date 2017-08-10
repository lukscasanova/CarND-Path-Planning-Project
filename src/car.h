#ifndef CAR_HEADER
#define CAR_HEADER

#include <iostream>

class Car{
public:
	Car(){
		has_last=false;
		s=0;
		s_v=0;
		s_a=0;

		d=0;
		d_v=0;
		d_a=0;

		delta_t=0.02;
	}

	void updateState(double curr_s, double curr_d){
		if(s!=0){
			has_last = true;
		}else{
			s = curr_s;
			d = curr_d;
			return;
		}
		double last_s_v = s_v;
		last_s = s;
		double last_d_v = d_v;
		last_d = d;

		s = curr_s;
		d = curr_d;

		s_v = (s - last_s)/delta_t;
		d_v = (d - last_d)/delta_t;

		s_a = (s_v - last_s_v)/delta_t;
		d_a = (d_v - last_d_v)/delta_t;
	}

	void printState(){
		std::cout << "s: " << s << ", s_v:" << s_v << ", s_a:" << s_a << std::endl;
		std::cout << "d: " << d << ", d_v:" << d_v << ", d_a:" << d_a << std::endl;
	}

	double last_s;
	double last_d;

	double s;
	double s_v;
	double s_a;

	double d;
	double d_v;
	double d_a;

	double x;
	double y;
	double x_v;
	double y_v;

	double delta_t;

	bool has_last;
private:


};


#endif /* CAR_HEADER */