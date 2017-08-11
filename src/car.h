#ifndef CAR_HEADER
#define CAR_HEADER

#include <iostream>

class Car{
public:
	Car(){
		s=0;
		s_v=0;
		s_a=0;

		d=0;
		d_v=0;
		d_a=0;

		x=0;
		y=0;
		x_v=0;
		y_v=0;

	}

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

private:


};


#endif /* CAR_HEADER */