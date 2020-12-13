#include "dubins.h"
#include <math.h>

int main(){

	// Initial data
	double x0 = 0;
	double y0 = 0;
	double th0 = -2/3*M_PI;
	double xf = 4;
	double yf = 0;
	double thf = M_PI/3.0;
	double kmax = 3.0;
	dubinsCurve curve;
	dubins_shortest_path(curve,x0,y0,th0,xf,yf,thf,kmax);

	return 0;
}
