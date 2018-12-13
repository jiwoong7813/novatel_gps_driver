#ifndef __CONV_LLH_TO_XYZ__
#define __CONV_LLH_TO_XYZ__

#include <math.h>

class ConvLLHToXYZ {
private:
	double m_lat;
	double m_lon;
	double m_h;
	
	double m_x;
	double m_y;
	double m_z;
public:
	double get_lat(void) const;
	double get_lon(void) const;
	double get_h(void) const;

	double get_x(void) const;
	double get_y(void) const;
	double get_z(void) const;

	void set_llh(double lat, double lon, double h);
	void set_xyz(double x, double y, double z);

	void conv_llh_to_xyz(void);
};

#endif
