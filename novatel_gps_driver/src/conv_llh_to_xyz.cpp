#include <novatel_gps_driver/conv_llh_to_xyz.hpp>

double ConvLLHToXYZ::get_lat(void) const {
	return m_lat;
}

double ConvLLHToXYZ::get_lon(void) const {
	return m_lon;
}

double ConvLLHToXYZ::get_h(void) const {
	return m_h;
}

double ConvLLHToXYZ::get_x(void) const {
	return m_x;
}

double ConvLLHToXYZ::get_y(void) const {
	return m_y;
}

double ConvLLHToXYZ::get_z(void) const {
	return m_z;
}

void ConvLLHToXYZ::set_llh(double lat, double lon, double h) {
	m_lat = lat;
	m_lon = lon;
	m_h = h;
}

void ConvLLHToXYZ::set_xyz(double x, double y, double z) {
	m_x = x;
	m_y = y;
	m_z = z;
}

void ConvLLHToXYZ::conv_llh_to_xyz(void) {
	
	const double lat0 = 38.0;
	const double lon0 = 129.0;

	const double a = 6378137.0;
	const double f = 1.0 / 298.2572221010;
	const double b = a * (1.0-f);

	const double k0 = 1.0;

	const double dx = 600000.0;
	const double dy = 200000.0;

	const double phi = get_lat()*M_PI / 180.0;
	const double phi0 = 38.0*M_PI / 180.0;
	const double lambda = get_lon()*M_PI / 180.0;
	const double lambda0 = 129.0*M_PI / 180.0;

	double e1;
	double e2;
	double T;
	double C;
	double A;
	double N;
	double M;
	double M0;
	double YE, XN;

	e1 = (pow(a, 2)-pow(b, 2)) / pow(a, 2);
	e2 = (pow(a, 2)-pow(b, 2)) / pow(b, 2);

	T = pow(tan(phi), 2);
	C = e1 / (1.0-e1) * pow(cos(phi), 2);
	A = (lambda-lambda0) * cos(phi);
	N = a / sqrt(1.0-e1*pow(sin(phi), 2));
	M = a * ((1.0 - e1/4.0 - 3.0*pow(e1, 2)/64.0 - 5.0*pow(e1, 3)/256.0)*phi
		- (3.0*e1/8.0 + 3.0*pow(e1, 2)/32.0 + 45.0*pow(e1, 3)/1024.0)*sin(2.0*phi)
		+ (15.0*pow(e1, 2)/256.0 + 45.0*pow(e1, 3)/1024.0)*sin(4.0*phi)
		- 35.0*pow(e1, 3)/3072.0*sin(6.0*phi));
	M0 = a * ((1.0 - e1/4.0 - 3.0*pow(e1, 2)/64.0 - 5.0*pow(e1, 3)/256.0)*phi0
		- (3.0*e1/8.0 + 3.0*pow(e1, 2)/32.0 + 45.0*pow(e1, 3)/1024.0)*sin(2.0*phi0)
		+ (15.0*pow(e1, 2)/256.0 + 45.0*pow(e1, 3)/1024.0)*sin(4.0*phi0)
		- 35.0*pow(e1, 3)/3072.0*sin(6.0*phi0));

	YE = dy + k0 * N * (A
		+ pow(A, 3)/6.0*(1.0-T+C)
		+ pow(A, 5)/120.0*(5.0-18.0*T+pow(T, 2)+72.0*C-58.0*e2));

	XN = dx + k0 * (M - M0
		+ N*tan(phi)
		* (pow(A, 2)/2.0 + pow(A, 4)/24.0*(5.0-T+9.0*C+4.0*pow(C, 2))
		+ pow(A, 6)/720.0*(61.0-58.0*T+pow(T, 2)+600.0*C-330.0*e2)));

	set_xyz(XN, YE, 0.0);
}
