#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <vector>

typedef std::vector<double> dvector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x);
double rad2deg(double x);

#endif // HELPERS_H

