#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <vector>

typedef std::vector<double> dvector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x);
double rad2deg(double x);

double dabs(double x);
double dmin(double x1, double x2);
double dmax(double x1, double x2);
double dsort(double & x1, double & x2);

// scalar product for 2d vector
double sprod(const dvector & v1, const dvector & v2);
// length of 2d vector
double lenvec(const dvector & v);
// sum & diff of two vectors
dvector sumvec(const dvector & v1, const dvector & v2);
dvector diffvec(const dvector & v1, const dvector & v2);
// scale vector
dvector scalevec(const dvector & v, double scale);
// normalized vector
dvector normalvec(const dvector & v);
// compute the projection vector (projection of v onto base)
dvector projection(const dvector & base, const dvector & v);
// compute the length of the prejection of v on base
double projectionlength(const dvector & base, const dvector & v);

#endif // HELPERS_H

