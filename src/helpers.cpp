#include "helpers.h"


// For converting back and forth between radians and degrees.
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double sprod(const dvector &v1, const dvector &v2)
{
  return (v1[0]*v2[0] + v1[1]*v2[1]);
}

double lenvec(const dvector &v)
{
  return sqrt(sprod(v, v));
}

dvector sumvec(const dvector & v1, const dvector & v2)
{
  return {v1[0]+v2[0], v1[1]+v2[1]};
}

dvector diffvec(const dvector & v1, const dvector & v2)
{
  return {v1[0]-v2[0], v1[1]-v2[1]};
}

dvector scalevec(const dvector & v, double scale)
{
  return {v[0]*scale, v[1]*scale};
}

dvector normalvec(const dvector & v)
{
  double invlength(1./lenvec(v));
  return scalevec(v, invlength);
}

double projectionlength(const dvector & base, const dvector & v)
{
  return sprod(base, v)/lenvec(base);
}

double dabs(double x)
{
  return x>=0 ? x : -x;
}
