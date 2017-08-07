#include "helpers.h"
#include <numeric>
#include <functional>


// For converting back and forth between radians and degrees.
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double sprod(const dvector &v1, const dvector &v2)
{
  return inner_product(v1.cbegin(), v1.cend(), v2.cbegin(), 0.);
}

double lenvec(const dvector &v)
{
  return sqrt(sprod(v, v));
}

dvector binop(const dvector & v1, const dvector & v2, std::function<double(double, double)> r)
{
  dvector result(v1.size());
  for (int i(0); i<v1.size(); ++i)
    result[i]=r(v1[i], v2[i]);
  return result;
}

dvector unaryop(const dvector & v, std::function<double(double)> r)
{
  dvector result(v.size());
  for (int i(0); i<v.size(); ++i)
    result[i]=r(v[i]);
  return result;
}

dvector sumvec(const dvector & v1, const dvector & v2)
{
  return binop(v1, v2, [](double x, double y){ return x+y; });
}

dvector diffvec(const dvector & v1, const dvector & v2)
{
  return binop(v1, v2, [](double x, double y){ return x-y; });
}

dvector scalevec(const dvector & v, double scale)
{
  return unaryop(v, [scale](double x){return x*scale; });
}

dvector normalvec(const dvector & v)
{
  double invlength(1./lenvec(v));
  return scalevec(v, invlength);
}

dvector projection(const dvector & base, const dvector & v)
{
  return scalevec(base, sprod(v, base)/sprod(base, base));
}

double projectionlength(const dvector & base, const dvector & v)
{
  return sprod(base, v)/lenvec(base);
}

double dabs(double x)
{
  return x>=0 ? x : -x;
}

double dmin(double x1, double x2)
{
  return x1<=x2 ? x1 : x2;
}

double dmax(double x1, double x2)
{
  return x1>=x2 ? x1 : x2;
}
