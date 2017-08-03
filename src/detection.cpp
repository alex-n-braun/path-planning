#include "detection.h"

Detection::Detection()
  : id(-1), v(6)
{ }

Detection::Detection(const json &detection)
  : id(detection[0]), v(6)
{
  for (int i(1); i<detection.size(); ++i)
  {
    v[i-1]=detection[i];
  }
}

Detection::Detection(const Detection & detection)
  : id(detection.id), v(detection.v)
{ }

double Detection::x() const
{
  return v[0];
}

double Detection::y() const
{
  return v[1];
}

dvector Detection::pos() const
{
  return {x(), y()};
}

double Detection::vx() const
{
  return v[2];
}

double Detection::vy() const
{
  return v[3];
}

dvector Detection::speed() const
{
  return {vx(), vy()};
}

double Detection::s() const
{
  return v[4];
}

double Detection::d() const
{
  return v[5];
}

std::ostream & operator<<(std::ostream &out, const Detection & d)
{
  out<<"id: "<<d.id<<", x: "<<d.x()<<", y: "<<d.y()<<", vx: "<<d.vx()<<", vy: "<<d.vy()<<", s: "<<d.s()<<", d: "<<d.d();
  return out;
}

