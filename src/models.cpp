#include "models.h"
#include "helpers.h"

/*
 * Moving Point model
 */

Models::MovingPoint::Properties::Properties()
{ }

Models::MovingPoint::Properties::Properties(const Models::MovingPoint::Properties &p)
{ }

Models::MovingPoint::State::State(double x_, double y_, double vx_, double vy_)
  : v({x_, y_, vx_, vy_})
{ }

Models::MovingPoint::State::State(const dvector &v_)
  : v(v_)
{ }

Models::MovingPoint::State::State(const Models::MovingPoint::State &s)
  : v(s.v)
{ }

Models::MovingPoint::State::State(const Models::MovingPoint::State &s, const Models::MovingPoint::Update &u)
  : v(sumvec(s.v, u.v))
{ }

double Models::MovingPoint::State::x() const
{
  return v[0];
}

double Models::MovingPoint::State::y() const
{
  return v[1];
}

double Models::MovingPoint::State::vx() const
{
  return v[2];
}

double Models::MovingPoint::State::vy() const
{
  return v[3];
}

Models::MovingPoint::Actuation::Actuation(double ax_, double ay_)
  : v({ax_, ay_})
{ }

Models::MovingPoint::Actuation::Actuation(const dvector &v_)
  : v(v_)
{ }

double Models::MovingPoint::Actuation::ax() const
{
  return v[0];
}

double Models::MovingPoint::Actuation::ay() const
{
  return v[1];
}

Models::MovingPoint::Update::Update(const Properties & p, const State &s, const Actuation &a, double delta_t)
  : v({delta_t * s.vx(),
       delta_t * s.vy(),
       delta_t * a.ax(),
       delta_t * a.ay()})
{ }

Models::MovingPoint::MovingPoint(double x_, double y_, double vx_, double vy_)
  : state(x_, y_, vx_, vy_), properties()
{ }

Models::MovingPoint::MovingPoint(const Models::MovingPoint &mp)
  : state(mp.state), properties(mp.properties)
{ }

Models::MovingPoint::State Models::MovingPoint::advance(const Actuation &a, double delta_t)
{
  Models::MovingPoint::Update update(properties, state, a, delta_t);
  Models::MovingPoint::State newstate(state, update);
  return newstate;
}


void Models::MovingPoint::advance_state(const Models::MovingPoint::Actuation & a, double delta_t)
{
  Models::MovingPoint::State newstate(advance(a, delta_t));
  set_state(newstate);
}

void Models::MovingPoint::set_state(const Models::MovingPoint::State &newstate)
{
  state=newstate;
}

/*
 * Bicycle Model
 */

Models::Bicycle::Properties::Properties(double l)
  : L(l)
{ }

Models::Bicycle::Properties::Properties(const Models::Bicycle::Properties &p)
  : L(p.L)
{ }

Models::Bicycle::State::State(double x_, double y_, double theta_, double speed_)
  : v({x_, y_, theta_, speed_})
{ }

Models::Bicycle::State::State(const dvector &v_)
  : v(v_)
{ }

Models::Bicycle::State::State(const Models::Bicycle::State &s)
  : v(s.v)
{ }

Models::Bicycle::State::State(const Models::Bicycle::State &s, const Models::Bicycle::Update &u)
  : v(sumvec(s.v, u.v))
{ }

double Models::Bicycle::State::x() const
{
  return v[0];
}

double Models::Bicycle::State::y() const
{
  return v[1];
}

double Models::Bicycle::State::theta() const
{
  return v[2];
}

double Models::Bicycle::State::speed() const
{
  return v[3];
}

double Models::Bicycle::State::vx() const
{
  return speed()*cos(theta());
}

double Models::Bicycle::State::vy() const
{
  return speed()*sin(theta());
}

Models::MovingPoint Models::Bicycle::State::moving_point() const
{
  return Models::MovingPoint(x(), y(), vx(), vy());
}

Models::Bicycle::Actuation::Actuation(double a, double delta)
  : v({a, delta})
{ }

Models::Bicycle::Actuation::Actuation(const dvector &v_)
  : v(v_)
{ }

double Models::Bicycle::Actuation::a() const
{
  return v[0];
}

double Models::Bicycle::Actuation::delta() const
{
  return v[1];
}

Models::Bicycle::Update::Update(const Properties & p, const State &s, const Models::Bicycle::Actuation &a, double delta_t)
  : v({delta_t * s.vx(),
       delta_t * s.vy(),
       delta_t * s.speed()/p.L*tan(a.delta()),
       delta_t * a.a()})
{ }


Models::Bicycle::Bicycle(double x_, double y_, double theta_, double speed_, double L_)
  : state(x_, y_, theta_, speed_), properties(L_)
{ }

Models::Bicycle::Bicycle(const Models::Bicycle &bike)
  : state(bike.state), properties(bike.properties)
{ }

Models::Bicycle::State Models::Bicycle::advance(const Actuation &a, double delta_t)
{
  Models::Bicycle::Update update(properties, state, a, delta_t);
  Models::Bicycle::State newstate(state, update);
  return newstate;
}


void Models::Bicycle::advance_state(const Models::Bicycle::Actuation & a, double delta_t)
{
  Models::Bicycle::State newstate(advance(a, delta_t));
  set_state(newstate);
}

void Models::Bicycle::set_state(const Models::Bicycle::State &newstate)
{
  state=newstate;
}
