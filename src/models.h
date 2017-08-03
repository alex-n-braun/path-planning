#ifndef MODELS_H
#define MODELS_H

#include "helpers.h"

namespace Models
{

class MovingPoint {
public:
  class Properties {
  public:
    Properties();
    Properties(const Properties & p);
  };
  class Update;
  class State {
  public:
    dvector v;  // x, y, vx, vy
    State(double x_, double y_, double vx_, double vy_);
    State(const dvector & v_);
    State(const State & s);
    State(const State & s, const Update & u);
    double x() const;
    double y() const;
    double vx() const;
    double vy() const;
  };
  class Actuation {
  public:
    dvector v; // acceleration ax, ay
    Actuation(double ax_, double ay_);
    Actuation(const dvector & v_);
    double ax() const;
    double ay() const;
  };
  class Update {
  public:
    dvector v; // delta of x, y, vx, vy given a delta time
    Update(const Properties & p, const State & s, const Actuation & a, double delta_t);
  };

  State state;
  Properties properties;

  MovingPoint(double x_, double y_, double vx_, double vy_);
  MovingPoint(const MovingPoint & mp);
  MovingPoint(const State & s_, const Properties & p_);

  void set_state(const State & newstate);
  State advance(const Actuation & a, double delta_t) const;
  void advance_state(const Actuation & a, double delta_t);
  MovingPoint advanced_vehicle(const Actuation & a, double delta_t) const;
};


class Bicycle {
public:
  class Properties {
  public:
    double L; // distance between axes
    Properties(double l);
    Properties(const Properties & p);
  };
  class Update;
  class State {
  public:
    dvector v;  // x, y, theta, v
    State(double x_, double y_, double theta_, double speed_);
    State(const dvector & v_);
    State(const State & s);
    State(const State & s, const Update & u);
    double x() const;
    double y() const;
    double theta() const;
    double speed() const;
    double vx() const;
    double vy() const;
    MovingPoint moving_point() const;
  };
  class Actuation {
  public:
    dvector v; // acceleration, steering angle: a, delta
    Actuation(double a, double delta);
    Actuation(const dvector & v_);
    double a() const;
    double delta() const;
  };
  class Update {
  public:
    dvector v; // delta of x, y, theta, v given a delta time
    Update(const Properties & p, const State & s, const Actuation & a, double delta_t);
  };

  State state;
  Properties properties;

  Bicycle(double x_, double y_, double theta_, double speed_, double L_ = 4.); // default length 4 m
  Bicycle(const Bicycle & bike);
  Bicycle(const State & s_, const Properties & p_);

  void set_state(const State & newstate);
  State advance(const Actuation & a, double delta_t) const;
  void advance_state(const Actuation & a, double delta_t);
  Bicycle advanced_vehicle(const Actuation & a, double delta_t) const;
};


}

#endif // MODELS_H
