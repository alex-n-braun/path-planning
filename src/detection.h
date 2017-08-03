#ifndef DETECTION_H
#define DETECTION_H

#include <iostream>
#include "helpers.h"
#include "json.hpp"

using json = nlohmann::json;

class Detection
{
public:
  int id;
  dvector v;

  Detection();
  Detection(const json & detection);
  Detection(const Detection & detection);
  double x() const;
  double y() const;
  dvector pos() const;
  double vx() const;
  double vy() const;
  dvector speed() const;
  double s() const;
  double d() const;
};

std::ostream & operator<<(std::ostream & out, const Detection & d);

#endif // DETECTION_H
