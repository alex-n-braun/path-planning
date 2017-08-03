#ifndef DETECTION_H
#define DETECTION_H

#include <iostream>
#include <utility>
#include <vector>
#include <map>
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
  Detection(int id_, const dvector & v_);
  double x() const;
  double y() const;
  dvector pos() const;
  double vx() const;
  double vy() const;
  dvector speed() const;
  double s() const;
  double d() const;
};

// keep record of the detections of a specific car
class Record
{
private:
  int current;
  bool full;
  std::vector< std::pair<double, dvector> > v;
public:
  int id;
  Record();
  Record(int id_, int size_);
  Record(const Record & rec);
  void record(double time_stamp, const Detection & d);  // add a detection to the record
  std::pair<double, Detection> operator[](int i) const; // get (time_stamp, detection) with index i
  std::pair<double, Detection> get_last() const;        // get most recent (time_stamp, detection)
  int size() const;   // storage size
  int count() const;  // num elements stored
  bool is_full() const;
  friend std::ostream & operator<<(std::ostream & out, const Record & r);
};

std::ostream & operator<<(std::ostream & out, const Detection & d);

#endif // DETECTION_H
