#ifndef RECORDS_H
#define RECORDS_H

#include "data.h"
#include "detection.h"

/* storage for all records; keep record of all cars
 * that are within a distance s_range from the ego
 * car
 */
class Records
{
private:
  int rec_size;
public:
  std::map<int, Record> records;
  double s_range;
  Records(double s_range_);
  void update(double time_stamp, const Telemetry & telemetry);
};

#endif // RECORDS_H
