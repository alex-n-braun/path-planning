#ifndef EGO_H
#define EGO_H

#include "data.h"
#include "predictions.h"
#include "records.h"
#include "highwaymap.h"

class Ego
{
private:
  Trajectory::MinJerk * min_jerk;
  int init;
public:
  Ego();
  ~Ego();
  void re_init();
  Response path(const Telemetry &t, const HighwayMap &m,
                const Records & records, const Predictions::Predictions & predictions);
};

#endif // EGO_H
