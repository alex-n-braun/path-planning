#ifndef FINGEREXERCISES_H
#define FINGEREXERCISES_H

#include "json.hpp"
#include "data.h"
#include "highwaymap.h"

Response fe_constspeed(const Telemetry & t);
Response fe_circle(const Telemetry & t);
Response fe_waypoints(const Telemetry & t, const HighwayMap & m);
Response fe_rightmostlane(const Telemetry &t, const HighwayMap &m);
Response fe_smooth_rightmostlane(const Telemetry &t, const HighwayMap &m);

#endif // FINGEREXERCISES_H
