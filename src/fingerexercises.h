#ifndef FINGEREXERCISES_H
#define FINGEREXERCISES_H

#include "json.hpp"
#include "data.h"
#include "highwaymap.h"
#include "records.h"
#include "predictions.h"

Response fe_constspeed(const Telemetry & t);
Response fe_circle(const Telemetry & t);
Response fe_waypoints(const Telemetry & t, const HighwayMap & m);
Response fe_rightmostlane(const Telemetry &t, const HighwayMap &m);
Response fe_smooth_rightmostlane(const Telemetry &t, const HighwayMap &m);
Response fe_even_more_smooth_rightmostlane(const Telemetry &t, const HighwayMap &m);
Response fe_rightmostlane_constspeed(const Telemetry &t, const HighwayMap &m);
Response fe_rightmostlane_constdist(const Telemetry &t, const HighwayMap &m, const Records &records, const Predictions::Predictions & predictions);
Response fe_minjerk(const Telemetry &t, const HighwayMap &m, const Records &records, const Predictions::Predictions & predictions);
Response fe_evenmore_minjerk_old(const Telemetry &t, const HighwayMap &m, const Records &records, const Predictions::Predictions & predictions);
Response fe_evenmore_minjerk(const Telemetry &t, const HighwayMap &m, const Records &records, const Predictions::Predictions & predictions);

#endif // FINGEREXERCISES_H
