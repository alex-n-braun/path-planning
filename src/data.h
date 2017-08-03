#ifndef DATA_H
#define DATA_H


#include <vector>
#include <map>
#include "json.hpp"
#include "detection.h"

using namespace std;
using json = nlohmann::json;

struct Response {
  vector<double> next_x_vals;
  vector<double> next_y_vals;
};

class Telemetry {
public:
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  // Previous path data given to the Planner
  const json & previous_path_x;
  const json & previous_path_y;
  // Previous path's end s and d values
  double end_path_s;
  double end_path_d;
  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  std::map<int, Detection> sensor_fusion;
  Telemetry(const json & j)
    : car_x(j["x"]), car_y(j["y"]), car_s(j["s"]), car_d(j["d"]),
      car_yaw(j["yaw"]), car_speed(j["speed"]),
      previous_path_x(j["previous_path_x"]),
      previous_path_y(j["previous_path_y"]),
      end_path_s(j["end_path_s"]), end_path_d(j["end_path_d"])
  {
    for (int i(0); i<j["sensor_fusion"].size(); ++i)
    {
      sensor_fusion[j["sensor_fusion"][i][0]] = Detection(j["sensor_fusion"][i]);
    }
  }
};


#endif // DATA_H

