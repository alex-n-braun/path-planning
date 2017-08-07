#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "data.h"
#include "highwaymap.h"
#include "fingerexercises.h"
#include "helpers.h"
#include "records.h"
#include "predictions.h"
#include "ego.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Waypoint map to read from
  string map_file_ = "../path-planning/data/highway_map.csv";

  ifstream in_map_(map_file_.c_str(), ifstream::in);
  HighwayMap highway_map(in_map_);

  chrono::system_clock::time_point start_time = chrono::system_clock::now();
  double old_time_stamp(0);

  // recording sensor detections of cars in a certain range in s direction
  Records records(250. /* range in m */);

  // ego object
  Ego ego(highway_map);

  h.onMessage([&ego, &highway_map, &records, start_time, &old_time_stamp]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);
//      std::cout<<s<<std::endl;

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object

            auto duration(chrono::system_clock::now()-start_time);
            auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

            double time_stamp(double(millis)*1e-3); // time stamp in seconds

            Telemetry telemetry(j[1]);

            std::cout<<"Delta time: "<<time_stamp-old_time_stamp
                    <<", s: "<<telemetry.car_s
                    <<std::endl;

            // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

            // update the records of the car's telemetry data
            records.update(time_stamp, telemetry);

            // generate predictions of the other car's behavior
            Predictions::Predictions predictions(records, highway_map, 1.0, 12);

            Response r;

            if (time_stamp>-1.)
            {
//            /* finger exercises */
//            //  r=fe_constspeed(telemetry);
//            //  r=fe_circle(telemetry);
//            //  r=fe_waypoints(telemetry, highway_map);
//            //  r=fe_rightmostlane(telemetry, highway_map);
//            //  r=fe_smooth_rightmostlane(telemetry, highway_map);
//            //  r=fe_even_more_smooth_rightmostlane(telemetry, highway_map);
//            //  r=fe_rightmostlane_constspeed(telemetry, highway_map);
//            //  r=fe_rightmostlane_constdist(telemetry, highway_map, records, predictions);
//            //  r=fe_minjerk(telemetry, highway_map, records, predictions);
//              r=fe_evenmore_minjerk(telemetry, highway_map, records, predictions);
//            /* end of finger exercises */
              r = ego.path(telemetry, predictions);
            }
            else if (time_stamp<0.5)
            {
              dvector xy=highway_map.getSmoothXY(HighwayMap::max_s-200., 2.+4.+3.5);
              std::cout<<"start-s: "<<HighwayMap::max_s-200.<<" == "<<highway_map.getSmoothFrenet(xy)[0]<<std::endl;
              double x(xy[0]);
              double y(xy[1]);
              r.next_x_vals={x, x, x, x};
              r.next_y_vals={y, y, y, y};
            }


            json msgJson;
            msgJson["next_x"] = r.next_x_vals;
            msgJson["next_y"] = r.next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

//            this_thread::sleep_for(chrono::milliseconds(500));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            old_time_stamp = time_stamp;
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h, &ego](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    ego.re_init();
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

