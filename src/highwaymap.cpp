#include <string>
#include <sstream>
#include <math.h>
#include "helpers.h"
#include "highwaymap.h"


HighwayMap::HighwayMap()
{
}

HighwayMap::HighwayMap(std::ifstream &in_map)
{
  std::string line;
  while (getline(in_map, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  /* fit splines to x and y as a function of s;
   * take care that the sline corresponds to a closed loop,
   * therefore add the first element again (with max_s)
   */
  std::vector<double> ext_s(map_waypoints_s);
  ext_s.push_back(max_s);
  std::vector<double> ext_x(map_waypoints_x);
  ext_x.push_back(map_waypoints_x[0]);
  std::vector<double> ext_y(map_waypoints_y);
  ext_y.push_back(map_waypoints_y[0]);

  map_splines_x.set_points(ext_s, ext_x);
  map_splines_y.set_points(ext_s, ext_y);
}

double HighwayMap::distance(double x1, double y1, double x2, double y2) const
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int HighwayMap::ClosestWaypoint(double x, double y) const
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < map_waypoints_x.size(); i++)
  {
    double map_x = map_waypoints_x[i];
    double map_y = map_waypoints_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int HighwayMap::NextWaypoint(double x, double y, double theta) const
{

  int closestWaypoint = ClosestWaypoint(x,y);

  double map_x = map_waypoints_x[closestWaypoint];
  double map_y = map_waypoints_y[closestWaypoint];

  double heading = atan2( (map_y-y),(map_x-x) );

  double angle = abs(theta-heading);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  }

  return closestWaypoint;

}

std::vector<double> HighwayMap::getFrenet(double x, double y, double theta) const
{
  int next_wp = NextWaypoint(x,y, theta);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = map_waypoints_x.size()-1;
  }

  double n_x = map_waypoints_x[next_wp]-map_waypoints_x[prev_wp];
  double n_y = map_waypoints_y[next_wp]-map_waypoints_y[prev_wp];
  double x_x = x - map_waypoints_x[prev_wp];
  double x_y = y - map_waypoints_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-map_waypoints_x[prev_wp];
  double center_y = 2000-map_waypoints_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(map_waypoints_x[i],map_waypoints_y[i],
                         map_waypoints_x[i+1],map_waypoints_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

std::vector<double> HighwayMap::getXY(double s, double d) const
{
  int prev_wp = -1;

  while(s > map_waypoints_s[prev_wp+1] && (prev_wp < (int)(map_waypoints_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%map_waypoints_x.size();

  double heading = atan2((map_waypoints_y[wp2]-map_waypoints_y[prev_wp]),(map_waypoints_x[wp2]-map_waypoints_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-map_waypoints_s[prev_wp]);

  double seg_x = map_waypoints_x[prev_wp]+seg_s*cos(heading);
  double seg_y = map_waypoints_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}

std::vector<double> HighwayMap::getSmoothXY(double s, double d) const
{
  // center of the street
  std::vector<double> center({map_splines_x(s), map_splines_y(s)});
  // tangent vector
  std::vector<double> tangent({map_splines_x.derivative(s),
                               map_splines_y.derivative(s)});
  // normalize tangent vector
  double l(sqrt(tangent[0]*tangent[0]+tangent[1]*tangent[1]));
  tangent[0]/=l;
  tangent[1]/=l;
  // rotate by pi/2 and stretch by d
  std::vector<double> normal({tangent[1]*d, -tangent[0]*d});
  // compute position
  std::vector<double> position({center[0]+normal[0], center[1]+normal[1]});

  return position;
}

Waypoint HighwayMap::operator[](int i) const
{
   int k(i>=0 ? i % map_waypoints_dx.size()
              : map_waypoints_dx.size() - ((-i-1) % (map_waypoints_dx.size())+1));
   return Waypoint(map_waypoints_x[k],
                   map_waypoints_y[k],
                   map_waypoints_s[k],
                   map_waypoints_dx[k],
                   map_waypoints_dy[k]);
}

