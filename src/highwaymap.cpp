#include <math.h>
#include <string>
#include <sstream>
#include <iostream>
#include "helpers.h"
#include "highwaymap.h"

const double HighwayMap::max_s(6945.554);

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
  ext_s.insert(ext_s.begin(), map_waypoints_s[map_waypoints_s.size()-1]-max_s);
  ext_s.push_back(max_s);
  ext_s.push_back(max_s+map_waypoints_s[1]);
  std::vector<double> ext_x(map_waypoints_x);
  ext_x.insert(ext_x.begin(), map_waypoints_x[map_waypoints_x.size()-1]);
  ext_x.push_back(map_waypoints_x[0]);
  ext_x.push_back(map_waypoints_x[1]);
  std::vector<double> ext_y(map_waypoints_y);
  ext_y.insert(ext_y.begin(), map_waypoints_y[map_waypoints_y.size()-1]);
  ext_y.push_back(map_waypoints_y[0]);
  ext_y.push_back(map_waypoints_y[1]);

  map_trajectory = Trajectory::Spline(ext_s, ext_x, ext_y);
}

double HighwayMap::distance(double x1, double y1, double x2, double y2) const
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

double HighwayMap::distance_s(double s1, double s2)
{
  double dist(s2-s1);
  /* loop is closed; even if s2 is larger then s1, s2 could be
   * "behind" s1
   */
  dist = (dist>0.5*max_s ? dist-max_s : dist);
  dist = (dist<-0.5*max_s ? dist+max_s : dist);
  return dist;
}

bool HighwayMap::is_ahead(double s1, double s2)
// is s2 ahead of s1? not simply s2>s1, take loop into account
{
  return HighwayMap::distance_s(s1, s2) > 0;
}

double HighwayMap::range_s(double s)
{
  while (s>HighwayMap::max_s) s-=HighwayMap::max_s;
  while (s<0) s+=HighwayMap::max_s;
  return s;
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

dvector HighwayMap::getFrenet(double x, double y, double theta) const
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

/* intention: provide a better solution that is exact in the range
 * of a given error.
 * We achieve this with the help of an iterative solver with maximum
 * number of iterations.
 */
//double maxiterations__(0);
dvector HighwayMap::getSmoothFrenet(double x, double y) const
{
  // find closest waypoint
  int i_c_wp = ClosestWaypoint(x,y);

  // and get s-value for this waypoint. this serves as initial value.
  double s(map_waypoints_s[i_c_wp]);

  int i(0);
  const int maxit(6); // maximum number of iterations
  const double maxerror(1e-3); // maximum error in unit [m]; 1e-3m = 1mm
  double proj;
  do
  {
    dvector c_xy(map_trajectory(s)); // x and y for given s, while d=0 (center of the road, c_xy)
    dvector tangent(map_trajectory.tangent(s)); // tangent in that point
    dvector delta(diffvec(dvector({x,y}), c_xy)); // delta vector from c_xy to {x,y}
    proj = projectionlength(tangent, delta); // length of projection of {x,y} on tangent
    /* the signed length of the projection vector onto the tangent vector for
     * the current value of s. This is the approximate distance from the actual value
     * of s to the one we are looking for. In fact, if the road goes straight,
     * proj holds the exact value we need to correct our s value for.
     */
    s+=proj;
    ++i;
  } while ((i<maxit) & (dabs(proj)>maxerror));
  dvector c_xy(map_trajectory(s));
  double d(lenvec(diffvec(dvector({x,y}), c_xy)));

//  { // for debugging
//    maxiterations__ = i>maxiterations__ ? i : maxiterations__;
//    std::cout<<"s: "<<s<<", d: "<<d<<", iterations: "<<i<<", max: "<<maxiterations__<<", error: "<<proj<<std::endl;
//  }
  return {s,d};
}

dvector HighwayMap::getSmoothFrenet(const dvector &xy) const
{
  return getSmoothFrenet(xy[0], xy[1]);
}

dvector HighwayMap::getXY(double s, double d) const
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

dvector HighwayMap::getSmoothXY(double s, double d) const
{
  // center of the street
  dvector center(map_trajectory(s));
  // normalized tangent vector
  dvector tangent(normalvec(map_trajectory.tangent(s)));

  // rotate by pi/2 and stretch by d
  dvector normal({tangent[1]*d, -tangent[0]*d});
  // compute position
  dvector position({center[0]+normal[0], center[1]+normal[1]});

  return position;
}

dvector HighwayMap::getSmoothXY(const dvector &sd) const
{
  return {getSmoothXY(sd[0], sd[1])};
}

int HighwayMap::getLane(double d) const
{
  return d/4;
}

double HighwayMap::get_d_from_lane(int lane) const
{
  return 2.+4.+3.6*double(lane-1);
}

dvector HighwayMap::tangent(double s) const
{
  return map_trajectory.tangent(s);
}

dvector HighwayMap::orthogonal(double s) const
{
  dvector t(tangent(s));
  return {t[1], -t[0]};
}

double HighwayMap::curvature(double s) const
{
  return map_trajectory.curvature(s);
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

