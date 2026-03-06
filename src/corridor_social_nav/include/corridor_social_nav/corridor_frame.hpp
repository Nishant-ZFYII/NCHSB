#ifndef CORRIDOR_SOCIAL_NAV__CORRIDOR_FRAME_HPP_
#define CORRIDOR_SOCIAL_NAV__CORRIDOR_FRAME_HPP_

#include <vector>
#include <utility>
#include "nav_msgs/msg/path.hpp"

namespace corridor_social_nav
{

struct FrenetPoint
{
  double s;  // arc-length along corridor centreline
  double d;  // signed lateral offset (positive = left of centreline)
};

struct FrenetState
{
  double s, d;
  double ds, dd;  // velocities in Frenet frame
};

class CorridorFrame
{
public:
  CorridorFrame() = default;

  void updatePath(const nav_msgs::msg::Path & path);

  FrenetPoint toFrenet(double x, double y) const;

  std::pair<double, double> toCartesian(double s, double d) const;

  FrenetState toFrenetState(
    double x, double y, double vx, double vy) const;

  double totalLength() const { return cumulative_length_.empty() ? 0.0 : cumulative_length_.back(); }

  bool isValid() const { return waypoints_.size() >= 2; }

private:
  struct Waypoint { double x, y; };
  std::vector<Waypoint> waypoints_;
  std::vector<double> cumulative_length_;
};

}  // namespace corridor_social_nav

#endif  // CORRIDOR_SOCIAL_NAV__CORRIDOR_FRAME_HPP_
