#include "corridor_social_nav/corridor_frame.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

namespace corridor_social_nav
{

void CorridorFrame::updatePath(const nav_msgs::msg::Path & path)
{
  waypoints_.clear();
  cumulative_length_.clear();

  if (path.poses.size() < 2) {
    return;
  }

  for (const auto & pose : path.poses) {
    waypoints_.push_back({pose.pose.position.x, pose.pose.position.y});
  }

  cumulative_length_.resize(waypoints_.size(), 0.0);
  for (size_t i = 1; i < waypoints_.size(); ++i) {
    double dx = waypoints_[i].x - waypoints_[i - 1].x;
    double dy = waypoints_[i].y - waypoints_[i - 1].y;
    cumulative_length_[i] = cumulative_length_[i - 1] + std::hypot(dx, dy);
  }
}

FrenetPoint CorridorFrame::toFrenet(double x, double y) const
{
  if (!isValid()) {
    return {0.0, 0.0};
  }

  double best_s = 0.0;
  double best_d = std::numeric_limits<double>::max();
  double best_dist_sq = std::numeric_limits<double>::max();

  for (size_t i = 0; i + 1 < waypoints_.size(); ++i) {
    double ax = waypoints_[i].x, ay = waypoints_[i].y;
    double bx = waypoints_[i + 1].x, by = waypoints_[i + 1].y;
    double abx = bx - ax, aby = by - ay;
    double apx = x - ax, apy = y - ay;
    double ab_len_sq = abx * abx + aby * aby;

    if (ab_len_sq < 1e-12) { continue; }

    double t = std::clamp((apx * abx + apy * aby) / ab_len_sq, 0.0, 1.0);

    double proj_x = ax + t * abx;
    double proj_y = ay + t * aby;
    double dx = x - proj_x;
    double dy = y - proj_y;
    double dist_sq = dx * dx + dy * dy;

    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      double seg_len = std::sqrt(ab_len_sq);
      best_s = cumulative_length_[i] + t * seg_len;

      double cross = abx * (y - ay) - aby * (x - ax);
      best_d = (cross >= 0.0 ? 1.0 : -1.0) * std::sqrt(dist_sq);
    }
  }

  return {best_s, best_d};
}

std::pair<double, double> CorridorFrame::toCartesian(double s, double d) const
{
  if (!isValid()) {
    return {0.0, 0.0};
  }

  double s_clamped = std::clamp(s, 0.0, cumulative_length_.back());

  size_t seg = 0;
  for (size_t i = 1; i < cumulative_length_.size(); ++i) {
    if (cumulative_length_[i] >= s_clamped) {
      seg = i - 1;
      break;
    }
  }

  double seg_len = cumulative_length_[seg + 1] - cumulative_length_[seg];
  double t = (seg_len > 1e-12) ? (s_clamped - cumulative_length_[seg]) / seg_len : 0.0;

  double cx = waypoints_[seg].x + t * (waypoints_[seg + 1].x - waypoints_[seg].x);
  double cy = waypoints_[seg].y + t * (waypoints_[seg + 1].y - waypoints_[seg].y);

  double tx = waypoints_[seg + 1].x - waypoints_[seg].x;
  double ty = waypoints_[seg + 1].y - waypoints_[seg].y;
  double tlen = std::hypot(tx, ty);
  if (tlen > 1e-12) {
    double nx = -ty / tlen;
    double ny = tx / tlen;
    cx += d * nx;
    cy += d * ny;
  }

  return {cx, cy};
}

FrenetState CorridorFrame::toFrenetState(
  double x, double y, double vx, double vy) const
{
  FrenetPoint fp = toFrenet(x, y);

  if (!isValid()) {
    return {fp.s, fp.d, 0.0, 0.0};
  }

  double s_clamped = std::clamp(fp.s, 0.0, cumulative_length_.back());
  size_t seg = 0;
  for (size_t i = 1; i < cumulative_length_.size(); ++i) {
    if (cumulative_length_[i] >= s_clamped) {
      seg = i - 1;
      break;
    }
  }

  double tx = waypoints_[seg + 1].x - waypoints_[seg].x;
  double ty = waypoints_[seg + 1].y - waypoints_[seg].y;
  double tlen = std::hypot(tx, ty);

  double ds = 0.0, dd = 0.0;
  if (tlen > 1e-12) {
    tx /= tlen;
    ty /= tlen;
    ds = vx * tx + vy * ty;
    dd = -vx * ty + vy * tx;
  }

  return {fp.s, fp.d, ds, dd};
}

}  // namespace corridor_social_nav
