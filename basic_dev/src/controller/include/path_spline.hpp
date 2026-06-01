#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Dense>

namespace path_spline
{

struct Sample
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
};

inline double clamp(double x, double lo, double hi)
{
  return std::max(lo, std::min(x, hi));
}

inline Eigen::Vector3d clampNorm(const Eigen::Vector3d& v, double max_norm)
{
  if (max_norm <= 0.0)
  {
    return Eigen::Vector3d::Zero();
  }
  const double n = v.norm();
  if (n <= max_norm || n < 1e-9)
  {
    return v;
  }
  return v * (max_norm / n);
}

// Arc-length cubic Hermite spline through 3D waypoints (xyz only).
class PathSpline3D
{
public:
  bool buildFromWaypoints(const std::vector<Eigen::Vector3d>& waypoints)
  {
    if (waypoints.size() < 2)
    {
      return false;
    }

    points_ = waypoints;
    arc_length_.clear();
    arc_length_.reserve(points_.size());
    arc_length_.push_back(0.0);

    for (std::size_t i = 1; i < points_.size(); ++i)
    {
      const double ds = (points_[i] - points_[i - 1]).norm();
      arc_length_.push_back(arc_length_.back() + std::max(ds, 1e-6));
    }
    total_length_ = arc_length_.back();

    tangents_.assign(points_.size(), Eigen::Vector3d::Zero());
    for (std::size_t i = 0; i < points_.size(); ++i)
    {
      Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
      if (i > 0 && i + 1 < points_.size())
      {
        const double ds =
            std::max(arc_length_[i + 1] - arc_length_[i - 1], 1e-6);
        tangent = (points_[i + 1] - points_[i - 1]) / ds;
      }
      else if (i + 1 < points_.size())
      {
        const double ds = std::max(arc_length_[i + 1] - arc_length_[i], 1e-6);
        tangent = (points_[i + 1] - points_[i]) / ds;
      }
      else if (i > 0)
      {
        const double ds = std::max(arc_length_[i] - arc_length_[i - 1], 1e-6);
        tangent = (points_[i] - points_[i - 1]) / ds;
      }
      tangents_[i] = clampNorm(tangent, 1.0);
    }

    return total_length_ > 1e-6;
  }

  double totalLength() const
  {
    return total_length_;
  }

  std::size_t numWaypoints() const
  {
    return points_.size();
  }

  std::size_t nearestWaypointIndex(
      const Eigen::Vector3d& query,
      std::size_t hint_index,
      std::size_t search_back,
      std::size_t search_ahead) const
  {
    if (points_.empty())
    {
      return 0;
    }

    const std::size_t n = points_.size();
    const std::size_t hint = std::min(hint_index, n - 1);
    const std::size_t i_begin =
        hint > search_back ? hint - search_back : 0;
    const std::size_t i_end =
        std::min(n - 1, hint + search_ahead);

    std::size_t best = hint;
    double best_dist2 = (points_[hint] - query).squaredNorm();
    for (std::size_t i = i_begin; i <= i_end; ++i)
    {
      const double dist2 = (points_[i] - query).squaredNorm();
      if (dist2 < best_dist2)
      {
        best_dist2 = dist2;
        best = i;
      }
    }
    return best;
  }

  double arcLengthAtIndex(std::size_t index) const
  {
    if (arc_length_.empty())
    {
      return 0.0;
    }
    index = std::min(index, arc_length_.size() - 1);
    return arc_length_[index];
  }

  // Closest arc-length s to query (segment projection, not waypoint distance).
  double closestArcLength(
      const Eigen::Vector3d& query,
      std::size_t hint_index,
      std::size_t search_back,
      std::size_t search_ahead) const
  {
    if (points_.size() < 2)
    {
      return 0.0;
    }

    const std::size_t hint = nearestWaypointIndex(
        query, hint_index, search_back, search_ahead);
    const std::size_t n = points_.size();
    const std::size_t seg_begin = hint > 0 ? hint - 1 : 0;
    const std::size_t seg_end = std::min(n - 2, hint + search_ahead);

    double best_s = arc_length_[hint];
    double best_dist2 = (points_[hint] - query).squaredNorm();

    for (std::size_t seg = seg_begin; seg <= seg_end; ++seg)
    {
      const Eigen::Vector3d& p0 = points_[seg];
      const Eigen::Vector3d& p1 = points_[seg + 1];
      const Eigen::Vector3d ab = p1 - p0;
      const double ab2 = ab.squaredNorm();
      if (ab2 < 1e-12)
      {
        continue;
      }
      const double t = clamp((query - p0).dot(ab) / ab2, 0.0, 1.0);
      const Eigen::Vector3d proj = p0 + t * ab;
      const double dist2 = (proj - query).squaredNorm();
      if (dist2 < best_dist2)
      {
        best_dist2 = dist2;
        const double s0 = arc_length_[seg];
        const double s1 = arc_length_[seg + 1];
        best_s = s0 + t * (s1 - s0);
      }
    }
    return best_s;
  }

  Sample sampleAtArcLength(double s, double speed) const
  {
    Sample out;
    if (points_.size() < 2)
    {
      return out;
    }

    s = clamp(s, 0.0, total_length_);
    speed = std::max(0.0, speed);

    std::size_t seg = 0;
    while (seg + 1 < arc_length_.size() && arc_length_[seg + 1] < s)
    {
      ++seg;
    }
    if (seg + 1 >= points_.size())
    {
      seg = points_.size() - 2;
    }

    const double s0 = arc_length_[seg];
    const double s1 = arc_length_[seg + 1];
    const double ds = std::max(s1 - s0, 1e-6);
    const double u = clamp((s - s0) / ds, 0.0, 1.0);

    const Eigen::Vector3d& p0 = points_[seg];
    const Eigen::Vector3d& p1 = points_[seg + 1];
    const Eigen::Vector3d& m0 = tangents_[seg];
    const Eigen::Vector3d& m1 = tangents_[seg + 1];

    const double u2 = u * u;
    const double u3 = u2 * u;
    const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
    const double h10 = u3 - 2.0 * u2 + u;
    const double h01 = -2.0 * u3 + 3.0 * u2;
    const double h11 = u3 - u2;

    out.position = h00 * p0 + h10 * ds * m0 + h01 * p1 + h11 * ds * m1;

    const double dh00 = 6.0 * u2 - 6.0 * u;
    const double dh10 = 3.0 * u2 - 4.0 * u + 1.0;
    const double dh01 = -6.0 * u2 + 6.0 * u;
    const double dh11 = 3.0 * u2 - 2.0 * u;
    const Eigen::Vector3d dp_du =
        dh00 * p0 + dh10 * ds * m0 + dh01 * p1 + dh11 * ds * m1;
    const Eigen::Vector3d dp_ds = dp_du / ds;

    const double ddh00 = 12.0 * u - 6.0;
    const double ddh10 = 6.0 * u - 4.0;
    const double ddh01 = -12.0 * u + 6.0;
    const double ddh11 = 6.0 * u - 2.0;
    const Eigen::Vector3d d2p_du2 =
        ddh00 * p0 + ddh10 * ds * m0 + ddh01 * p1 + ddh11 * ds * m1;
    const Eigen::Vector3d d2p_ds2 = d2p_du2 / (ds * ds);

    out.velocity = dp_ds * speed;
    out.acceleration = d2p_ds2 * speed * speed;
    return out;
  }

private:
  std::vector<Eigen::Vector3d> points_;
  std::vector<double> arc_length_;
  std::vector<Eigen::Vector3d> tangents_;
  double total_length_ = 0.0;
};

// Insert points on straight chords so segment length <= max_segment_m.
inline std::vector<Eigen::Vector3d> densifyWaypointsByMaxSegment(
    const std::vector<Eigen::Vector3d>& input,
    double max_segment_m)
{
  if (input.size() < 2 || max_segment_m <= 0.0)
  {
    return input;
  }

  std::vector<Eigen::Vector3d> out;
  out.reserve(input.size() * 2);
  out.push_back(input.front());
  for (std::size_t i = 1; i < input.size(); ++i)
  {
    const Eigen::Vector3d& a = out.back();
    const Eigen::Vector3d& b = input[i];
    const double len = (b - a).norm();
    if (len > max_segment_m)
    {
      const int segments = static_cast<int>(std::ceil(len / max_segment_m));
      for (int k = 1; k < segments; ++k)
      {
        const double t = static_cast<double>(k) / static_cast<double>(segments);
        out.push_back(a + t * (b - a));
      }
    }
    out.push_back(b);
  }
  return out;
}

inline std::vector<Eigen::Vector3d> smoothWaypoints(
    const std::vector<Eigen::Vector3d>& input,
    int window)
{
  if (window < 3 || input.size() < 3)
  {
    return input;
  }
  if (window % 2 == 0)
  {
    ++window;
  }
  const int half = window / 2;
  std::vector<Eigen::Vector3d> out = input;
  for (std::size_t i = 0; i < input.size(); ++i)
  {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    int count = 0;
    for (int k = -half; k <= half; ++k)
    {
      const int idx = static_cast<int>(i) + k;
      if (idx < 0 || idx >= static_cast<int>(input.size()))
      {
        continue;
      }
      sum += input[static_cast<std::size_t>(idx)];
      ++count;
    }
    if (count > 0)
    {
      out[i] = sum / static_cast<double>(count);
    }
  }
  return out;
}

}  // namespace path_spline
