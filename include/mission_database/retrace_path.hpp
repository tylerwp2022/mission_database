//==============================================================================
// retrace_path.hpp
//==============================================================================
// PURPOSE:
//   Turns the recorded breadcrumb trail into "the way back" -- an ordered list
//   of lat/lon points the robot can drive through to return along the path it
//   came in on. Backs the GetRetracePath service.
//
// WHY A SEPARATE, ROS-FREE HEADER:
//   This is pure geometry over a list of points. Keeping it free of rclcpp and
//   SQLite means it can be compiled and tested with nothing but g++
//   (see test/test_retrace_path.cpp), and the node-side service callback
//   (src/retrace_service.cpp) stays a thin "read DB -> call this -> fill
//   response" wrapper.
//
// THE PROBLEM THIS SOLVES (why "just reverse the trail" is wrong):
//   The logger records a crumb every min_distance_m no matter what the robot
//   is doing -- including while it is retracing. After one loss/retrace/
//   proceed cycle the trail looks like this (s = metres along the route):
//
//       outbound      1..20   s =   8 .. 160     <- comms lost at 160
//       retrace #1   21..25   s = 152 .. 120     <- backed out, held, PROCEED
//       re-advance   26..35   s = 128 .. 200     <- comms lost again at 200
//
//   Reversing that raw trail from crumb 35 drives back to 120, then FORWARD
//   to 160 (replaying retrace #1 backwards), then back again.
//
// THE RULE (loop erasure):
//   Replay the trail oldest -> newest, keeping a list of "kept" points. If a
//   new crumb lands within closure_radius_m of a point already kept,
//   everything recorded since that kept point was an out-and-back excursion:
//   drop it. What survives is one simple line from the oldest crumb to where
//   the robot is now. The retrace path is that line, reversed.
//
//   WHY NO EXTRA BOOKKEEPING IS NEEDED: consecutive crumbs on an ordinary
//   stretch of trail are always >= min_distance_m apart (the logger guarantees
//   it). So as long as closure_radius_m < min_distance_m, an ordinary stretch
//   can never trigger the rule -- only a second pass over old ground can.
//   The caller MUST enforce closure_radius_m < min_distance_m.
//
// KNOWN LIMITS (by design, tunable):
//   - If the route legitimately passes within closure_radius_m of itself
//     (tight switchback, lanes closer than ~7 m), the loop between the two
//     passes is cut out and the robot is sent straight across the gap.
//   - A return pass more than ~closure_radius_m to the side of the outbound
//     pass is not recognised as the same ground; that stretch is replayed.
//   - The logger is unchanged: the on-disk trail still contains every crumb,
//     so study data and TAK replay are unaffected. Erasure happens only in
//     this query.
//
// THREADING:
//   Stateless free functions over caller-owned data. Safe to call from any
//   thread.
//
// COST:
//   O(n * kept) point comparisons, each usually settled by a cheap lat/lon
//   box test. Thousands of crumbs take milliseconds. Memory is one transient
//   vector of 16 bytes per crumb for the duration of the call.
//==============================================================================

#pragma once

#include <algorithm>  // std::max
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace mission_database::retrace
{

//==============================================================================
// TYPES
//==============================================================================

struct Point
{
  double lat{0.0};  // degrees
  double lon{0.0};  // degrees
};

struct Params
{
  // Loop-closure radius. MUST be < the logger's min_distance_m (see header).
  double closure_radius_m{7.2};

  // Start-point search radius around the robot. The path starts at the OLDEST
  // kept point within this radius, so the first target is always behind the
  // robot on the trail, never beside or ahead of it.
  double anchor_radius_m{12.0};

  // <= 0: whole trail + home.  > 0: stop once path length >= this.
  double max_path_m{0.0};

  // <= 0: every crumb.  > 0: thin to at least this spacing (last point kept).
  double min_spacing_m{0.0};
};

struct Result
{
  std::vector<Point> path;       // driving order: path[0] is the first target
  double path_length_m{0.0};     // robot -> path[0] -> ... -> path.back()
  bool ends_at_home{false};
  std::size_t crumbs_total{0};   // crumbs given
  std::size_t crumbs_erased{0};  // crumbs dropped as excursions
};

//==============================================================================
// GEOMETRY HELPERS
//==============================================================================

namespace detail
{
constexpr double kEarthRadiusM = 6'371'000.0;  // same sphere as the node's haversine
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
}  // namespace detail

// Great-circle distance in metres.
// WHY DUPLICATED (the node has its own haversineDistance): this header must
// stay free of the node class so it can be tested standalone.
inline double haversineM(const Point & a, const Point & b)
{
  const double phi1 = a.lat * detail::kDegToRad;
  const double phi2 = b.lat * detail::kDegToRad;
  const double dphi = (b.lat - a.lat) * detail::kDegToRad;
  const double dlam = (b.lon - a.lon) * detail::kDegToRad;
  const double s1 = std::sin(dphi / 2.0);
  const double s2 = std::sin(dlam / 2.0);
  const double h = s1 * s1 + std::cos(phi1) * std::cos(phi2) * s2 * s2;
  return detail::kEarthRadiusM * 2.0 * std::atan2(std::sqrt(h), std::sqrt(1.0 - h));
}

// True if a and b are within radius_m of each other.
// WHY THE BOX TEST FIRST: on a sphere the distance between two points is never
// less than their north-south separation, so that check can reject without
// trig. The east-west check is scaled by 0.99 so it can only under-estimate,
// i.e. it never rejects a pair the full haversine would have accepted.
inline bool withinRadius(const Point & a, const Point & b, double radius_m)
{
  const double ns_m = std::abs(a.lat - b.lat) * detail::kDegToRad * detail::kEarthRadiusM;
  if (ns_m > radius_m) {return false;}

  const double max_abs_lat = std::max(std::abs(a.lat), std::abs(b.lat));
  const double ew_m = std::abs(a.lon - b.lon) * detail::kDegToRad * detail::kEarthRadiusM *
    std::cos(max_abs_lat * detail::kDegToRad) * 0.99;
  if (ew_m > radius_m) {return false;}

  return haversineM(a, b) <= radius_m;
}

//==============================================================================
// eraseLoops
//   trail: crumbs oldest -> newest (ORDER BY id ASC).
//   Returns the kept points, oldest -> newest, with excursions removed.
//==============================================================================
inline std::vector<Point> eraseLoops(const std::vector<Point> & trail, double closure_radius_m)
{
  std::vector<Point> kept;
  kept.reserve(trail.size());

  for (const Point & crumb : trail) {
    // Find the OLDEST kept point this crumb lands on. Oldest (not nearest) so
    // that nested excursions are removed in one cut.
    std::size_t hit = kept.size();
    for (std::size_t i = 0; i < kept.size(); ++i) {
      if (withinRadius(kept[i], crumb, closure_radius_m)) {
        hit = i;
        break;
      }
    }

    if (hit < kept.size()) {
      // Second pass over old ground: drop everything recorded since kept[hit].
      // The crumb itself is not added -- kept[hit] already marks this spot,
      // and keeping the ORIGINAL point keeps the line on the outbound track.
      kept.erase(kept.begin() + static_cast<std::ptrdiff_t>(hit) + 1, kept.end());
    } else {
      kept.push_back(crumb);
    }
  }
  return kept;
}

//==============================================================================
// computeRetracePath
//   trail : crumbs oldest -> newest.
//   from  : robot position; nullopt = "unknown, start from the newest point".
//   home  : stored home position; nullopt = none stored.
//==============================================================================
inline Result computeRetracePath(
  const std::vector<Point> & trail,
  const std::optional<Point> & from,
  const std::optional<Point> & home,
  const Params & params)
{
  Result result;
  result.crumbs_total = trail.size();

  const std::vector<Point> kept = eraseLoops(trail, params.closure_radius_m);
  result.crumbs_erased = trail.size() - kept.size();

  const bool to_home = (params.max_path_m <= 0.0);

  //----------------------------------------------------------------------------
  // STEP 1: full-fidelity path, newest -> oldest, within the budget
  //----------------------------------------------------------------------------
  std::vector<Point> full;
  // Where the next leg is measured from. Unset until we know a position
  // (robot, else the first point we emit).
  std::optional<Point> cursor = from;
  double length_m = 0.0;

  if (!kept.empty()) {
    // Start at the OLDEST kept point near the robot so the first target is
    // behind it on the trail. No kept point nearby (robot strayed, or the
    // newest crumbs were erased) -> start from the newest kept point.
    std::size_t start = kept.size() - 1;
    if (from.has_value()) {
      for (std::size_t i = 0; i < kept.size(); ++i) {
        if (withinRadius(kept[i], *from, params.anchor_radius_m)) {
          start = i;
          break;
        }
      }
    }

    for (std::size_t i = start + 1; i-- > 0; ) {
      if (cursor.has_value()) {length_m += haversineM(*cursor, kept[i]);}
      full.push_back(kept[i]);
      cursor = kept[i];
      // Include the point that crosses the budget: "at least N metres".
      if (!to_home && length_m >= params.max_path_m) {break;}
    }
  }

  if (to_home && home.has_value()) {
    if (cursor.has_value()) {length_m += haversineM(*cursor, *home);}
    full.push_back(*home);
    result.ends_at_home = true;
  }

  //----------------------------------------------------------------------------
  // STEP 2: optional thinning (last point always kept)
  //----------------------------------------------------------------------------
  if (params.min_spacing_m <= 0.0 || full.size() <= 1) {
    result.path = std::move(full);
    result.path_length_m = length_m;
    return result;
  }

  std::optional<Point> prev = from;  // previous point on the FULL path
  double since_kept_m = 0.0;         // along-path distance since last kept point
  for (std::size_t i = 0; i < full.size(); ++i) {
    if (prev.has_value()) {since_kept_m += haversineM(*prev, full[i]);}
    prev = full[i];
    const bool is_last = (i + 1 == full.size());
    if (is_last || since_kept_m >= params.min_spacing_m) {
      result.path.push_back(full[i]);
      since_kept_m = 0.0;
    }
  }

  // Report the length of what is actually returned (straight legs between the
  // thinned points), since that is what the robot will be asked to drive.
  std::optional<Point> leg_from = from;
  for (const Point & p : result.path) {
    if (leg_from.has_value()) {result.path_length_m += haversineM(*leg_from, p);}
    leg_from = p;
  }
  return result;
}

}  // namespace mission_database::retrace
