//==============================================================================
// retrace_service.cpp
//==============================================================================
// PURPOSE:
//   Implements MissionDatabaseNode::retracePathCallback -- the handler for
//   /{robot}/mission_database/retrace_path (mission_database/GetRetracePath).
//
//   "I'm here -- give me the way back." Reads the breadcrumb trail and home
//   position from SQLite, hands them to the pure geometry in
//   retrace_path.hpp, and copies the result into the response.
//
// WHY A SEPARATE FILE:
//   It is a member of MissionDatabaseNode (it needs db_), but living in its
//   own translation unit means mission_database_node.cpp only gains the
//   one-block service registration. All the path logic is in
//   include/mission_database/retrace_path.hpp, which has no ROS or SQLite
//   dependency and is tested by test/test_retrace_path.cpp.
//
// CONTRACT:
//   - Read-only. Never writes to the database; the logger is unchanged.
//   - Same trail + same from position -> same answer. That is what lets a
//     restarted BT pick up a retrace where the dead one left off.
//   - An empty path is success=true with an explanatory message.
//     success=false means the DB query itself failed.
//
// PARAMETERS (read on every call, so `ros2 param set` takes effect at once):
//   retrace_closure_radius_m  (default 0.9 x min_distance_m)
//       A crumb landing this close to an older kept point means "second pass
//       over old ground" and the excursion in between is dropped. MUST stay
//       below min_distance_m or ordinary neighbouring crumbs would trigger it
//       and the trail would collapse; it is clamped here with a WARN if not.
//   retrace_anchor_radius_m   (default 1.5 x min_distance_m)
//       The path starts at the oldest kept point within this radius of the
//       robot, so the first target is behind the robot, never beside it.
//
// THREADING:
//   Touches only db_ (opened SQLITE_OPEN_FULLMUTEX, so SQLite serialises
//   access), min_distance_m_ and debug_enabled_ (written once in the
//   constructor), and the parameter API (thread-safe). state_mutex_ is not
//   needed. The trail is read in a single SELECT, so a crumb inserted or an
//   eviction run during the call cannot produce a half-updated path.
//
// QUICK CHECK FROM A SHELL:
//   ros2 service call /warthog1/mission_database/retrace_path
//        mission_database/srv/GetRetracePath "{max_path_m: 30.0}"     (one line)
//   (from_lat/from_lon left at 0.0 = "start from the newest point")
//==============================================================================

#include <cmath>
#include <optional>
#include <sstream>
#include <vector>

#include "mission_database/mission_database_node.hpp"
#include "mission_database/retrace_path.hpp"

namespace mission_database
{

void MissionDatabaseNode::retracePathCallback(
  const std::shared_ptr<srv::GetRetracePath::Request> request,
  std::shared_ptr<srv::GetRetracePath::Response> response) const
{
  //--------------------------------------------------------------------------
  // STEP 1: snapshot the trail (oldest -> newest) in one query
  //--------------------------------------------------------------------------
  std::vector<retrace::Point> trail;
  {
    SqliteStmt stmt(db_, "SELECT lat, lon FROM breadcrumbs ORDER BY id ASC;");
    if (!stmt) {
      response->success = false;
      response->message = std::string("Trail query failed: ") + sqlite3_errmsg(db_);
      RCLCPP_ERROR(get_logger(), "[MissionDatabase] Retrace: %s", response->message.c_str());
      return;
    }
    while (sqlite3_step(stmt.get()) == SQLITE_ROW) {
      trail.push_back(retrace::Point{
          sqlite3_column_double(stmt.get(), 0),
          sqlite3_column_double(stmt.get(), 1)});
    }
  }

  //--------------------------------------------------------------------------
  // STEP 2: home position (optional -- the row exists only once home is set)
  //--------------------------------------------------------------------------
  std::optional<retrace::Point> home;
  {
    SqliteStmt stmt(db_, "SELECT lat, lon FROM home_position WHERE id = 1;");
    if (stmt && sqlite3_step(stmt.get()) == SQLITE_ROW) {
      home = retrace::Point{
        sqlite3_column_double(stmt.get(), 0),
        sqlite3_column_double(stmt.get(), 1)};
    }
  }

  //--------------------------------------------------------------------------
  // STEP 3: robot position -- sentinel handling
  //   Non-finite, or both exactly 0.0 (an unfilled request), means "unknown".
  //--------------------------------------------------------------------------
  std::optional<retrace::Point> from;
  const bool from_unset =
    !std::isfinite(request->from_lat) || !std::isfinite(request->from_lon) ||
    (request->from_lat == 0.0 && request->from_lon == 0.0);
  if (!from_unset) {
    from = retrace::Point{request->from_lat, request->from_lon};
  }

  //--------------------------------------------------------------------------
  // STEP 4: tunables
  //--------------------------------------------------------------------------
  retrace::Params params;
  params.closure_radius_m = get_parameter("retrace_closure_radius_m").as_double();
  params.anchor_radius_m = get_parameter("retrace_anchor_radius_m").as_double();
  params.max_path_m = request->max_path_m;
  params.min_spacing_m = request->min_spacing_m;

  // WHY CLAMP: at or above the crumb spacing, every crumb "lands on" its
  // neighbour and the whole trail is erased down to one point.
  const double closure_limit_m = 0.95 * min_distance_m_;
  if (!(params.closure_radius_m > 0.0) || params.closure_radius_m > closure_limit_m) {
    RCLCPP_WARN(get_logger(),
      "[MissionDatabase] Retrace: retrace_closure_radius_m=%.2f must be in (0, %.2f] "
      "(below min_distance_m=%.2f) -- using %.2f",
      params.closure_radius_m, closure_limit_m, min_distance_m_, closure_limit_m);
    params.closure_radius_m = closure_limit_m;
  }

  //--------------------------------------------------------------------------
  // STEP 5: compute and fill the response
  //--------------------------------------------------------------------------
  const retrace::Result result = retrace::computeRetracePath(trail, from, home, params);

  response->lats.reserve(result.path.size());
  response->lons.reserve(result.path.size());
  for (const retrace::Point & p : result.path) {
    response->lats.push_back(p.lat);
    response->lons.push_back(p.lon);
  }
  response->path_length_m = result.path_length_m;
  response->ends_at_home = result.ends_at_home;
  response->crumbs_total = static_cast<int32_t>(result.crumbs_total);
  response->crumbs_erased = static_cast<int32_t>(result.crumbs_erased);
  response->success = true;

  std::ostringstream msg;
  msg.setf(std::ios::fixed);
  msg.precision(1);
  if (result.path.empty()) {
    msg << "OK: empty path ("
        << (trail.empty() ? "no breadcrumbs recorded" : "nothing within budget")
        << (request->max_path_m <= 0.0 && !home.has_value() ? ", no home position stored" : "")
        << ")";
  } else {
    msg << "OK: " << result.path.size() << " points, " << result.path_length_m << " m"
        << (result.ends_at_home ? ", ends at home" : "");
    if (request->max_path_m <= 0.0 && !home.has_value()) {
      msg << " (no home position stored -- ends at oldest breadcrumb)";
    }
  }
  response->message = msg.str();

  // Always visible: one line per call. Calls happen on comms-loss events, not
  // in a loop, so this is not spammy.
  RCLCPP_INFO(get_logger(),
    "[MissionDatabase] Retrace: %s | crumbs=%zu erased=%zu from=%s budget=%.1f m spacing=%.1f m",
    response->message.c_str(), result.crumbs_total, result.crumbs_erased,
    from.has_value() ? "robot" : "newest-crumb",
    request->max_path_m, request->min_spacing_m);

  if (debug_enabled_) {
    for (std::size_t i = 0; i < result.path.size(); ++i) {
      RCLCPP_INFO(get_logger(), "[DEBUG][MissionDatabase] Retrace pt %zu: (%.9f, %.9f)",
        i, result.path[i].lat, result.path[i].lon);
    }
  }
}

}  // namespace mission_database
