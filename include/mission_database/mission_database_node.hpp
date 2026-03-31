//==============================================================================
// mission_database_node.hpp
//==============================================================================
// PURPOSE:
//   Standalone ROS2 node that maintains a persistent, on-disk record of:
//     - Breadcrumb trail  : GPS positions with comms status and heading
//     - Waypoints         : Navigation objectives dispatched by BT nodes,
//                           with completion status and comms/heading at dispatch
//     - Home position     : Manually set base/start location
//
//   All data is stored in a SQLite database rather than RAM.  The node's
//   heap footprint is essentially constant regardless of mission length.
//
// DATABASE SCHEMA:
//   breadcrumbs (
//     id INTEGER PK, lat REAL, lon REAL, timestamp TEXT,
//     has_comms INTEGER,     -- NULL=unknown, 0=no, 1=yes
//     heading REAL,          -- NULL=unknown, degrees 0-360 from compass
//     metadata TEXT          -- JSON object for extensible fields
//   )
//   home_position (
//     id INTEGER PK CHECK(id=1),   -- single-row enforced
//     lat REAL, lon REAL, timestamp TEXT, set_by TEXT
//   )
//   waypoints (
//     id INTEGER PK, waypoint_id INTEGER,
//     lat REAL, lon REAL,
//     heading REAL,                      -- target heading from WaypointEvent
//     name TEXT,
//     dispatched_at TEXT,
//     has_comms_at_dispatch INTEGER,     -- NULL=unknown, 0=no, 1=yes
//     compass_heading_at_dispatch REAL,  -- NULL=unknown, degrees 0-360
//     status TEXT,                       -- 'pending'|'reached'|'failed'
//     completed_at TEXT                  -- NULL until REACHED/FAILED
//   )
//
// SUBSCRIPTIONS:
//   /{robot_name}/sensors/ublox/fix        (sensor_msgs/NavSatFix)
//   /{robot_name}/comms                    (west_point_comms_sim/msg/CommsStatus)
//   /{robot_name}/compass                  (std_msgs/Float64, degrees 0-360)
//   /{robot_name}/mission_database/waypoint_event  (mission_database/WaypointEvent)
//
// PUBLICATIONS  (all transient_local / latched):
//   /{robot_name}/mission_database/breadcrumb_trail   (std_msgs/String, JSON)
//   /{robot_name}/mission_database/waypoints          (std_msgs/String, JSON)
//   /{robot_name}/mission_database/recovery_nav       (std_msgs/String, JSON)
//   /{robot_name}/mission_database/last_comms_position (std_msgs/String, JSON)
//   /{robot_name}/mission_database/home_position       (std_msgs/String, JSON)
//   /{robot_name}/mission_database/stats               (std_msgs/String, JSON)
//
// SERVICES:
//   /{robot_name}/mission_database/query  (mission_database/QueryBreadcrumbs)
//
// ROS2 PARAMETERS:
//   robot_name           (string,  required)
//   db_path              (string,  default="/tmp/{robot_name}_mission_database.db")
//   home_lat             (double,  default=NaN -- unset)
//   home_lon             (double,  default=NaN -- unset)
//   min_distance_m       (double,  default=5.0)
//   max_db_size_mb       (double,  default=10.0)
//   publish_window_size  (int,     default=100)
//   publish_rate_hz      (double,  default=1.0)
//   debug                (bool,    default=false)
//
// ADAPTING THE COMPASS SUBSCRIPTION:
//   When you create your compass node, publish std_msgs/Float64 (degrees 0-360)
//   on /{robot_name}/compass and it will wire up automatically.
//   If you use a custom message type instead, change the template type in
//   compass_sub_, compassCallback's parameter, and the msg->data access --
//   three lines in two files, nothing else changes.
//
// THREAD SAFETY:
//   state_mutex_ protects all in-memory state. SQLite is opened with
//   SQLITE_OPEN_FULLMUTEX. All DB calls happen on the single executor thread.
//==============================================================================

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <west_point_comms_sim/msg/comms_status.hpp>

#include "mission_database/msg/waypoint_event.hpp"
#include "mission_database/srv/query_breadcrumbs.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace mission_database {

//==============================================================================
// RAII SQLITE STATEMENT GUARD
//==============================================================================
class SqliteStmt
{
public:
    SqliteStmt(sqlite3 * db, const char * sql)
    {
        sqlite3_prepare_v2(db, sql, -1, &stmt_, nullptr);
    }
    ~SqliteStmt() { sqlite3_finalize(stmt_); }

    SqliteStmt(const SqliteStmt &)             = delete;
    SqliteStmt & operator=(const SqliteStmt &) = delete;

    sqlite3_stmt *  get()    const { return stmt_; }
    explicit operator bool() const { return stmt_ != nullptr; }

private:
    sqlite3_stmt * stmt_{nullptr};
};

//==============================================================================
// MISSIONDATABASENODE
//==============================================================================
class MissionDatabaseNode : public rclcpp::Node
{
public:
    explicit MissionDatabaseNode(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

    ~MissionDatabaseNode();  // Closes SQLite cleanly (WAL checkpoint on exit)

private:
    //==========================================================================
    // CALLBACKS
    //==========================================================================

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    void commsCallback(
        const west_point_comms_sim::msg::CommsStatus::SharedPtr msg);

    // Receives heading in degrees (0-360 CW from north) from std_msgs/Float64.
    // If your compass node uses a custom type, change Float64 to that type
    // here, in compass_sub_, and replace msg->data with your heading field.
    void compassCallback(const std_msgs::msg::Float64::SharedPtr msg);

    // Handles DISPATCHED / REACHED / FAILED events from BT nodes.
    void waypointEventCallback(const msg::WaypointEvent::SharedPtr msg);

    void publishTimerCallback();

    void queryServiceCallback(
        const std::shared_ptr<srv::QueryBreadcrumbs::Request>  request,
              std::shared_ptr<srv::QueryBreadcrumbs::Response> response) const;

    //==========================================================================
    // DATABASE
    //==========================================================================

    void openDatabase();   // Creates tables + runs migrations. Throws on failure.
    void closeDatabase();
    int  execSql(const char * sql);

    // INSERT one breadcrumb row. Caller must NOT hold state_mutex_.
    void insertBreadcrumb(double lat, double lon,
                          const std::string & timestamp,
                          const std::optional<bool>   & has_comms,
                          const std::optional<double> & heading,
                          const std::map<std::string, std::string> & metadata);

    void evictOldestRows(int64_t count);
    int64_t getRowCount() const;
    void enforceStorageLimit();

    // INSERT waypoint row on DISPATCHED.
    void insertWaypoint(uint32_t waypoint_id,
                        double lat, double lon,
                        double target_heading,
                        const std::string & name,
                        const std::string & dispatched_at,
                        const std::optional<bool>   & has_comms_at_dispatch,
                        const std::optional<double> & compass_heading_at_dispatch);

    // UPDATE status + completed_at on REACHED or FAILED.
    void updateWaypointStatus(uint32_t waypoint_id,
                              const std::string & status,
                              const std::string & completed_at);

    //==========================================================================
    // QUERIES
    //==========================================================================

    // JSON array of breadcrumbs. limit=0 returns all rows.
    std::string queryBreadcrumbsJson(int32_t limit,
                                     bool    filter_comms = false,
                                     bool    comms_val    = true) const;

    //==========================================================================
    // PUBLISH HELPERS
    //==========================================================================

    void publishTrail();
    void publishStats();
    void publishWaypointList();
    void publishHomePosition();
    void updateAndPublishLastCommsPosition();
    void publishRecoveryNavBundle();

    // Writes to DB and calls publishHomePosition() + publishRecoveryNavBundle().
    // source: "parameter" | "runtime_update"
    // heading: nullopt if not specified (stored as SQL NULL, published as JSON null)
    void setHomePosition(double lat, double lon,
                         const std::optional<double> & heading,
                         const std::string & source);

    //==========================================================================
    // PURE UTILITIES
    //==========================================================================

    static double haversineDistance(double lat1, double lon1,
                                    double lat2, double lon2);

    static bool hasBaseInTransitive(const std::vector<std::string> & transitive);

    //==========================================================================
    // ROS2 INTERFACES
    //==========================================================================

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr            gps_sub_;
    rclcpp::Subscription<west_point_comms_sim::msg::CommsStatus>::SharedPtr comms_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr                 compass_sub_;
    rclcpp::Subscription<msg::WaypointEvent>::SharedPtr                     waypoint_event_sub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   trail_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   stats_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   last_comms_pos_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   home_pos_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   waypoints_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   recovery_nav_pub_;

    rclcpp::Service<srv::QueryBreadcrumbs>::SharedPtr     query_service_;
    rclcpp::TimerBase::SharedPtr                          publish_timer_;

    // Kept alive so the parameter-change callback remains registered.
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    //==========================================================================
    // SQLITE
    //==========================================================================

    sqlite3 *             db_{nullptr};
    std::filesystem::path db_path_;

    //==========================================================================
    // IN-MEMORY STATE  (state_mutex_)
    //
    // Only the values needed to decide whether the next GPS sample warrants a
    // new breadcrumb, and to snapshot into new DB rows. All history is on disk.
    //==========================================================================

    mutable std::mutex    state_mutex_;

    std::optional<double> last_lat_;
    std::optional<double> last_lon_;
    std::optional<bool>   current_has_comms_;
    std::optional<double> current_heading_;    // degrees 0-360, nullopt until first compass msg

    // In-memory cache of the last-recorded comms position for O(1) publish.
    std::optional<double> last_comms_lat_;
    std::optional<double> last_comms_lon_;

    //==========================================================================
    // PARAMETERS
    //==========================================================================

    std::string robot_name_;
    double      min_distance_m_;
    size_t      max_db_size_bytes_;
    int32_t     publish_window_size_;
    bool        debug_enabled_;

    //==========================================================================
    // DIAGNOSTIC COUNTERS  (state_mutex_)
    //==========================================================================

    uint64_t stat_gps_received_{0};
    uint64_t stat_gps_skipped_dedup_{0};
    uint64_t stat_crumbs_recorded_{0};
    uint64_t stat_crumbs_evicted_{0};
};

}  // namespace mission_database