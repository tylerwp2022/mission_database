//==============================================================================
// mission_database_node.cpp
//==============================================================================
// SQLite-backed mission database node.
// See mission_database_node.hpp for full design documentation.
//==============================================================================

#include "mission_database/mission_database_node.hpp"

#include <algorithm>    // std::find
#include <cmath>
#include <cstdio>       // std::snprintf
#include <ctime>        // std::gmtime, std::strftime
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace mission_database {

//==============================================================================
// CONSTANTS
//==============================================================================

static constexpr double EARTH_RADIUS_M   = 6'371'000.0;
static constexpr double DEG_TO_RAD       = M_PI / 180.0;
static constexpr int64_t EVICTION_BATCH  = 50;

//==============================================================================
// formatTimestamp
// Converts a ROS stamp to "2024-03-15 14:23:07.042 UTC"
//==============================================================================

static std::string formatTimestamp(int32_t sec, uint32_t nanosec)
{
    const std::time_t t = static_cast<std::time_t>(sec);
    std::tm * utc = std::gmtime(&t);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", utc);
    char full[48];
    std::snprintf(full, sizeof(full), "%s.%03u UTC", buf,
                  static_cast<unsigned>(nanosec / 1'000'000u));
    return std::string(full);
}

//==============================================================================
// CONSTRUCTION
//==============================================================================

MissionDatabaseNode::MissionDatabaseNode(const rclcpp::NodeOptions & options)
: Node("mission_database_node", options)
{
    //--------------------------------------------------------------------------
    // PARAMETERS
    //--------------------------------------------------------------------------

    robot_name_ = declare_parameter<std::string>("robot_name", "");
    if (robot_name_.empty()) {
        RCLCPP_FATAL(get_logger(),
            "[MissionDatabase] 'robot_name' parameter is required.");
        throw std::runtime_error("robot_name parameter is required");
    }

    const std::string default_db = "/tmp/" + robot_name_ + "_mission_database.db";
    db_path_ = declare_parameter<std::string>("db_path", default_db);

    min_distance_m_ = declare_parameter<double>("min_distance_m", 8.0);

    const double max_db_size_mb = declare_parameter<double>("max_db_size_mb", 10.0);
    max_db_size_bytes_ = static_cast<size_t>(max_db_size_mb * 1024.0 * 1024.0);

    publish_window_size_ = declare_parameter<int>("publish_window_size", 100);
    if (publish_window_size_ <= 0) publish_window_size_ = 100;

    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 1.0);
    debug_enabled_ = declare_parameter<bool>("debug", false);

    // Home position -- NaN sentinel means "not set via parameter".
    declare_parameter<double>("home_lat", std::numeric_limits<double>::quiet_NaN());
    declare_parameter<double>("home_lon", std::numeric_limits<double>::quiet_NaN());
    declare_parameter<double>("home_heading", std::numeric_limits<double>::quiet_NaN());

    //--------------------------------------------------------------------------
    // OPEN DATABASE
    //--------------------------------------------------------------------------

    openDatabase();

    //--------------------------------------------------------------------------
    // TOPIC NAMES
    //--------------------------------------------------------------------------

    const std::string ns                   = "/" + robot_name_;
    const std::string gps_topic            = ns + "/sensors/ublox/fix";
    const std::string comms_topic          = ns + "/comms";
    const std::string compass_topic        = ns + "/compass";
    const std::string gps_speed_topic      = ns + "/gps_speed";
    const std::string waypoint_event_topic = ns + "/mission_database/waypoint_event";
    const std::string trail_topic          = ns + "/mission_database/breadcrumb_trail";
    const std::string stats_topic          = ns + "/mission_database/stats";
    const std::string last_comms_topic     = ns + "/mission_database/last_comms_position";
    const std::string home_topic           = ns + "/mission_database/home_position";
    const std::string waypoints_topic      = ns + "/mission_database/waypoints";
    const std::string recovery_nav_topic   = ns + "/mission_database/recovery_nav";
    const std::string query_svc            = ns + "/mission_database/query";

    //--------------------------------------------------------------------------
    // SUBSCRIBERS
    //--------------------------------------------------------------------------

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic, 5,
        std::bind(&MissionDatabaseNode::gpsCallback, this, std::placeholders::_1));

    comms_sub_ = create_subscription<west_point_comms_sim::msg::CommsStatus>(
        comms_topic, 1,
        std::bind(&MissionDatabaseNode::commsCallback, this, std::placeholders::_1));

    // Compass -- queue=1, only latest heading matters.
    // QoS set to best_effort to match the compass node's publisher.
    // The default RELIABLE subscription QoS is incompatible with a
    // BEST_EFFORT publisher and causes a "no messages will be sent" warning.
    rclcpp::QoS compass_qos(1);
    compass_qos.reliable();
    compass_sub_ = create_subscription<std_msgs::msg::Float64>(
        compass_topic, compass_qos,
        std::bind(&MissionDatabaseNode::compassCallback, this, std::placeholders::_1));

    // GPS speed -- best_effort to match typical sensor publisher QoS.
    rclcpp::QoS speed_qos(1);
    speed_qos.best_effort();
    gps_speed_sub_ = create_subscription<std_msgs::msg::Float64>(
        gps_speed_topic, speed_qos,
        std::bind(&MissionDatabaseNode::gpsSpeedCallback, this, std::placeholders::_1));

    waypoint_event_sub_ = create_subscription<msg::WaypointEvent>(
        waypoint_event_topic, 20,
        std::bind(&MissionDatabaseNode::waypointEventCallback, this,
                  std::placeholders::_1));

    //--------------------------------------------------------------------------
    // PUBLISHERS  (transient_local = latched)
    //--------------------------------------------------------------------------

    rclcpp::QoS latched_qos(1);
    latched_qos.transient_local();

    trail_pub_          = create_publisher<std_msgs::msg::String>(trail_topic,        latched_qos);
    stats_pub_          = create_publisher<std_msgs::msg::String>(stats_topic,        latched_qos);
    last_comms_pos_pub_ = create_publisher<std_msgs::msg::String>(last_comms_topic,   latched_qos);
    home_pos_pub_       = create_publisher<std_msgs::msg::String>(home_topic,         latched_qos);
    waypoints_pub_      = create_publisher<std_msgs::msg::String>(waypoints_topic,    latched_qos);
    recovery_nav_pub_   = create_publisher<std_msgs::msg::String>(recovery_nav_topic, latched_qos);

    //--------------------------------------------------------------------------
    // PARAMETER CHANGE CALLBACK (for runtime home position updates)
    //--------------------------------------------------------------------------

    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & params)
        -> rcl_interfaces::msg::SetParametersResult
        {
            bool lat_changed = false, lon_changed = false;
            double new_lat = std::numeric_limits<double>::quiet_NaN();
            double new_lon = std::numeric_limits<double>::quiet_NaN();

            for (const auto & p : params) {
                if (p.get_name() == "home_lat")     { new_lat = p.as_double(); lat_changed = true; }
                if (p.get_name() == "home_lon")     { new_lon = p.as_double(); lon_changed = true; }
            }

            if (lat_changed || lon_changed) {
                if (!lat_changed) new_lat = get_parameter("home_lat").as_double();
                if (!lon_changed) new_lon = get_parameter("home_lon").as_double();
                if (std::isfinite(new_lat) && std::isfinite(new_lon)) {
                    const double hdg = get_parameter("home_heading").as_double();
                    const std::optional<double> heading =
                        std::isfinite(hdg) ? std::optional<double>(hdg) : std::nullopt;
                    setHomePosition(new_lat, new_lon, heading, "runtime_update");
                }
            }

            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        });

    //--------------------------------------------------------------------------
    // SERVICE
    //--------------------------------------------------------------------------

    query_service_ = create_service<srv::QueryBreadcrumbs>(
        query_svc,
        std::bind(&MissionDatabaseNode::queryServiceCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    //--------------------------------------------------------------------------
    // PUBLISH TIMER
    //--------------------------------------------------------------------------

    const auto period_ms = static_cast<int>(1000.0 / publish_rate_hz);
    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&MissionDatabaseNode::publishTimerCallback, this));

    //--------------------------------------------------------------------------
    // STARTUP LOG + INITIAL PUBLISHES
    //--------------------------------------------------------------------------

    RCLCPP_INFO(get_logger(),
        "[MissionDatabase] Started for '%s'\n"
        "  DB              : %s  (%ld existing rows)\n"
        "  GPS topic       : %s\n"
        "  Comms topic     : %s\n"
        "  Compass topic   : %s\n"
        "  GPS speed topic : %s\n"
        "  Waypoint events : %s\n"
        "  Recovery nav    : %s\n"
        "  min_distance    : %.1f m  |  max_db_size: %.1f MB  |  debug: %s",
        robot_name_.c_str(), db_path_.c_str(), getRowCount(),
        gps_topic.c_str(), comms_topic.c_str(), compass_topic.c_str(),
        gps_speed_topic.c_str(),
        waypoint_event_topic.c_str(), recovery_nav_topic.c_str(),
        min_distance_m_, max_db_size_mb, debug_enabled_ ? "true" : "false");

    // Seed home position: launch param overrides DB; otherwise re-publish stored value.
    const double param_lat = get_parameter("home_lat").as_double();
    const double param_lon = get_parameter("home_lon").as_double();
    const double param_hdg = get_parameter("home_heading").as_double();
    if (std::isfinite(param_lat) && std::isfinite(param_lon)) {
        const std::optional<double> heading =
            std::isfinite(param_hdg) ? std::optional<double>(param_hdg) : std::nullopt;
        setHomePosition(param_lat, param_lon, heading, "parameter");
    } else {
        publishHomePosition();
    }

    publishTrail();
    publishStats();
    updateAndPublishLastCommsPosition();
    publishWaypointList();
    publishRecoveryNavBundle();
}

//==============================================================================
// DESTRUCTOR
//==============================================================================

MissionDatabaseNode::~MissionDatabaseNode() { closeDatabase(); }

//==============================================================================
// DATABASE OPEN / CLOSE
//==============================================================================

void MissionDatabaseNode::openDatabase()
{
    const int flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE |
                      SQLITE_OPEN_FULLMUTEX;
    const int rc = sqlite3_open_v2(db_path_.c_str(), &db_, flags, nullptr);
    if (rc != SQLITE_OK) {
        const std::string err = sqlite3_errmsg(db_);
        sqlite3_close(db_); db_ = nullptr;
        throw std::runtime_error(
            "[MissionDatabase] Failed to open DB: " + err);
    }

    execSql("PRAGMA journal_mode=WAL;");
    execSql("PRAGMA cache_size=64;");     // 64 pages x 4KB = ~256KB RAM
    execSql("PRAGMA synchronous=NORMAL;");
    execSql("PRAGMA temp_store=MEMORY;");

    // ── BREADCRUMBS ───────────────────────────────────────────────────────────
    execSql(
        "CREATE TABLE IF NOT EXISTS breadcrumbs ("
        "  id        INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  lat       REAL    NOT NULL,"
        "  lon       REAL    NOT NULL,"
        "  timestamp TEXT    NOT NULL,"
        "  has_comms INTEGER,"       // NULL=unknown, 0=no, 1=yes
        "  heading   REAL,"          // NULL=unknown, degrees 0-360 from compass
        "  speed     REAL,"          // NULL=unknown, m/s from gps_speed
        "  metadata  TEXT    NOT NULL DEFAULT '{}'"
        ");");

    // ── HOME POSITION ─────────────────────────────────────────────────────────
    execSql(
        "CREATE TABLE IF NOT EXISTS home_position ("
        "  id        INTEGER PRIMARY KEY CHECK (id = 1),"
        "  lat       REAL    NOT NULL,"
        "  lon       REAL    NOT NULL,"
        "  heading   REAL,"          // NULL if not set, degrees 0-360
        "  timestamp TEXT    NOT NULL,"
        "  set_by    TEXT    NOT NULL"
        ");");

    // ── WAYPOINTS ─────────────────────────────────────────────────────────────
    execSql(
        "CREATE TABLE IF NOT EXISTS waypoints ("
        "  id                          INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  waypoint_id                 INTEGER NOT NULL,"
        "  lat                         REAL    NOT NULL,"
        "  lon                         REAL    NOT NULL,"
        "  heading                     REAL,"
        "  radius                      REAL    NOT NULL DEFAULT 2.0,"
        "  name                        TEXT    NOT NULL DEFAULT '',"
        "  dispatched_at               TEXT    NOT NULL,"
        "  has_comms_at_dispatch       INTEGER,"
        "  compass_heading_at_dispatch REAL,"
        "  status                      TEXT    NOT NULL DEFAULT 'pending',"
        "  completed_at                TEXT"
        ");");

    // ── MIGRATIONS (idempotent: ignore error if column already exists) ────────
    sqlite3_exec(db_,
        "ALTER TABLE breadcrumbs ADD COLUMN heading REAL;",
        nullptr, nullptr, nullptr);
    sqlite3_exec(db_,
        "ALTER TABLE breadcrumbs ADD COLUMN speed REAL;",
        nullptr, nullptr, nullptr);
    sqlite3_exec(db_,
        "ALTER TABLE waypoints ADD COLUMN compass_heading_at_dispatch REAL;",
        nullptr, nullptr, nullptr);
    sqlite3_exec(db_,
        "ALTER TABLE waypoints ADD COLUMN radius REAL NOT NULL DEFAULT 2.0;",
        nullptr, nullptr, nullptr);
    sqlite3_exec(db_,
        "ALTER TABLE home_position ADD COLUMN heading REAL;",
        nullptr, nullptr, nullptr);
}

void MissionDatabaseNode::closeDatabase()
{
    if (db_) {
        sqlite3_wal_checkpoint_v2(db_, nullptr, SQLITE_CHECKPOINT_FULL, nullptr, nullptr);
        sqlite3_close(db_);
        db_ = nullptr;
        RCLCPP_INFO(get_logger(), "[MissionDatabase] Database closed.");
    }
}

int MissionDatabaseNode::execSql(const char * sql)
{
    char * errmsg = nullptr;
    const int rc = sqlite3_exec(db_, sql, nullptr, nullptr, &errmsg);
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(get_logger(), "[MissionDatabase] SQL error '%s': %s",
                     sql, errmsg ? errmsg : "(null)");
        sqlite3_free(errmsg);
    }
    return rc;
}

//==============================================================================
// GPS CALLBACK
//==============================================================================

void MissionDatabaseNode::gpsCallback(
    const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    const double lat = msg->latitude;
    const double lon = msg->longitude;

    if (msg->status.status < 0 || !std::isfinite(lat) || !std::isfinite(lon)) {
        return;
    }

    std::optional<bool>   comms_snapshot;
    std::optional<double> heading_snapshot;
    std::optional<double> speed_snapshot;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ++stat_gps_received_;

        if (last_lat_.has_value()) {
            const double dist = haversineDistance(*last_lat_, *last_lon_, lat, lon);
            if (dist < min_distance_m_) {
                ++stat_gps_skipped_dedup_;
                return;
            }
        }

        comms_snapshot   = current_has_comms_;
        heading_snapshot = current_heading_;
        speed_snapshot   = current_speed_;
        last_lat_        = lat;
        last_lon_        = lon;
        ++stat_crumbs_recorded_;
    }

    // Use the message stamp if it's set; fall back to wall time if the GPS
    // driver left the header at the default (sec=0, nanosec=0).  A zero stamp
    // would produce "1970-01-01 00:00:00.000 UTC" which is misleading.
    const bool stamp_is_zero =
        (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0);
    const rclcpp::Time stamp =
        stamp_is_zero ? this->now() : rclcpp::Time(msg->header.stamp);

    const std::string ts = formatTimestamp(
        static_cast<int32_t>(stamp.seconds()),
        static_cast<uint32_t>(stamp.nanoseconds() % 1'000'000'000ULL));

    insertBreadcrumb(lat, lon, ts, comms_snapshot, heading_snapshot, speed_snapshot, {});
    enforceStorageLimit();
    publishTrail();
    publishStats();

    if (comms_snapshot.has_value() && comms_snapshot.value()) {
        updateAndPublishLastCommsPosition();
    }

    RCLCPP_INFO(get_logger(),
        "[MissionDatabase] Breadcrumb: (%.9f, %.9f) comms=%s hdg=%s spd=%s",
        lat, lon,
        comms_snapshot.has_value() ? (comms_snapshot.value() ? "yes" : "no") : "?",
        heading_snapshot.has_value()
            ? (std::to_string(static_cast<int>(heading_snapshot.value())) + "deg").c_str()
            : "?",
        speed_snapshot.has_value()
            ? (std::to_string(speed_snapshot.value()).substr(0, 5) + "m/s").c_str()
            : "?");
}

//==============================================================================
// COMMS CALLBACK
//==============================================================================

void MissionDatabaseNode::commsCallback(
    const west_point_comms_sim::msg::CommsStatus::SharedPtr msg)
{
    const bool has_base = hasBaseInTransitive(msg->transitive);
    std::lock_guard<std::mutex> lock(state_mutex_);

    const bool changed = !current_has_comms_.has_value() ||
                         current_has_comms_.value() != has_base;
    current_has_comms_ = has_base;

    if (changed) {
        std::string t, d;
        for (const auto & p : msg->transitive) t += p + " ";
        for (const auto & p : msg->direct)     d += p + " ";
        RCLCPP_INFO(get_logger(),
            "[MissionDatabase] Comms -> %s  direct:[%s]  transitive:[%s]",
            has_base ? "CONNECTED" : "DISCONNECTED", d.c_str(), t.c_str());
    }
}

//==============================================================================
// COMPASS CALLBACK
//
// std_msgs/Float64 -- degrees 0-360 clockwise from north when calibrated,
// -1.0 when uncalibrated.
//
// -1.0 and any non-finite value are treated as "not yet calibrated" and
// current_heading_ is cleared to nullopt so breadcrumbs record NULL heading
// rather than a meaningless value.  When a valid reading arrives after a
// period of uncalibrated output, heading recording resumes automatically.
//==============================================================================

void MissionDatabaseNode::compassCallback(
    const std_msgs::msg::Float64::SharedPtr msg)
{
    const double raw = msg->data;

    // -1.0 is the uncalibrated sentinel.  Also guard against any other
    // non-finite value from a misconfigured publisher.
    const bool uncalibrated = (raw < 0.0) || !std::isfinite(raw);

    std::lock_guard<std::mutex> lock(state_mutex_);

    if (uncalibrated) {
        if (current_heading_.has_value()) {
            RCLCPP_INFO(get_logger(),
                "[MissionDatabase] Compass uncalibrated (value: %.1f) -- "
                "heading cleared, breadcrumbs will record NULL until calibrated.", raw);
            current_heading_ = std::nullopt;
        } else if (debug_enabled_) {
            RCLCPP_DEBUG(get_logger(),
                "[DEBUG][MissionDatabase] Compass still uncalibrated: %.1f", raw);
        }
        return;
    }

    // Normalise to [0, 360) -- handles any value outside that range
    double heading = std::fmod(raw, 360.0);
    if (heading < 0.0) heading += 360.0;

    const bool was_calibrated = current_heading_.has_value();
    current_heading_ = heading;

    if (!was_calibrated) {
        RCLCPP_INFO(get_logger(),
            "[MissionDatabase] Compass calibrated -- first heading: %.1f deg", heading);
    } else if (debug_enabled_) {
        RCLCPP_DEBUG(get_logger(),
            "[DEBUG][MissionDatabase] Compass: %.1f deg", heading);
    }
}

//==============================================================================
// GPS SPEED CALLBACK
//==============================================================================

void MissionDatabaseNode::gpsSpeedCallback(
    const std_msgs::msg::Float64::SharedPtr msg)
{
    if (!std::isfinite(msg->data) || msg->data < 0.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[MissionDatabase] GPS speed invalid (%.3f m/s) -- ignoring",
            msg->data);
        return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    current_speed_ = msg->data;

    if (debug_enabled_) {
        RCLCPP_DEBUG(get_logger(),
            "[DEBUG][MissionDatabase] GPS speed: %.2f m/s", msg->data);
    }
}

//==============================================================================
// WAYPOINT EVENT CALLBACK
//==============================================================================

void MissionDatabaseNode::waypointEventCallback(
    const msg::WaypointEvent::SharedPtr msg)
{
    // Fall back to wall time if the BT node didn't fill in the header stamp
    // (a zero stamp would produce "1970-01-01 00:00:00.000 UTC").
    const bool stamp_is_zero =
        (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0);
    const rclcpp::Time stamp =
        stamp_is_zero ? this->now() : rclcpp::Time(msg->header.stamp);

    const std::string now = formatTimestamp(
        static_cast<int32_t>(stamp.seconds()),
        static_cast<uint32_t>(stamp.nanoseconds() % 1'000'000'000ULL));

    if (msg->event_type == msg::WaypointEvent::DISPATCHED) {

        std::optional<bool>   comms_snapshot;
        std::optional<double> heading_snapshot;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            comms_snapshot   = current_has_comms_;
            heading_snapshot = current_heading_;
        }

        // Use radius from message, default to 2.0 if zero or negative
        const double radius = (msg->radius > 0.0) ? msg->radius : 2.0;

        insertWaypoint(msg->waypoint_id, msg->lat, msg->lon,
                       msg->heading, radius, msg->name, now,
                       comms_snapshot, heading_snapshot);

        RCLCPP_INFO(get_logger(),
            "[MissionDatabase] Waypoint DISPATCHED id=%u '%s' "
            "(%.9f, %.9f) target_hdg=%.1f comms=%s robot_hdg=%s",
            msg->waypoint_id, msg->name.c_str(), msg->lat, msg->lon,
            msg->heading,
            comms_snapshot.has_value() ? (comms_snapshot.value() ? "yes" : "no") : "?",
            heading_snapshot.has_value()
                ? (std::to_string(static_cast<int>(*heading_snapshot)) + "deg").c_str()
                : "?");

        publishWaypointList();
        publishRecoveryNavBundle();

    } else if (msg->event_type == msg::WaypointEvent::REACHED ||
               msg->event_type == msg::WaypointEvent::FAILED)
    {
        const std::string status =
            (msg->event_type == msg::WaypointEvent::REACHED) ? "reached" : "failed";

        updateWaypointStatus(msg->waypoint_id, status, now);

        RCLCPP_INFO(get_logger(), "[MissionDatabase] Waypoint %s id=%u",
                    status.c_str(), msg->waypoint_id);

        publishWaypointList();
        publishRecoveryNavBundle();

    } else {
        RCLCPP_WARN(get_logger(),
            "[MissionDatabase] Unknown WaypointEvent type=%u -- ignoring",
            msg->event_type);
    }
}

//==============================================================================
// PUBLISH TIMER
//==============================================================================

void MissionDatabaseNode::publishTimerCallback()
{
    publishTrail();
    publishStats();
}

//==============================================================================
// QUERY SERVICE
//==============================================================================

void MissionDatabaseNode::queryServiceCallback(
    const std::shared_ptr<srv::QueryBreadcrumbs::Request>  request,
          std::shared_ptr<srv::QueryBreadcrumbs::Response> response) const
{
    try {
        response->json_result = queryBreadcrumbsJson(
            request->last_n, request->filter_by_comms, request->has_comms_value);
        response->count   = static_cast<int32_t>(getRowCount());
        response->success = true;
        response->message = "OK";
    } catch (const std::exception & e) {
        response->success     = false;
        response->message     = std::string("Query failed: ") + e.what();
        response->json_result = "[]";
        response->count       = 0;
        RCLCPP_ERROR(get_logger(), "[MissionDatabase] Query error: %s", e.what());
    }
}

//==============================================================================
// insertBreadcrumb
//==============================================================================

void MissionDatabaseNode::insertBreadcrumb(
    double lat, double lon, const std::string & timestamp,
    const std::optional<bool>   & has_comms,
    const std::optional<double> & heading,
    const std::optional<double> & speed,
    const std::map<std::string, std::string> & metadata)
{
    const char * sql =
        "INSERT INTO breadcrumbs (lat, lon, timestamp, has_comms, heading, speed, metadata) "
        "VALUES (?, ?, ?, ?, ?, ?, ?);";

    SqliteStmt stmt(db_, sql);
    if (!stmt) {
        RCLCPP_ERROR(get_logger(), "[MissionDatabase] INSERT prepare failed: %s",
                     sqlite3_errmsg(db_));
        return;
    }

    sqlite3_bind_double(stmt.get(), 1, lat);
    sqlite3_bind_double(stmt.get(), 2, lon);
    sqlite3_bind_text(stmt.get(),   3, timestamp.c_str(), -1, SQLITE_TRANSIENT);

    if (has_comms.has_value()) {
        sqlite3_bind_int(stmt.get(), 4, has_comms.value() ? 1 : 0);
    } else {
        sqlite3_bind_null(stmt.get(), 4);
    }

    if (heading.has_value()) {
        sqlite3_bind_double(stmt.get(), 5, *heading);
    } else {
        sqlite3_bind_null(stmt.get(), 5);
    }

    if (speed.has_value()) {
        sqlite3_bind_double(stmt.get(), 6, *speed);
    } else {
        sqlite3_bind_null(stmt.get(), 6);
    }

    // Metadata JSON
    std::ostringstream meta;
    meta << "{";
    bool first = true;
    for (const auto & [k, v] : metadata) {
        if (!first) meta << ",";
        meta << "\"" << k << "\":\"" << v << "\"";
        first = false;
    }
    meta << "}";
    const std::string meta_str = meta.str();
    sqlite3_bind_text(stmt.get(), 7, meta_str.c_str(), -1, SQLITE_TRANSIENT);

    if (sqlite3_step(stmt.get()) != SQLITE_DONE) {
        RCLCPP_ERROR(get_logger(), "[MissionDatabase] INSERT failed: %s",
                     sqlite3_errmsg(db_));
    }
}

//==============================================================================
// insertWaypoint
//==============================================================================

void MissionDatabaseNode::insertWaypoint(
    uint32_t waypoint_id, double lat, double lon,
    double target_heading, double radius,
    const std::string & name,
    const std::string & dispatched_at,
    const std::optional<bool>   & has_comms_at_dispatch,
    const std::optional<double> & compass_heading_at_dispatch)
{
    const char * sql =
        "INSERT INTO waypoints "
        "  (waypoint_id, lat, lon, heading, radius, name, dispatched_at, "
        "   has_comms_at_dispatch, compass_heading_at_dispatch, status) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, 'pending');";

    SqliteStmt stmt(db_, sql);
    if (!stmt) {
        RCLCPP_ERROR(get_logger(), "[MissionDatabase] Waypoint INSERT prepare failed: %s",
                     sqlite3_errmsg(db_));
        return;
    }

    sqlite3_bind_int(stmt.get(),    1, static_cast<int>(waypoint_id));
    sqlite3_bind_double(stmt.get(), 2, lat);
    sqlite3_bind_double(stmt.get(), 3, lon);

    // target_heading: -1.0 means "no constraint"
    if (target_heading >= 0.0) {
        sqlite3_bind_double(stmt.get(), 4, target_heading);
    } else {
        sqlite3_bind_null(stmt.get(), 4);
    }

    // radius: clamp to minimum 0.1m to guard against zero/negative values
    sqlite3_bind_double(stmt.get(), 5, std::max(0.1, radius));

    sqlite3_bind_text(stmt.get(), 6, name.c_str(),         -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt.get(), 7, dispatched_at.c_str(), -1, SQLITE_TRANSIENT);

    if (has_comms_at_dispatch.has_value()) {
        sqlite3_bind_int(stmt.get(), 8, has_comms_at_dispatch.value() ? 1 : 0);
    } else {
        sqlite3_bind_null(stmt.get(), 8);
    }

    if (compass_heading_at_dispatch.has_value()) {
        sqlite3_bind_double(stmt.get(), 9, *compass_heading_at_dispatch);
    } else {
        sqlite3_bind_null(stmt.get(), 9);
    }

    if (sqlite3_step(stmt.get()) != SQLITE_DONE) {
        RCLCPP_ERROR(get_logger(), "[MissionDatabase] Waypoint INSERT failed: %s",
                     sqlite3_errmsg(db_));
    }
}

//==============================================================================
// updateWaypointStatus
//==============================================================================

void MissionDatabaseNode::updateWaypointStatus(
    uint32_t waypoint_id, const std::string & status,
    const std::string & completed_at)
{
    const char * sql =
        "UPDATE waypoints SET status=?, completed_at=? "
        "WHERE id = ("
        "  SELECT id FROM waypoints "
        "  WHERE waypoint_id=? AND status='pending' "
        "  ORDER BY id DESC LIMIT 1"
        ");";

    SqliteStmt stmt(db_, sql);
    if (!stmt) {
        RCLCPP_ERROR(get_logger(), "[MissionDatabase] UPDATE prepare failed: %s",
                     sqlite3_errmsg(db_));
        return;
    }

    sqlite3_bind_text(stmt.get(), 1, status.c_str(),       -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt.get(), 2, completed_at.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt.get(),  3, static_cast<int>(waypoint_id));

    if (sqlite3_step(stmt.get()) != SQLITE_DONE) {
        RCLCPP_ERROR(get_logger(), "[MissionDatabase] UPDATE failed: %s",
                     sqlite3_errmsg(db_));
        return;
    }

    if (sqlite3_changes(db_) == 0) {
        RCLCPP_WARN(get_logger(),
            "[MissionDatabase] No pending row found for waypoint_id=%u status='%s'",
            waypoint_id, status.c_str());
    }
}

//==============================================================================
// evictOldestRows
//==============================================================================

void MissionDatabaseNode::evictOldestRows(int64_t count)
{
    const char * sql =
        "DELETE FROM breadcrumbs "
        "WHERE id IN (SELECT id FROM breadcrumbs ORDER BY id ASC LIMIT ?);";

    SqliteStmt stmt(db_, sql);
    if (!stmt) return;

    sqlite3_bind_int64(stmt.get(), 1, count);
    if (sqlite3_step(stmt.get()) != SQLITE_DONE) return;

    const int deleted = sqlite3_changes(db_);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        stat_crumbs_evicted_ += static_cast<uint64_t>(deleted);
    }

    RCLCPP_WARN(get_logger(),
        "[MissionDatabase] Evicted %d breadcrumb(s). Total evicted: %lu",
        deleted, stat_crumbs_evicted_);
}

//==============================================================================
// getRowCount
//==============================================================================

int64_t MissionDatabaseNode::getRowCount() const
{
    SqliteStmt stmt(db_, "SELECT COUNT(*) FROM breadcrumbs;");
    if (!stmt) return -1;
    if (sqlite3_step(stmt.get()) == SQLITE_ROW) {
        return sqlite3_column_int64(stmt.get(), 0);
    }
    return -1;
}

//==============================================================================
// enforceStorageLimit
//==============================================================================

void MissionDatabaseNode::enforceStorageLimit()
{
    sqlite3_wal_checkpoint_v2(db_, nullptr, SQLITE_CHECKPOINT_PASSIVE,
                              nullptr, nullptr);

    std::error_code ec;
    const auto db_size = std::filesystem::file_size(db_path_, ec);
    if (ec || db_size <= max_db_size_bytes_) return;

    while (true) {
        const auto current = std::filesystem::file_size(db_path_, ec);
        if (ec || current <= max_db_size_bytes_) break;
        // getRowCount() returns -1 on DB error -- treat as empty to avoid
        // an infinite eviction loop when the DB itself has a problem.
        if (getRowCount() <= 0) break;
        evictOldestRows(EVICTION_BATCH);
        sqlite3_wal_checkpoint_v2(db_, nullptr, SQLITE_CHECKPOINT_PASSIVE,
                                  nullptr, nullptr);
    }
}

//==============================================================================
// queryBreadcrumbsJson
//==============================================================================

std::string MissionDatabaseNode::queryBreadcrumbsJson(
    int32_t limit, bool filter_comms, bool comms_val) const
{
    std::string sql;

    if (limit <= 0) {
        sql = "SELECT lat, lon, timestamp, has_comms, heading, speed, metadata "
              "FROM breadcrumbs";
        if (filter_comms) sql += comms_val ? " WHERE has_comms=1" : " WHERE has_comms=0";
        sql += " ORDER BY id ASC;";
    } else {
        std::string inner =
            "SELECT lat, lon, timestamp, has_comms, heading, speed, metadata FROM breadcrumbs";
        if (filter_comms) inner += comms_val ? " WHERE has_comms=1" : " WHERE has_comms=0";
        inner += " ORDER BY id DESC LIMIT " + std::to_string(limit);
        sql = "SELECT lat, lon, timestamp, has_comms, heading, speed, metadata "
              "FROM (" + inner + ") ORDER BY rowid ASC;";
    }

    SqliteStmt stmt(db_, sql.c_str());
    if (!stmt) return "[]";

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9);
    oss << "[";
    bool first = true;

    while (sqlite3_step(stmt.get()) == SQLITE_ROW) {
        if (!first) oss << ",";

        const double lat = sqlite3_column_double(stmt.get(), 0);
        const double lon = sqlite3_column_double(stmt.get(), 1);
        const char * ts  = reinterpret_cast<const char *>(
            sqlite3_column_text(stmt.get(), 2));

        const bool comms_null = (sqlite3_column_type(stmt.get(), 3) == SQLITE_NULL);
        const bool comms_val2 = comms_null ? false : (sqlite3_column_int(stmt.get(), 3) != 0);

        const bool hdg_null = (sqlite3_column_type(stmt.get(), 4) == SQLITE_NULL);
        const double hdg    = hdg_null ? 0.0 : sqlite3_column_double(stmt.get(), 4);

        const bool spd_null = (sqlite3_column_type(stmt.get(), 5) == SQLITE_NULL);
        const double spd    = spd_null ? 0.0 : sqlite3_column_double(stmt.get(), 5);

        const char * meta = reinterpret_cast<const char *>(
            sqlite3_column_text(stmt.get(), 6));

        oss << "{\"lat\":"          << lat                   << ","
            << "\"lon\":"           << lon                   << ","
            << "\"timestamp\":\""  << (ts ? ts : "")         << "\","
            << "\"has_comms\":";
        if (comms_null) oss << "null"; else oss << (comms_val2 ? "true" : "false");
        oss << ",\"heading\":";
        if (hdg_null) oss << "null"; else oss << hdg;
        oss << ",\"speed\":";
        if (spd_null) oss << "null"; else oss << std::fixed << std::setprecision(3) << spd;
        oss << std::fixed << std::setprecision(9);  // restore precision for next row
        oss << ",\"metadata\":"     << (meta ? meta : "{}") << "}";

        first = false;
    }

    oss << "]";
    return oss.str();
}

//==============================================================================
// publishTrail
//==============================================================================

void MissionDatabaseNode::publishTrail()
{
    auto msg = std_msgs::msg::String{};
    msg.data = queryBreadcrumbsJson(publish_window_size_);
    trail_pub_->publish(msg);
}

//==============================================================================
// publishStats
//==============================================================================

void MissionDatabaseNode::publishStats()
{
    std::error_code ec;
    const auto sz = std::filesystem::file_size(db_path_, ec);
    const double mb = ec ? -1.0 : static_cast<double>(sz) / (1024.0 * 1024.0);

    uint64_t gps_rx, gps_skip, rec, evict;
    bool comms_known, comms_val;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        gps_rx    = stat_gps_received_;
        gps_skip  = stat_gps_skipped_dedup_;
        rec       = stat_crumbs_recorded_;
        evict     = stat_crumbs_evicted_;
        comms_known = current_has_comms_.has_value();
        comms_val   = current_has_comms_.value_or(false);
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "{\"robot_name\":\"" << robot_name_ << "\","
        << "\"db_row_count\":"  << getRowCount()    << ","
        << "\"db_size_mb\":"    << mb               << ","
        << "\"db_limit_mb\":"
        << static_cast<double>(max_db_size_bytes_) / (1024.0*1024.0) << ","
        << "\"gps_received\":"  << gps_rx           << ","
        << "\"gps_deduped\":"   << gps_skip         << ","
        << "\"crumbs_total\":"  << rec              << ","
        << "\"crumbs_evicted\":"<< evict            << ","
        << "\"current_has_comms\":";
    if (comms_known) oss << (comms_val ? "true" : "false"); else oss << "null";
    oss << "}";

    auto out = std_msgs::msg::String{};
    out.data = oss.str();
    stats_pub_->publish(out);
}

//==============================================================================
// publishWaypointList
//==============================================================================

void MissionDatabaseNode::publishWaypointList()
{
    const char * sql =
        "SELECT waypoint_id, lat, lon, heading, radius, name, dispatched_at, "
        "       has_comms_at_dispatch, compass_heading_at_dispatch, "
        "       status, completed_at "
        "FROM waypoints ORDER BY id ASC;";

    SqliteStmt stmt(db_, sql);
    if (!stmt) return;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9);
    std::string rows;
    int count = 0;

    auto col_str = [&](int c) -> std::string {
        const char * p = reinterpret_cast<const char *>(
            sqlite3_column_text(stmt.get(), c));
        return p ? p : "";
    };
    auto col_null = [&](int c) { return sqlite3_column_type(stmt.get(), c) == SQLITE_NULL; };

    while (sqlite3_step(stmt.get()) == SQLITE_ROW) {
        if (count > 0) rows += ",";

        std::ostringstream row;
        row << std::fixed << std::setprecision(9);
        row << "{"
            << "\"waypoint_id\":"              << sqlite3_column_int(stmt.get(), 0)    << ","
            << "\"lat\":"                      << sqlite3_column_double(stmt.get(), 1) << ","
            << "\"lon\":"                      << sqlite3_column_double(stmt.get(), 2) << ","
            << "\"target_heading\":";
        if (col_null(3)) row << "null"; else row << sqlite3_column_double(stmt.get(), 3);
        row << ","
            << "\"radius\":"                   << sqlite3_column_double(stmt.get(), 4) << ","
            << "\"name\":\""                   << col_str(5) << "\","
            << "\"dispatched_at\":\""          << col_str(6) << "\","
            << "\"has_comms_at_dispatch\":";
        if (col_null(7)) row << "null";
        else row << (sqlite3_column_int(stmt.get(), 7) ? "true" : "false");
        row << ","
            << "\"compass_heading_at_dispatch\":";
        if (col_null(8)) row << "null"; else row << sqlite3_column_double(stmt.get(), 8);
        row << ","
            << "\"status\":\""                 << col_str(9) << "\","
            << "\"completed_at\":";
        const std::string ca = col_str(10);
        if (ca.empty()) row << "null"; else row << "\"" << ca << "\"";
        row << "}";

        rows += row.str();
        ++count;
    }

    auto out = std_msgs::msg::String{};
    out.data = "{\"robot_name\":\"" + robot_name_ + "\","
             + "\"waypoint_count\":" + std::to_string(count) + ","
             + "\"waypoints\":[" + rows + "]}";
    waypoints_pub_->publish(out);
}

//==============================================================================
// setHomePosition / publishHomePosition
//==============================================================================

void MissionDatabaseNode::setHomePosition(
    double lat, double lon,
    const std::optional<double> & heading,
    const std::string & source)
{
    if (!std::isfinite(lat) || !std::isfinite(lon)) return;

    const std::string now = formatTimestamp(
        static_cast<int32_t>(this->now().seconds()),
        static_cast<uint32_t>(this->now().nanoseconds() % 1'000'000'000ULL));

    const char * sql =
        "INSERT OR REPLACE INTO home_position (id, lat, lon, heading, timestamp, set_by) "
        "VALUES (1, ?, ?, ?, ?, ?);";

    SqliteStmt stmt(db_, sql);
    if (!stmt) return;

    sqlite3_bind_double(stmt.get(), 1, lat);
    sqlite3_bind_double(stmt.get(), 2, lon);

    if (heading.has_value()) {
        sqlite3_bind_double(stmt.get(), 3, *heading);
    } else {
        sqlite3_bind_null(stmt.get(), 3);
    }

    sqlite3_bind_text(stmt.get(), 4, now.c_str(),    -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt.get(), 5, source.c_str(), -1, SQLITE_TRANSIENT);

    if (sqlite3_step(stmt.get()) != SQLITE_DONE) {
        RCLCPP_ERROR(get_logger(), "[MissionDatabase] Home INSERT failed: %s",
                     sqlite3_errmsg(db_));
        return;
    }

    RCLCPP_INFO(get_logger(),
        "[MissionDatabase] Home set: (%.9f, %.9f) heading=%s source=%s",
        lat, lon,
        heading.has_value()
            ? (std::to_string(static_cast<int>(*heading)) + "deg").c_str()
            : "not set",
        source.c_str());

    publishHomePosition();
    publishRecoveryNavBundle();
}

void MissionDatabaseNode::publishHomePosition()
{
    const char * sql =
        "SELECT lat, lon, heading, timestamp, set_by FROM home_position WHERE id=1;";
    SqliteStmt stmt(db_, sql);

    std_msgs::msg::String out;
    if (stmt && sqlite3_step(stmt.get()) == SQLITE_ROW) {
        const double lat = sqlite3_column_double(stmt.get(), 0);
        const double lon = sqlite3_column_double(stmt.get(), 1);

        const bool   hdg_null = (sqlite3_column_type(stmt.get(), 2) == SQLITE_NULL);
        const double hdg      = hdg_null ? 0.0 : sqlite3_column_double(stmt.get(), 2);

        auto col = [&](int c) -> std::string {
            const char * p = reinterpret_cast<const char *>(
                sqlite3_column_text(stmt.get(), c));
            return p ? p : "";
        };

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(9);
        oss << "{\"valid\":true,"
            << "\"lat\":"         << lat    << ","
            << "\"lon\":"         << lon    << ","
            << "\"heading\":";
        if (hdg_null) oss << "null"; else oss << hdg;
        oss << ","
            << "\"timestamp\":\"" << col(3) << "\","
            << "\"set_by\":\""    << col(4) << "\"}";
        out.data = oss.str();
    } else {
        out.data = "{\"valid\":false}";
        RCLCPP_INFO_ONCE(get_logger(),
            "[MissionDatabase] Home not set. "
            "Use home_lat/home_lon/home_heading params or: "
            "ros2 param set /%s/mission_database_node home_lat <lat>",
            robot_name_.c_str());
    }
    home_pos_pub_->publish(out);
}

//==============================================================================
// updateAndPublishLastCommsPosition
//==============================================================================

void MissionDatabaseNode::updateAndPublishLastCommsPosition()
{
    const char * sql =
        "SELECT lat, lon, timestamp FROM breadcrumbs "
        "WHERE has_comms=1 ORDER BY id DESC LIMIT 1;";
    SqliteStmt stmt(db_, sql);

    std_msgs::msg::String out;
    if (stmt && sqlite3_step(stmt.get()) == SQLITE_ROW) {
        const double lat = sqlite3_column_double(stmt.get(), 0);
        const double lon = sqlite3_column_double(stmt.get(), 1);
        const char * ts  = reinterpret_cast<const char *>(
            sqlite3_column_text(stmt.get(), 2));

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_comms_lat_ = lat;
            last_comms_lon_ = lon;
        }

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(9);
        oss << "{\"valid\":true,"
            << "\"lat\":"         << lat              << ","
            << "\"lon\":"         << lon              << ","
            << "\"timestamp\":\"" << (ts ? ts : "")   << "\"}";
        out.data = oss.str();

        RCLCPP_INFO(get_logger(),
            "[MissionDatabase] Last comms position: (%.9f, %.9f)", lat, lon);
    } else {
        out.data = "{\"valid\":false}";
    }

    last_comms_pos_pub_->publish(out);
    publishRecoveryNavBundle();
}

//==============================================================================
// publishRecoveryNavBundle
//
// Assembles: home + last_comms_position + reached waypoints (most-recent first)
// into one JSON message for use by recovery navigation BT behaviours.
//
// JSON structure:
//   {
//     "robot_name": "warthog1",
//     "home": { "valid":true, "lat":..., "lon":..., "timestamp":..., "set_by":... },
//     "last_comms_position": { "valid":true, "lat":..., "lon":..., "timestamp":... },
//     "return_waypoints": [    <- reached only, most-recent first
//       { "waypoint_id":3, "lat":..., "lon":..., "target_heading":...,
//         "name":..., "reached_at":..., "had_comms":...,
//         "compass_heading_at_dispatch":... },
//       ...
//     ]
//   }
//==============================================================================

void MissionDatabaseNode::publishRecoveryNavBundle()
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9);
    oss << "{\"robot_name\":\"" << robot_name_ << "\",";

    // ── HOME ──────────────────────────────────────────────────────────────────
    {
        SqliteStmt s(db_,
            "SELECT lat, lon, heading, timestamp, set_by FROM home_position WHERE id=1;");
        oss << "\"home\":";
        if (s && sqlite3_step(s.get()) == SQLITE_ROW) {
            const double lat = sqlite3_column_double(s.get(), 0);
            const double lon = sqlite3_column_double(s.get(), 1);
            const bool   hdg_null = (sqlite3_column_type(s.get(), 2) == SQLITE_NULL);
            const double hdg      = hdg_null ? 0.0 : sqlite3_column_double(s.get(), 2);
            auto col = [&](int c) -> std::string {
                const char * p = reinterpret_cast<const char *>(
                    sqlite3_column_text(s.get(), c));
                return p ? p : "";
            };
            oss << "{\"valid\":true,"
                << "\"lat\":"         << lat    << ","
                << "\"lon\":"         << lon    << ","
                << "\"heading\":";
            if (hdg_null) oss << "null"; else oss << hdg;
            oss << ","
                << "\"timestamp\":\"" << col(3) << "\","
                << "\"set_by\":\""    << col(4) << "\"}";
        } else {
            oss << "{\"valid\":false}";
        }
    }

    oss << ",";

    // ── LAST COMMS POSITION ───────────────────────────────────────────────────
    {
        SqliteStmt s(db_,
            "SELECT lat, lon, timestamp FROM breadcrumbs "
            "WHERE has_comms=1 ORDER BY id DESC LIMIT 1;");
        oss << "\"last_comms_position\":";
        if (s && sqlite3_step(s.get()) == SQLITE_ROW) {
            const double lat = sqlite3_column_double(s.get(), 0);
            const double lon = sqlite3_column_double(s.get(), 1);
            const char * ts  = reinterpret_cast<const char *>(
                sqlite3_column_text(s.get(), 2));
            oss << "{\"valid\":true,"
                << "\"lat\":"         << lat              << ","
                << "\"lon\":"         << lon              << ","
                << "\"timestamp\":\"" << (ts ? ts : "")   << "\"}";
        } else {
            oss << "{\"valid\":false}";
        }
    }

    oss << ",";

    // ── RETURN WAYPOINTS (reached only, most-recent first) ────────────────────
    {
        SqliteStmt s(db_,
            "SELECT waypoint_id, lat, lon, heading, radius, name, completed_at, "
            "       has_comms_at_dispatch, compass_heading_at_dispatch "
            "FROM waypoints WHERE status='reached' ORDER BY id DESC;");

        oss << "\"return_waypoints\":[";
        bool first = true;

        if (s) {
            while (sqlite3_step(s.get()) == SQLITE_ROW) {
                if (!first) oss << ",";

                auto col_null = [&](int c) {
                    return sqlite3_column_type(s.get(), c) == SQLITE_NULL;
                };
                auto col_str = [&](int c) -> std::string {
                    const char * p = reinterpret_cast<const char *>(
                        sqlite3_column_text(s.get(), c));
                    return p ? p : "";
                };

                oss << "{"
                    << "\"waypoint_id\":"              << sqlite3_column_int(s.get(), 0)    << ","
                    << "\"lat\":"                      << sqlite3_column_double(s.get(), 1) << ","
                    << "\"lon\":"                      << sqlite3_column_double(s.get(), 2) << ","
                    << "\"target_heading\":";
                if (col_null(3)) oss << "null"; else oss << sqlite3_column_double(s.get(), 3);
                oss << ","
                    << "\"radius\":"                   << sqlite3_column_double(s.get(), 4) << ","
                    << "\"name\":\""    << col_str(5) << "\","
                    << "\"reached_at\":\"" << col_str(6) << "\","
                    << "\"had_comms\":";
                if (col_null(7)) oss << "null";
                else oss << (sqlite3_column_int(s.get(), 7) ? "true" : "false");
                oss << ","
                    << "\"compass_heading_at_dispatch\":";
                if (col_null(8)) oss << "null"; else oss << sqlite3_column_double(s.get(), 8);
                oss << "}";

                first = false;
            }
        }

        oss << "]";
    }

    oss << "}";

    auto out = std_msgs::msg::String{};
    out.data = oss.str();
    recovery_nav_pub_->publish(out);
}

//==============================================================================
// STATIC UTILITIES
//==============================================================================

bool MissionDatabaseNode::hasBaseInTransitive(
    const std::vector<std::string> & transitive)
{
    return std::find(transitive.begin(), transitive.end(), "base_station")
           != transitive.end();
}

double MissionDatabaseNode::haversineDistance(
    double lat1, double lon1, double lat2, double lon2)
{
    const double phi1     = lat1 * DEG_TO_RAD;
    const double phi2     = lat2 * DEG_TO_RAD;
    const double d_phi    = (lat2 - lat1) * DEG_TO_RAD;
    const double d_lambda = (lon2 - lon1) * DEG_TO_RAD;
    const double a = std::sin(d_phi/2) * std::sin(d_phi/2)
                   + std::cos(phi1) * std::cos(phi2)
                   * std::sin(d_lambda/2) * std::sin(d_lambda/2);
    return EARTH_RADIUS_M * 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0-a));
}

}  // namespace mission_database

//==============================================================================
// MAIN
//==============================================================================

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mission_database::MissionDatabaseNode>());
    rclcpp::shutdown();
    return 0;
}
