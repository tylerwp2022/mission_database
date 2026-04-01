# mission_database

A ROS2 (Jazzy) package for the Warthog UGV simulation that maintains a persistent, on-disk record of robot mission data. Designed to run continuously alongside the robot stack and provide recovery navigation data when operating in comms-denied areas.

## Features

- **Breadcrumb trail** — GPS positions recorded every 8 metres, each annotated with comms status, compass heading, and GPS speed
- **Waypoint tracking** — BT nodes publish waypoint dispatch/completion events; the database tracks status and comms at each waypoint
- **Home position** — manually configured per robot, persists across restarts
- **Recovery navigation bundle** — a single topic containing home position, last known comms position, and confirmed-reached waypoints in reverse order; everything a BT behaviour needs to navigate back when comms drops
- **SQLite on disk** — near-zero RAM footprint; data survives node crashes; queryable with any SQLite tool
- **TAK replay** — standalone script replays a mission database to ATAK with correct timing, comms-aware colouring, pause/resume controls, and optional cleanup

---

## Package Structure

```
mission_database/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── mission_database.launch.py
├── msg/
│   └── WaypointEvent.msg
├── srv/
│   └── QueryBreadcrumbs.srv
├── include/
│   └── mission_database/
│       └── mission_database_node.hpp
├── src/
│   └── mission_database_node.cpp
└── scripts/
    └── db_tak_replay.py
```

---

## Dependencies

| Dependency | Notes |
|---|---|
| `rclcpp` | ROS2 C++ client library |
| `sensor_msgs` | `NavSatFix` for GPS |
| `std_msgs` | `String` (JSON topics and compass) |
| `west_point_comms_sim` | `CommsStatus` message type |
| `libsqlite3-dev` | System library — `sudo apt install libsqlite3-dev` |

---

## Building

```bash
cd your_ws
sudo apt install libsqlite3-dev
colcon build --packages-select mission_database
source install/setup.bash
```

---

## Running

### Launch file (recommended)

```bash
ros2 launch mission_database mission_database.launch.py \
    robot_names:=warthog1,warthog2,warthog3
```

Per-robot home positions are pre-configured in `ROBOT_HOME_DEFAULTS` at the top of the launch file. Edit those coordinates before deploying.

### Override home position from command line

```bash
ros2 launch mission_database mission_database.launch.py \
    robot_names:=warthog1 \
    home_lat:=39.35260 \
    home_lon:=-76.34522 \
    home_heading:=140.0
```

### Update home position at runtime (no restart needed)

```bash
ros2 param set /warthog1/mission_database_node home_lat 39.35260
ros2 param set /warthog1/mission_database_node home_lon -76.34522
ros2 param set /warthog1/mission_database_node home_heading 140.0
```

---

## Launch Parameters

| Parameter | Default | Description |
|---|---|---|
| `robot_names` | `warthog1` | Comma-separated list of robot names |
| `home_lat` | `nan` | Override home latitude (uses per-robot default if nan) |
| `home_lon` | `nan` | Override home longitude |
| `home_heading` | `nan` | Override home heading (degrees 0–360) |
| `min_distance_m` | `8.0` | Minimum metres between breadcrumbs |
| `max_db_size_mb` | `10.0` | DB size limit before oldest breadcrumbs are evicted |
| `publish_window_size` | `100` | Breadcrumbs included in the trail topic |
| `publish_rate_hz` | `1.0` | Periodic re-publish rate |
| `db_dir` | `/phoenix/src/utils/mission_database/database` | Directory for `.db` files |
| `debug` | `false` | Verbose logging |

---

## Topics

### Subscriptions

| Topic | Type | Purpose |
|---|---|---|
| `/{robot}/sensors/ublox/fix` | `sensor_msgs/NavSatFix` | GPS position |
| `/{robot}/comms` | `west_point_comms_sim/CommsStatus` | Mesh radio connectivity |
| `/{robot}/compass` | `std_msgs/String` | Heading in degrees 0–360 when calibrated |
| `/{robot}/mission_database/waypoint_event` | `mission_database/WaypointEvent` | BT waypoint dispatch/completion |

### Publications (all `transient_local` / latched)

| Topic | Type | Description |
|---|---|---|
| `/{robot}/mission_database/breadcrumb_trail` | `std_msgs/String` (JSON) | Rolling window of last 100 breadcrumbs |
| `/{robot}/mission_database/waypoints` | `std_msgs/String` (JSON) | All waypoints with status |
| `/{robot}/mission_database/recovery_nav` | `std_msgs/String` (JSON) | Home + last comms pos + return waypoints |
| `/{robot}/mission_database/last_comms_position` | `std_msgs/String` (JSON) | Most recent breadcrumb with comms |
| `/{robot}/mission_database/home_position` | `std_msgs/String` (JSON) | Stored home location |
| `/{robot}/mission_database/stats` | `std_msgs/String` (JSON) | Diagnostics and DB size |

### Service

| Service | Type | Description |
|---|---|---|
| `/{robot}/mission_database/query` | `mission_database/QueryBreadcrumbs` | Query breadcrumbs with optional filters |

---

## WaypointEvent Message

BT nodes publish to `/{robot}/mission_database/waypoint_event` to record navigation objectives.

### Message fields

| Field | Type | Description |
|---|---|---|
| `header` | `std_msgs/Header` | Standard ROS header (stamp, frame_id) |
| `event_type` | `uint8` | `DISPATCHED=0`, `REACHED=1`, `FAILED=2` |
| `waypoint_id` | `uint32` | Unique ID within the mission session — must match between DISPATCHED and the subsequent REACHED/FAILED |
| `lat` | `float64` | Target latitude (required for DISPATCHED; ignored for REACHED/FAILED) |
| `lon` | `float64` | Target longitude (required for DISPATCHED; ignored for REACHED/FAILED) |
| `heading` | `float64` | Desired heading at the waypoint in degrees 0–360. Set to `-1.0` if heading is unconstrained |
| `radius` | `float64` | Acceptance radius in metres — robot is considered arrived when within this distance. Default `2.0` m |
| `name` | `string` | Optional human-readable label (e.g. `"alpha"`, `"checkpoint_3"`) |

### Example usage

```cpp
// When sending a waypoint to navigation:
auto evt        = mission_database::msg::WaypointEvent{};
evt.header.stamp = this->now();
evt.event_type  = WaypointEvent::DISPATCHED;
evt.waypoint_id = objective_id;   // unique ID for correlation
evt.lat         = target_lat;
evt.lon         = target_lon;
evt.heading     = target_heading; // degrees 0-360, or -1.0 if unconstrained
evt.radius      = 2.0;            // metres
evt.name        = "checkpoint_alpha";
waypoint_event_pub_->publish(evt);

// On arrival or failure:
auto done_evt        = mission_database::msg::WaypointEvent{};
done_evt.header.stamp = this->now();
done_evt.event_type  = WaypointEvent::REACHED;  // or FAILED
done_evt.waypoint_id = objective_id;            // same ID as DISPATCHED
waypoint_event_pub_->publish(done_evt);
```

### Retry behaviour

If a waypoint fails and is retried with the same `waypoint_id`, the database updates the most recent pending row rather than creating a duplicate. This means a failed-then-retried waypoint that eventually succeeds will show a single `reached` entry in the database.

---

## Recovery Navigation Bundle

The `recovery_nav` topic publishes a JSON object that gives a BT behaviour everything needed to navigate back to comms and then home:

```json
{
  "robot_name": "warthog1",
  "home": {
    "valid": true,
    "lat": 39.35260,
    "lon": -76.34522,
    "heading": 140.0,
    "timestamp": "2024-03-15 14:20:00.000 UTC",
    "set_by": "parameter"
  },
  "last_comms_position": {
    "valid": true,
    "lat": 39.35247,
    "lon": -76.34510,
    "timestamp": "2024-03-15 14:25:07.042 UTC"
  },
  "return_waypoints": [
    {
      "waypoint_id": 3,
      "lat": 39.35241,
      "lon": -76.34505,
      "target_heading": 140.0,
      "name": "obj_3",
      "reached_at": "2024-03-15 14:24:50.000 UTC",
      "had_comms": true,
      "compass_heading_at_dispatch": 138.5
    }
  ]
}
```

`return_waypoints` contains only confirmed-reached waypoints, ordered most-recent first (index 0 = closest to current position). The suggested recovery sequence is:

1. Navigate to `last_comms_position` — re-establish comms
2. Step through `return_waypoints` in order — retrace the path
3. Navigate to `home` — final destination

---

## Database Files

Each launch session creates a timestamped SQLite file per robot:

```
/phoenix/src/utils/mission_database/database/
├── warthog1_2024-03-15_14-23-07.db
├── warthog2_2024-03-15_14-23-07.db
└── warthog3_2024-03-15_14-23-07.db
```

You can query any database file directly while the node is running (SQLite supports concurrent reads):

```bash
# Last 10 breadcrumbs
sqlite3 /phoenix/src/utils/mission_database/database/warthog1_2024-03-15_14-23-07.db \
  "SELECT timestamp, lat, lon, has_comms, heading, speed FROM breadcrumbs ORDER BY id DESC LIMIT 10;"

# All waypoints and status
sqlite3 /phoenix/src/utils/mission_database/database/warthog1_2024-03-15_14-23-07.db \
  "SELECT name, status, has_comms_at_dispatch, compass_heading_at_dispatch FROM waypoints;"

# Home position
sqlite3 /phoenix/src/utils/mission_database/database/warthog1_2024-03-15_14-23-07.db \
  "SELECT lat, lon, heading, set_by FROM home_position;"
```

---

## TAK Replay

Replays a database file to ATAK by publishing CoT messages on `/{robot}/send_to_tak`. All events — breadcrumbs, waypoint dispatches, and waypoint completions — are replayed in a single unified timeline with correct inter-event timing.

### Comms-aware colouring

Breadcrumb spot markers are coloured based on the `has_comms` value recorded at that timestamp:

| Comms state | ATAK spot colour | Terminal label |
|---|---|---|
| `has_comms = 1` (link up) | White | `comms=OK` |
| `has_comms = 0` (link lost) | Red | `comms=LOST` |
| `has_comms = NULL` (unknown) | White | `comms=?` |

If the database has no `has_comms` data the comms column is omitted from the terminal output entirely.

### Waypoint events

Waypoint dispatch and completion events appear as numbered entries in the event stream rather than as side-effects of breadcrumb processing. The total event count includes breadcrumbs and waypoint updates:

```
Starting replay -- 17 breadcrumb(s), 2 waypoint event(s).  (comms status loaded)

[PLAYING] [   1/19]  2026-04-01 22:19:49 UTC  (39.35163, -76.34435)  hdg=n/a  spd=0.00m/s*  comms=OK
  Next in 17.8s (real gap: 17.8s  speed: 1.0x)  [Space/p = pause  q = quit]
[PLAYING] [   2/19]  2026-04-01 22:20:07 UTC  -> WP0 dispatched (yellow)
  Next in 10.7s (real gap: 10.7s  speed: 1.0x)  [Space/p = pause  q = quit]
...
[PLAYING] [  18/19]  2026-04-01 22:21:29 UTC  (39.35257, -76.34520)  hdg=324.1deg  spd=1.64m/s  comms=OK
  Next in 2.8s (real gap: 2.8s  speed: 1.0x)  [Space/p = pause  q = quit]
[PLAYING] [  19/19]  2026-04-01 22:21:32 UTC  -> WP0 reached (green)

Replay complete.  17 breadcrumb(s), 2 waypoint event(s) sent.
```

Waypoint completion events whose `completed_at` timestamp falls after the last breadcrumb (common when the robot reaches a waypoint near the end of recording) are appended at the correct timestamp and fire after a real timed gap — they are not a silent post-loop flush.

A `*` suffix on speed (e.g. `spd=0.00m/s*`) means GPS speed was not yet available when that breadcrumb was recorded; `0.0` is used as a fallback.

### Running

```bash
# Real-time replay
ros2 run mission_database db_tak_replay.py \
    /phoenix/src/utils/mission_database/database/warthog1_2024-03-15_14-23-07.db

# 4x speed
ros2 run mission_database db_tak_replay.py \
    /phoenix/src/utils/mission_database/database/warthog1_2024-03-15_14-23-07.db \
    --speed 4.0

# Plot all points instantly
ros2 run mission_database db_tak_replay.py \
    /phoenix/src/utils/mission_database/database/warthog1_2024-03-15_14-23-07.db \
    --instant
```

**Terminal controls** (timed mode): `Space`/`p` = pause/resume, `q` = quit

After replay completes you will be prompted whether to send delete CoT commands to remove all symbols (breadcrumbs, home marker, waypoint circles) from ATAK.

### Replay arguments

| Argument | Default | Description |
|---|---|---|
| `db_path` | required | Path to `.db` file |
| `--robot_name` | inferred from filename | Robot namespace |
| `--speed` | `1.0` | Playback speed multiplier |
| `--instant` | off | Send all events immediately with no timing delays |
| `--track_speed` | `0.0` | Fallback CoT `<track speed>` in m/s for breadcrumbs with no recorded GPS speed |
| `--skip_home` | off | Skip the home position CoT marker |

---

## Compass Topic

The node subscribes to `/{robot}/compass` (`std_msgs/String`). When calibrated, publish a numeric string e.g. `"142.5"` (degrees 0–360, clockwise from north). Non-numeric values like `"uncalibrated"` are ignored gracefully — `current_heading_` stays `nullopt` and breadcrumbs record `NULL` for heading until a valid reading arrives. When calibration is regained, heading recording resumes automatically.

---

## Database Schema

```sql
CREATE TABLE breadcrumbs (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    lat       REAL    NOT NULL,
    lon       REAL    NOT NULL,
    timestamp TEXT    NOT NULL,   -- "2024-03-15 14:23:07.042 UTC"
    has_comms INTEGER,            -- NULL=unknown, 0=no, 1=yes
    heading   REAL,               -- NULL=unknown, degrees 0-360 from compass
    speed     REAL,               -- NULL=unknown, m/s from GPS speed topic
    metadata  TEXT    NOT NULL DEFAULT '{}'
);

CREATE TABLE home_position (
    id        INTEGER PRIMARY KEY CHECK (id = 1),
    lat       REAL    NOT NULL,
    lon       REAL    NOT NULL,
    heading   REAL,               -- NULL=not set
    timestamp TEXT    NOT NULL,
    set_by    TEXT    NOT NULL    -- "parameter" | "runtime_update"
);

CREATE TABLE waypoints (
    id                          INTEGER PRIMARY KEY AUTOINCREMENT,
    waypoint_id                 INTEGER NOT NULL,
    lat                         REAL    NOT NULL,
    lon                         REAL    NOT NULL,
    radius                      REAL    NOT NULL DEFAULT 2.0,  -- arrival radius in metres
    heading                     REAL,   -- target heading from WaypointEvent
    name                        TEXT    NOT NULL DEFAULT '',
    dispatched_at               TEXT    NOT NULL,
    has_comms_at_dispatch       INTEGER,
    compass_heading_at_dispatch REAL,
    status                      TEXT    NOT NULL DEFAULT 'pending',
    completed_at                TEXT
);
```
