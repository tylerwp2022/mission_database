# mission_database

A ROS2 Jazzy package that maintains a persistent, on-disk SQLite record of robot mission data — breadcrumbs, waypoints, and home position — designed to support recovery navigation when operating in comms-denied areas.

## PETAAR26 Integration

This node is part of the PETAAR stack. Several previously hardcoded values are now driven by shared config:

| Was hardcoded | Now from | File |
|---|---|---|
| GPS topic `sensors/geofog/gps/fix` | `hardware.gps_topic_suffix` | `petaar26/config/hardware.yaml` |
| Database directory path | `paths.mission_db_dir` | `petaar26/config/paths.yaml` |
| Per-robot home positions | `robot_homes` dict | `petaar26/config/robot_homes.yaml` |

**To change the GPS topic** (e.g., when deploying on new hardware):
```
petaar26/config/hardware.yaml → hardware.gps_topic_suffix
```

**To change robot home positions:**
```
petaar26/config/robot_homes.yaml
```

**To change the database directory:**
```
petaar26/config/paths.yaml → paths.mission_db_dir
```

> **Bug fix (applied):** GPS subscriber topic was hardcoded to `sensors/geofog/gps/fix`. On NAI_3/NAI_4 (u-blox GPS), the node received no position data — no breadcrumbs were recorded and the recovery nav bundle was empty for those conditions.

---

## Features

- **Breadcrumb trail** — GPS positions recorded every 8 metres, annotated with comms status, compass heading, and GPS speed
- **Waypoint tracking** — BT nodes publish dispatch/completion events; the database tracks status and comms at each waypoint
- **Home position** — configured per robot in `robot_homes.yaml`, persists across restarts
- **Recovery navigation bundle** — single topic with home, last-comms position, and confirmed-reached waypoints in reverse order
- **SQLite on disk** — near-zero RAM footprint, survives node crashes, queryable with any SQLite tool
- **TAK replay** — standalone script (`db_tak_replay.py`) replays a mission database to ATAK with correct timing and comms-aware colouring

---

## Topics

### Subscriptions

| Topic | Type | Description |
|---|---|---|
| `/{robot}/{gps_topic_suffix}` | `sensor_msgs/NavSatFix` | GPS position. Suffix from `hardware.yaml` |
| `/{robot}/comms` | `west_point_comms_sim/CommsStatus` | Mesh connectivity (**sim only** — see below) |
| `/{robot}/compass` | `std_msgs/Float64` | Heading `[0,360)` when calibrated, `-1.0` when not |
| `/{robot}/sensors/gps_speed` | `std_msgs/Float64` | Ground speed from `gps_speed_node` |
| `/{robot}/mission_database/waypoint_event` | `mission_database/WaypointEvent` | Waypoint dispatch/completion from BT nodes |

> **⚠ Comms topic is sim-only.** On a real robot, `west_point_comms_sim_node` does not run. An adapter node that publishes `CommsStatus` from real mesh radio data will be needed. Until then, `has_comms` will be `NULL` in all breadcrumbs and waypoints on real hardware. See `petaar26/config/hardware.yaml → comms_status_topic` for details.

### Publications (all `transient_local` / latched)

| Topic | Type | Description |
|---|---|---|
| `/{robot}/mission_database/breadcrumb_trail` | `std_msgs/String` (JSON) | Rolling window of recent breadcrumbs |
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

## Launch Parameters

| Parameter | Default | Source | Description |
|---|---|---|---|
| `robot_names` | `warthog1` | launch arg | Comma-separated robot names |
| `gps_topic_suffix` | from `hardware.yaml` | `hardware.gps_topic_suffix` | GPS topic suffix |
| `home_lat` | `nan` | launch arg | Override home latitude for all robots (nan = use robot_homes.yaml) |
| `home_lon` | `nan` | launch arg | Override home longitude |
| `home_heading` | `nan` | launch arg | Override home heading (degrees 0–360) |
| `min_distance_m` | `8.0` | launch arg | Minimum metres between breadcrumbs |
| `max_db_size_mb` | `10.0` | launch arg | DB size limit before oldest breadcrumbs evicted |
| `publish_window_size` | `100` | launch arg | Breadcrumbs in trail topic |
| `publish_rate_hz` | `1.0` | launch arg | Periodic re-publish rate |
| `db_dir` | from `paths.yaml` | `paths.mission_db_dir` | Root directory for run folders |
| `debug` | `false` | launch arg | Verbose logging |

**Home position priority:** command-line args > `robot_homes.yaml` > `float('nan')` (runtime update via `ros2 param set`).

---

## Usage

```bash
# All robots (homes from robot_homes.yaml, GPS from hardware.yaml)
ros2 launch mission_database mission_database.launch.py \
    robot_names:=warthog1,warthog2,warthog3

# Override home for a specific deployment site
ros2 launch mission_database mission_database.launch.py \
    robot_names:=warthog1 \
    home_lat:=39.35260 home_lon:=-76.34522 home_heading:=140.0

# Update home at runtime without restart
ros2 param set /mission_database_warthog1 home_lat 39.35260
ros2 param set /mission_database_warthog1 home_lon -76.34522

# Query breadcrumbs directly
sqlite3 /path/to/warthog1.db \
    "SELECT timestamp, lat, lon, has_comms, heading FROM breadcrumbs ORDER BY id DESC LIMIT 10;"
```

---

## Database Files

Each launch session creates a timestamped SQLite file per robot, grouped under a shared run folder:

```
{db_dir}/
└── 2026-04-07_16-31-00/
    ├── warthog1.db
    ├── warthog2.db
    └── warthog3.db
```

Robots launched within 3 minutes of each other share the same run folder. The grouping window is configurable in `mission_database_launch.py → LAUNCH_GROUP_WINDOW_S`.

---

## WaypointEvent Message

| Field | Type | Description |
|---|---|---|
| `header` | `std_msgs/Header` | Timestamp set by `PublishWaypointEvent` BT node at publish time |
| `event_type` | `uint8` | `DISPATCHED=0`, `REACHED=1`, `FAILED=2` |
| `waypoint_id` | `uint32` | Unique ID — must match between DISPATCHED and its REACHED/FAILED |
| `lat` | `float64` | Target latitude (DISPATCHED only) |
| `lon` | `float64` | Target longitude (DISPATCHED only) |
| `heading` | `float64` | Desired heading at waypoint (degrees 0–360). `-1.0` = unconstrained |
| `radius` | `float64` | Arrival acceptance radius in metres (default 2.0) |
| `name` | `string` | Optional human-readable label |

---

## Recovery Navigation Bundle

The `recovery_nav` topic gives BT recovery behaviors everything needed to navigate back to comms and then home:

```json
{
  "robot_name": "warthog1",
  "home": { "valid": true, "lat": 39.35260, "lon": -76.34522, "heading": 140.0, "set_by": "parameter" },
  "last_comms_position": { "valid": true, "lat": 39.35247, "lon": -76.34510, "timestamp": "..." },
  "return_waypoints": [
    { "waypoint_id": 3, "lat": 39.35241, "lon": -76.34505, "reached_at": "...", "had_comms": true }
  ]
}
```

Suggested recovery sequence: `last_comms_position` → step through `return_waypoints` (most-recent first) → `home`.

---

## TAK Replay

```bash
# Real-time replay to ATAK
ros2 run mission_database db_tak_replay.py /path/to/warthog1.db

# 4× speed
ros2 run mission_database db_tak_replay.py /path/to/warthog1.db --speed 4.0

# Instant (plot all at once)
ros2 run mission_database db_tak_replay.py /path/to/warthog1.db --instant
```

Breadcrumb markers are coloured by comms status: white = comms OK, red = comms lost, white = unknown. Terminal controls: `Space`/`p` = pause/resume, `q` = quit.

---

## Database Schema

```sql
CREATE TABLE breadcrumbs (
    id INTEGER PRIMARY KEY, lat REAL, lon REAL,
    timestamp TEXT, has_comms INTEGER,   -- NULL/0/1
    heading REAL, speed REAL, metadata TEXT DEFAULT '{}'
);

CREATE TABLE home_position (
    id INTEGER PRIMARY KEY CHECK (id=1),
    lat REAL, lon REAL, heading REAL, timestamp TEXT, set_by TEXT
);

CREATE TABLE waypoints (
    id INTEGER PRIMARY KEY, waypoint_id INTEGER,
    lat REAL, lon REAL, radius REAL DEFAULT 2.0, heading REAL, name TEXT,
    dispatched_at TEXT,
    has_comms_at_dispatch INTEGER, compass_heading_at_dispatch REAL,
    status TEXT DEFAULT 'pending',  -- 'pending' | 'reached' | 'failed'
    completed_at TEXT
);
```

---

## Package Structure

```
mission_database/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── mission_database.launch.py
├── msg/
│   └── WaypointEvent.msg
├── srv/
│   └── QueryBreadcrumbs.srv
├── include/mission_database/
│   └── mission_database_node.hpp
├── src/
│   └── mission_database_node.cpp
└── scripts/
    └── db_tak_replay.py
```
