"""
mission_database.launch.py

Launches one mission_database_node per robot.

PER-ROBOT HOME DEFAULTS:
    Edit the ROBOT_HOME_DEFAULTS dict near the top of this file to set
    default home positions for warthog1, warthog2, warthog3 (or any robot).
    These are used automatically when home_lat/home_lon are not passed as
    launch arguments.

USAGE (uses ROBOT_HOME_DEFAULTS automatically):
    ros2 launch mission_database mission_database.launch.py
    ros2 launch mission_database mission_database.launch.py robot_names:=warthog1,warthog2,warthog3

USAGE (override home for all robots on the command line):
    ros2 launch mission_database mission_database.launch.py \
        robot_names:=warthog1 \
        home_lat:=39.35264 \
        home_lon:=-76.34541 \
        home_heading:=180.0

LAUNCH ARGUMENTS:
    robot_names       Comma-separated robot names         (default: warthog1)
    home_lat          Override home latitude  (degrees)   (default: nan = use per-robot default)
    home_lon          Override home longitude (degrees)   (default: nan = use per-robot default)
    home_heading      Override home heading   (0-360 deg) (default: nan = use per-robot default)
    min_distance_m    Min metres between breadcrumbs      (default: 5.0)
    max_db_size_mb    DB size limit before eviction        (default: 10.0)
    publish_window_size  Breadcrumbs in trail topic        (default: 100)
    publish_rate_hz   Periodic re-publish rate            (default: 1.0)
    db_dir            Directory for .db files             (default: /phoenix/src/utils/mission_database/database)
                      Each session creates: {robot_name}_{YYYY-MM-DD}_{HH-MM-SS}.db
    debug             Verbose logging                     (default: false)

USAGE (included from another launch file):
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from ament_index_python.packages import get_package_share_directory

    IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('mission_database'),
            '/launch/mission_database.launch.py'
        ]),
        launch_arguments={
            'robot_names': 'warthog1,warthog2',
        }.items()
    )
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import math
import os
from datetime import datetime


# =============================================================================
# PER-ROBOT HOME POSITION DEFAULTS
#
# Edit these coordinates to match your actual base/start location.
# These are used when home_lat / home_lon are NOT passed as launch arguments.
#
# If you DO pass home_lat and home_lon on the command line, those values
# override all robots (useful for quickly relocating a whole squad).
#
# heading: degrees clockwise from north (0-360), or None if not needed.
# =============================================================================

ROBOT_HOME_DEFAULTS = {
    "warthog1": {"lat": 39.35260, "lon": -76.34522, "heading": 140.0},
    "warthog2": {"lat": 39.35259, "lon": -76.34527, "heading": 140.0},
    "warthog3": {"lat": 39.35256, "lon": -76.34530, "heading": 140.0},
}


def launch_setup(context, *args, **kwargs):
    """
    OpaqueFunction lets us evaluate LaunchConfiguration values at launch time
    so we can split the robot_names CSV string into individual Node instances.
    """

    # ── READ LAUNCH ARGUMENTS ─────────────────────────────────────────────────

    robot_names_str = LaunchConfiguration("robot_names").perform(context)

    def _to_float(s: str) -> float:
        """Parse a launch argument string to float, treating 'nan' as float('nan')."""
        return float("nan") if s.strip().lower() == "nan" else float(s)

    home_lat = _to_float(LaunchConfiguration("home_lat").perform(context))
    home_lon = _to_float(LaunchConfiguration("home_lon").perform(context))
    home_heading = _to_float(LaunchConfiguration("home_heading").perform(context))
    min_distance_m = float(LaunchConfiguration("min_distance_m").perform(context))
    max_db_size_mb = float(LaunchConfiguration("max_db_size_mb").perform(context))
    publish_window = int(LaunchConfiguration("publish_window_size").perform(context))
    publish_rate_hz = float(LaunchConfiguration("publish_rate_hz").perform(context))
    db_dir = LaunchConfiguration("db_dir").perform(context)
    debug = LaunchConfiguration("debug").perform(context).lower() == "true"

    # Timestamp generated once at launch time so all robots in the same launch
    # share the same timestamp string -- makes it easy to correlate their DBs.
    launch_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    # Create the database directory if it doesn't already exist.
    os.makedirs(db_dir, exist_ok=True)

    # ── PARSE ROBOT NAMES ─────────────────────────────────────────────────────

    robot_names = [name.strip() for name in robot_names_str.split(",") if name.strip()]
    if not robot_names:
        raise RuntimeError(
            "[mission_database.launch] robot_names is empty. "
            "Pass at least one name, e.g. robot_names:=warthog1"
        )

    # ── ONE NODE PER ROBOT ────────────────────────────────────────────────────

    nodes = []
    for robot_name in robot_names:

        # ── RESOLVE HOME POSITION ─────────────────────────────────────────────
        # Priority: command-line args > per-robot defaults > unset (nan).
        #
        # If home_lat and home_lon were both given on the command line (finite
        # values), use them for every robot.  Otherwise fall back to that
        # robot's entry in ROBOT_HOME_DEFAULTS, if one exists.

        global_home_set = math.isfinite(home_lat) and math.isfinite(home_lon)

        if global_home_set:
            # Command-line override applies to all robots.
            robot_home_lat = home_lat
            robot_home_lon = home_lon
            robot_home_heading = home_heading
        elif robot_name in ROBOT_HOME_DEFAULTS:
            # Use the per-robot default from the dict above.
            d = ROBOT_HOME_DEFAULTS[robot_name]
            robot_home_lat = d.get("lat", float("nan"))
            robot_home_lon = d.get("lon", float("nan"))
            robot_home_heading = d.get("heading", float("nan"))
            if robot_home_heading is None:
                robot_home_heading = float("nan")
        else:
            # No default for this robot -- home will be unset until provided
            # at runtime via ros2 param set or a future launch.
            robot_home_lat = float("nan")
            robot_home_lon = float("nan")
            robot_home_heading = float("nan")

        # DB filename includes robot name + launch timestamp so each session
        # gets its own file.  Example: warthog1_2024-03-15_14-23-07.db
        db_path = f"{db_dir}/{robot_name}_{launch_timestamp}.db"

        nodes.append(
            Node(
                package="mission_database",
                executable="mission_database_node",
                # Node name is unique per robot so each instance has its own
                # parameter server, parameter namespace, and log identity.
                name=f"mission_database_{robot_name}",
                # No namespace here intentionally -- the node builds all its
                # topic paths internally as /{robot_name}/mission_database/...
                # so namespacing the node would double-prefix everything.
                parameters=[
                    {
                        "robot_name": robot_name,
                        "db_path": db_path,
                        "home_lat": robot_home_lat,
                        "home_lon": robot_home_lon,
                        "home_heading": robot_home_heading,
                        "min_distance_m": min_distance_m,
                        "max_db_size_mb": max_db_size_mb,
                        "publish_window_size": publish_window,
                        "publish_rate_hz": publish_rate_hz,
                        "debug": debug,
                    }
                ],
                # Prefix log output with the robot name so multi-robot runs
                # are easy to read in a combined log stream.
                output="screen",
                emulate_tty=True,
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            # ── LAUNCH ARGUMENTS ──────────────────────────────────────────────────
            DeclareLaunchArgument(
                "robot_names",
                default_value="warthog1",
                description="Comma-separated list of robot names, e.g. warthog1,warthog2,warthog3",
            ),
            DeclareLaunchArgument(
                "home_lat",
                default_value="nan",
                description=(
                    "Home position latitude in decimal degrees. "
                    "Leave as nan to use any previously stored home, or if home is not needed."
                ),
            ),
            DeclareLaunchArgument(
                "home_lon",
                default_value="nan",
                description="Home position longitude in decimal degrees. Leave as nan if not needed.",
            ),
            DeclareLaunchArgument(
                "home_heading",
                default_value="nan",
                description=(
                    "Home position heading in degrees (0-360 clockwise from north). "
                    "Leave as nan if heading is not relevant for your home location."
                ),
            ),
            DeclareLaunchArgument(
                "min_distance_m",
                default_value="8.0",
                description="Minimum distance in metres between recorded breadcrumbs.",
            ),
            DeclareLaunchArgument(
                "max_db_size_mb",
                default_value="10.0",
                description="Maximum SQLite database size in MB before oldest breadcrumbs are evicted.",
            ),
            DeclareLaunchArgument(
                "publish_window_size",
                default_value="100",
                description="Number of most-recent breadcrumbs included in the breadcrumb_trail topic.",
            ),
            DeclareLaunchArgument(
                "publish_rate_hz",
                default_value="1.0",
                description="Rate at which topics are re-published even without new data.",
            ),
            DeclareLaunchArgument(
                "db_dir",
                default_value="/phoenix/src/utils/mission_database/database",
                description=(
                    "Directory where SQLite database files are saved. "
                    "Created automatically if it does not exist. "
                    "Each launch session produces a new timestamped file per robot: "
                    "{robot_name}_{YYYY-MM-DD}_{HH-MM-SS}.db"
                ),
            ),
            DeclareLaunchArgument(
                "debug",
                default_value="false",
                description="Enable verbose [DEBUG] logging in the node.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
