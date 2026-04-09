"""
mission_database.launch.py
==========================
Launches one mission_database_node per robot.

PETAAR26 INTEGRATION:
    All parameter defaults are sourced from the petaar26 package config files:
      config/paths.yaml       → db_dir default (mission_db_dir)
      config/robot_homes.yaml → per-robot home positions (replaces hardcoded dict)

    To relocate robots or change the database directory for a new deployment,
    edit those config files only — no Python changes needed.

RUN FOLDER GROUPING:
    Each launch session creates a timestamped subdirectory inside db_dir and
    stores all robots' .db files there:

        database/
          2026-04-07_16-31-00/
            warthog1.db
            warthog2.db
            warthog3.db

    Because each robot's stack is launched roughly 30 seconds apart (staggered
    from stack_launch.xml), each robot's launch file invocation runs at a
    different wall-clock time. To group them into the same run folder this
    file scans db_dir for an existing run folder created within the last
    LAUNCH_GROUP_WINDOW_S seconds. If one is found the new robot joins it;
    otherwise a fresh folder is created. The window is set conservatively
    (default: 3 minutes) to safely cover the full 3-robot stagger.

PER-ROBOT HOME DEFAULTS:
    Home positions are now loaded from config/robot_homes.yaml in the petaar26
    package rather than from a hardcoded dict in this file. Edit that YAML to
    update staging positions for new experiments or deployment sites.

USAGE (uses robot_homes.yaml home positions automatically):
    ros2 launch mission_database mission_database.launch.py
    ros2 launch mission_database mission_database.launch.py robot_names:=warthog1,warthog2,warthog3

USAGE (override home for all robots on the command line):
    ros2 launch mission_database mission_database.launch.py \\
        robot_names:=warthog1 \\
        home_lat:=39.35264 \\
        home_lon:=-76.34541 \\
        home_heading:=180.0

LAUNCH ARGUMENTS:
    robot_names          Comma-separated robot names            (default: warthog1)
    home_lat             Override home latitude  (degrees)      (default: nan = use per-robot default)
    home_lon             Override home longitude (degrees)      (default: nan = use per-robot default)
    home_heading         Override home heading   (0-360 deg)    (default: nan = use per-robot default)
    min_distance_m       Min metres between breadcrumbs         (default: 8.0)
    max_db_size_mb       DB size limit before eviction          (default: 10.0)
    publish_window_size  Breadcrumbs in trail topic             (default: 100)
    publish_rate_hz      Periodic re-publish rate               (default: 1.0)
    db_dir               Root directory for run folders         (default: from config/paths.yaml)
    debug                Verbose logging                        (default: false)

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

import math
import os
import re
import yaml
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# =============================================================================
# PETAAR26 CONFIG LOADER
# =============================================================================

def _petaar26(filename: str) -> dict:
    """Load a petaar26 config YAML from the installed share directory."""
    path = os.path.join(
        get_package_share_directory('petaar26'),
        'config', filename
    )
    with open(path) as f:
        return yaml.safe_load(f)


_paths  = _petaar26('paths.yaml')['paths']
_hw     = _petaar26('hardware.yaml')['hardware']
_topics = _petaar26('topics.yaml')['topics']

# Per-robot home positions loaded from robot_homes.yaml.
# Previously this was a hardcoded ROBOT_HOME_DEFAULTS dict in this file.
# Now it lives in config/robot_homes.yaml so a new team edits only that file.
_homes_raw = _petaar26('robot_homes.yaml')['robot_homes']
ROBOT_HOME_DEFAULTS = {
    name: {
        'lat':     vals['lat'],
        'lon':     vals['lon'],
        'heading': vals.get('heading', float('nan')),
    }
    for name, vals in _homes_raw.items()
}


# =============================================================================
# RUN FOLDER GROUPING WINDOW
#
# When a robot's launch file starts it scans db_dir for an existing run folder
# created within this many seconds. If one is found the robot joins it rather
# than creating a new folder. Set this to comfortably exceed the total stagger
# across all robots:
#
#   stagger_per_robot (~30 s) × max_robots (3) = ~90 s worst case.
#   180 s gives a 2× safety margin.
#
# Only increase this if you have more robots or a longer stagger. Making it
# too large risks accidentally grouping two separate runs (e.g. a quick
# restart mid-day) into the same folder.
# =============================================================================

LAUNCH_GROUP_WINDOW_S = 180


# =============================================================================
# RUN FOLDER TIMESTAMP FORMAT
#
# Must sort lexicographically in chronological order (YYYY-MM-DD_HH-MM-SS
# satisfies this). Used both when creating and when scanning for folders.
# =============================================================================

_FOLDER_FORMAT = "%Y-%m-%d_%H-%M-%S"
_FOLDER_RE     = re.compile(r"^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}$")


# =============================================================================
# HELPERS
# =============================================================================

def _find_or_create_run_folder(db_dir: str, window_s: int) -> str:
    """
    Return the name (not full path) of the run folder this robot should use.

    Algorithm:
      1. List all subdirectories of db_dir whose names match the timestamp
         format (YYYY-MM-DD_HH-MM-SS).
      2. Among those created within window_s seconds of now, pick the OLDEST
         one — that is the folder created by the first robot in this run.
      3. If no such folder exists, generate a new timestamp and return it.
         The caller is responsible for creating the directory.

    WHY oldest-within-window:
      warthog1 creates the folder at T+0.
      warthog2 arrives at T+30: sees a folder that is 30 s old (< 180 s) → joins.
      warthog3 arrives at T+60: sees the same folder, now 60 s old (< 180 s) → joins.
      All three end up in the folder named after warthog1's start time.

    Thread/process safety:
      os.makedirs(..., exist_ok=True) on the chosen folder is idempotent, so
      two robots arriving at almost the same instant and both choosing to create
      a new folder will not race destructively. The worst outcome is two
      identically-named folders, which is prevented because strftime has
      1-second resolution — in practice only one process can win the race to
      create a given second's folder.
    """
    now        = datetime.now()
    candidates = []  # list of (age_seconds, folder_name)

    try:
        entries = os.listdir(db_dir)
    except FileNotFoundError:
        entries = []

    for entry in entries:
        if not _FOLDER_RE.match(entry):
            continue
        full_path = os.path.join(db_dir, entry)
        if not os.path.isdir(full_path):
            continue
        try:
            folder_time = datetime.strptime(entry, _FOLDER_FORMAT)
        except ValueError:
            continue
        age = (now - folder_time).total_seconds()
        # Only consider folders in [0, window_s]. Negative age (folder in the
        # future) would indicate clock skew or manual mis-naming — skip it.
        if 0.0 <= age <= window_s:
            candidates.append((age, entry))

    if candidates:
        # Sort descending by age: largest age = oldest folder = the one created
        # by the first robot in this run.
        candidates.sort(reverse=True)
        chosen = candidates[0][1]
        print(
            f"[mission_database.launch] Joining existing run folder '{chosen}' "
            f"(created {candidates[0][0]:.0f}s ago, window={window_s}s)"
        )
        return chosen

    # No suitable folder found — start a new run.
    new_folder = now.strftime(_FOLDER_FORMAT)
    print(
        f"[mission_database.launch] Starting new run folder '{new_folder}' "
        f"(no existing folder within {window_s}s window)"
    )
    return new_folder


# =============================================================================
# LAUNCH SETUP
# =============================================================================

def launch_setup(context, *args, **kwargs):
    """
    OpaqueFunction lets us evaluate LaunchConfiguration values at launch time
    so we can split the robot_names CSV string into individual Node instances
    and resolve per-robot home positions from robot_homes.yaml.
    """

    # ── READ LAUNCH ARGUMENTS ─────────────────────────────────────────────────

    def _to_float(s: str) -> float:
        """Parse a launch argument string to float, treating 'nan' as float('nan')."""
        return float('nan') if s.strip().lower() == 'nan' else float(s)

    robot_names_str = LaunchConfiguration('robot_names').perform(context)
    gps_topic_suffix = LaunchConfiguration('gps_topic_suffix').perform(context)
    home_lat        = _to_float(LaunchConfiguration('home_lat').perform(context))
    home_lon        = _to_float(LaunchConfiguration('home_lon').perform(context))
    home_heading    = _to_float(LaunchConfiguration('home_heading').perform(context))
    min_distance_m  = float(LaunchConfiguration('min_distance_m').perform(context))
    max_db_size_mb  = float(LaunchConfiguration('max_db_size_mb').perform(context))
    publish_window  = int(LaunchConfiguration('publish_window_size').perform(context))
    publish_rate_hz = float(LaunchConfiguration('publish_rate_hz').perform(context))
    db_dir          = LaunchConfiguration('db_dir').perform(context)
    debug           = LaunchConfiguration('debug').perform(context).lower() == 'true'

    # ── RESOLVE RUN FOLDER ────────────────────────────────────────────────────
    # Ensure db_dir exists before scanning it, then find or create the run
    # folder. All robots in the same run (within LAUNCH_GROUP_WINDOW_S seconds
    # of the first robot's launch) share the same folder.

    os.makedirs(db_dir, exist_ok=True)

    run_folder_name = _find_or_create_run_folder(db_dir, LAUNCH_GROUP_WINDOW_S)
    run_folder_path = os.path.join(db_dir, run_folder_name)
    os.makedirs(run_folder_path, exist_ok=True)

    # ── PARSE ROBOT NAMES ─────────────────────────────────────────────────────

    robot_names = [name.strip() for name in robot_names_str.split(',') if name.strip()]
    if not robot_names:
        raise RuntimeError(
            "[mission_database.launch] robot_names is empty. "
            "Pass at least one name, e.g. robot_names:=warthog1"
        )

    # ── ONE NODE PER ROBOT ────────────────────────────────────────────────────

    nodes = []
    for robot_name in robot_names:

        # ── RESOLVE HOME POSITION ─────────────────────────────────────────────
        # Priority:
        #   1. Command-line home_lat/home_lon (overrides ALL robots).
        #   2. Per-robot entry in config/robot_homes.yaml (ROBOT_HOME_DEFAULTS).
        #   3. float('nan') — home unset; node will accept runtime updates.

        global_home_set = math.isfinite(home_lat) and math.isfinite(home_lon)

        if global_home_set:
            robot_home_lat     = home_lat
            robot_home_lon     = home_lon
            robot_home_heading = home_heading
        elif robot_name in ROBOT_HOME_DEFAULTS:
            d = ROBOT_HOME_DEFAULTS[robot_name]
            robot_home_lat     = d.get('lat',     float('nan'))
            robot_home_lon     = d.get('lon',     float('nan'))
            robot_home_heading = d.get('heading', float('nan'))
            if robot_home_heading is None:
                robot_home_heading = float('nan')
        else:
            robot_home_lat     = float('nan')
            robot_home_lon     = float('nan')
            robot_home_heading = float('nan')
            print(
                f"[mission_database.launch] WARNING: No home position found for "
                f"'{robot_name}' in config/robot_homes.yaml. Home will be unset "
                f"until provided at runtime via ros2 param set."
            )

        # DB path: {db_dir}/{run_folder}/{robot_name}.db
        # Example: database/2026-04-07_16-31-00/warthog1.db
        db_path = os.path.join(run_folder_path, f"{robot_name}.db")

        nodes.append(
            Node(
                package='mission_database',
                executable='mission_database_node',
                # Node name is unique per robot so each instance has its own
                # parameter server, parameter namespace, and log identity.
                name=f'mission_database_{robot_name}',
                # No namespace here intentionally — the node builds all its
                # topic paths internally as /{robot_name}/mission_database/...
                # so namespacing the node would double-prefix everything.
                parameters=[{
                    'robot_name':          robot_name,
                    'gps_topic_suffix':    gps_topic_suffix,
                    'db_path':             db_path,
                    'home_lat':            robot_home_lat,
                    'home_lon':            robot_home_lon,
                    'home_heading':        robot_home_heading,
                    'min_distance_m':      min_distance_m,
                    'max_db_size_mb':      max_db_size_mb,
                    'publish_window_size': publish_window,
                    'publish_rate_hz':     publish_rate_hz,
                    'debug':               debug,
                }],
                output='screen',
                emulate_tty=True,
            )
        )

    return nodes


# =============================================================================
# LAUNCH DESCRIPTION
# =============================================================================

def generate_launch_description():
    return LaunchDescription([

        # ── LAUNCH ARGUMENTS ──────────────────────────────────────────────────

        DeclareLaunchArgument(
            'robot_names',
            default_value='warthog1',
            description='Comma-separated list of robot names, e.g. warthog1,warthog2,warthog3',
        ),
        DeclareLaunchArgument(
            'gps_topic_suffix',
            # Default to geofog for backward compatibility with existing configs
            # that do not pass this argument. sim_control.py passes the active
            # profile's gps_topic_suffix so the node subscribes to the correct
            # hardware topic (geofog for NAI_2/testing, ublox for NAI_3/NAI_4).
            default_value=_hw['gps_topic_suffix'],
            description=(
                "GPS topic suffix within each robot namespace. "
                "Node subscribes to /{robot_name}/{gps_topic_suffix}. "
                "Options: sensors/geofog/gps/fix (GeoFog, NAI_2) "
                "or sensors/ublox/fix (u-blox, NAI_3/NAI_4). "
                "Driven by gps_topic_suffix in the active profile (profiles.json)."
            ),
        ),
        DeclareLaunchArgument(
            'home_lat',
            default_value='nan',
            description=(
                "Home position latitude in decimal degrees. "
                "Overrides all robots. Leave as nan to use per-robot defaults "
                "from config/robot_homes.yaml."
            ),
        ),
        DeclareLaunchArgument(
            'home_lon',
            default_value='nan',
            description=(
                "Home position longitude in decimal degrees. "
                "Overrides all robots. Leave as nan to use per-robot defaults "
                "from config/robot_homes.yaml."
            ),
        ),
        DeclareLaunchArgument(
            'home_heading',
            default_value='nan',
            description=(
                "Home position heading in degrees (0-360 clockwise from north). "
                "Overrides all robots. Leave as nan to use per-robot defaults "
                "from config/robot_homes.yaml."
            ),
        ),
        DeclareLaunchArgument(
            'min_distance_m',
            default_value='8.0',
            description='Minimum distance in metres between recorded breadcrumbs.',
        ),
        DeclareLaunchArgument(
            'max_db_size_mb',
            default_value='10.0',
            description='Maximum SQLite database size in MB before oldest breadcrumbs are evicted.',
        ),
        DeclareLaunchArgument(
            'publish_window_size',
            default_value='100',
            description='Number of most-recent breadcrumbs included in the breadcrumb_trail topic.',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='1.0',
            description='Rate at which topics are re-published even without new data.',
        ),
        DeclareLaunchArgument(
            'db_dir',
            # Default sourced from config/paths.yaml — previously hardcoded.
            default_value=_paths['mission_db_dir'],
            description=(
                "Root directory for run folders. "
                "Created automatically if it does not exist. "
                "Each run produces a timestamped subdirectory containing one .db per robot: "
                "{db_dir}/{YYYY-MM-DD_HH-MM-SS}/{robot_name}.db. "
                "Robots launched within LAUNCH_GROUP_WINDOW_S seconds of each other "
                "share the same subdirectory. "
                "Default from config/paths.yaml → paths.mission_db_dir."
            ),
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable verbose [DEBUG] logging in the node.',
        ),

        OpaqueFunction(function=launch_setup),
    ])
