# WaypointEvent.msg
#
# Published by BehaviorTree nodes to record waypoint dispatch and completion.
# Topic: /{robot_name}/mission_database/waypoint_event
#
# WORKFLOW:
#   1. BT node sends waypoint to navigation:
#        publish WaypointEvent with event_type=DISPATCHED
#        The database snapshots current comms status at this moment.
#
#   2a. Navigation succeeds:
#        publish WaypointEvent with event_type=REACHED, same waypoint_id
#
#   2b. Navigation fails or is preempted:
#        publish WaypointEvent with event_type=FAILED, same waypoint_id
#
# EXAMPLE (BT node pseudocode):
#   auto evt       = WaypointEvent{};
#   evt.event_type = WaypointEvent::DISPATCHED;
#   evt.waypoint_id = objective.id;          // use your mission waypoint id
#   evt.lat        = target_lat;
#   evt.lon        = target_lon;
#   evt.heading    = target_heading_deg;     // -1.0 if no heading constraint
#   evt.name       = objective.name;
#   waypoint_event_pub_->publish(evt);
#
# NOTE ON waypoint_id:
#   This field is how REACHED/FAILED events are matched back to their DISPATCHED
#   event in the database.  Use your mission objective's id, or a monotonically
#   increasing counter — whatever is unique within a mission session.
#   If the same waypoint_id is dispatched twice (e.g. retry after failure), the
#   most recent pending row is updated by the subsequent REACHED/FAILED event.

# ── EVENT TYPE CONSTANTS ──────────────────────────────────────────────────────
uint8 DISPATCHED = 0   # Waypoint sent to navigation stack
uint8 REACHED    = 1   # Robot arrived at waypoint successfully
uint8 FAILED     = 2   # Navigation failed or was aborted/preempted

# ── FIELDS ────────────────────────────────────────────────────────────────────

std_msgs/Header header

# Which event this message represents (DISPATCHED, REACHED, or FAILED).
uint8 event_type

# GPS coordinates of the waypoint target.
# Required for DISPATCHED.  Ignored for REACHED/FAILED (looked up by waypt_num).
float64 lat
float64 lon

# Desired heading at the waypoint in degrees clockwise from north (0–360).
# Set to -1.0 if heading is not constrained for this waypoint.
float64 heading

# Acceptance radius around the waypoint in metres.
# The robot is considered to have reached the waypoint when it comes within
# this distance.  Default 2.0 m if not specified.
float64 radius

# Optional human-readable label (e.g. "alpha", "checkpoint_3", "extract_point").
# Used in the database and published JSON for human readability.
string name

# Unique identifier for this waypoint within the current mission.
# Must match between the DISPATCHED event and the subsequent REACHED/FAILED event.
uint32 waypoint_id
