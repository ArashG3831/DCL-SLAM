"""Shared deterministic course definition for motion characterization.

The course is diagnostic-only.  Completion is gated by an external Supervisor
observer, while ROS publishes only the requested velocity for the current
segment.  Supervisor pose data never enters a ROS state-estimation path.
"""

COURSE = (
    # A four-sided left-turn square returns to the starting region.
    {"kind": "straight", "distance_m": 0.60, "label": "east"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": 1, "label": "left_1"},
    {"kind": "straight", "distance_m": 0.60, "label": "north"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": 1, "label": "left_2"},
    {"kind": "straight", "distance_m": 0.60, "label": "west"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": 1, "label": "left_3"},
    {"kind": "straight", "distance_m": 0.60, "label": "south"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": 1, "label": "left_4"},
    # A smaller return loop supplies right-turn evidence as well.
    {"kind": "straight", "distance_m": 0.30, "label": "east_return"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": -1, "label": "right_1"},
    {"kind": "straight", "distance_m": 0.30, "label": "south_return"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": -1, "label": "right_2"},
    {"kind": "straight", "distance_m": 0.30, "label": "west_return"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": -1, "label": "right_3"},
    {"kind": "straight", "distance_m": 0.30, "label": "north_return"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": -1, "label": "right_4"},
)

# Long-world diagnostic course.  The robot starts at (16, 0), facing +Y in
# epuck_d500_two_world_large.wbt.  It repeatedly traverses the open right-hand
# corridor between y=0 and y=4 m, with a small lateral offset at each end.
# Five traversals are approximately 43 m total and deliberately revisit the
# same walls while exercising both left and right 90-degree turns.  This is
# diagnostic-only; it is not a production navigation route.
LONG_CORRIDOR_COURSE = (
    {"kind": "straight", "distance_m": 4.0, "label": "north_1"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": 1, "label": "left_top_1"},
    {"kind": "straight", "distance_m": 0.30, "label": "offset_west_1"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": 1, "label": "left_down_1"},
    {"kind": "straight", "distance_m": 4.0, "label": "south_1"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": 1, "label": "left_bottom_1"},
    {"kind": "straight", "distance_m": 0.30, "label": "offset_east_1"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": 1, "label": "left_up_1"},
    {"kind": "straight", "distance_m": 4.0, "label": "north_2"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": -1, "label": "right_top_2"},
    {"kind": "straight", "distance_m": 0.30, "label": "offset_east_2"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": -1, "label": "right_down_2"},
    {"kind": "straight", "distance_m": 4.0, "label": "south_2"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": -1, "label": "right_bottom_2"},
    {"kind": "straight", "distance_m": 0.30, "label": "offset_west_2"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": -1, "label": "right_up_2"},
    {"kind": "straight", "distance_m": 4.0, "label": "north_3"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": 1, "label": "left_top_3"},
    {"kind": "straight", "distance_m": 0.30, "label": "offset_west_3"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": 1, "label": "left_down_3"},
    {"kind": "straight", "distance_m": 4.0, "label": "south_3"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": 1, "label": "left_bottom_3"},
    {"kind": "straight", "distance_m": 0.30, "label": "offset_east_3"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": 1, "label": "left_up_3"},
    {"kind": "straight", "distance_m": 4.0, "label": "north_4"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": -1, "label": "right_top_4"},
    {"kind": "straight", "distance_m": 0.30, "label": "offset_east_4"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": -1, "label": "right_down_4"},
    {"kind": "straight", "distance_m": 4.0, "label": "south_4"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": -1, "label": "right_bottom_4"},
    {"kind": "straight", "distance_m": 0.30, "label": "offset_west_4"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": -1, "label": "right_up_4"},
    {"kind": "straight", "distance_m": 4.0, "label": "north_5"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": 1, "label": "left_top_5"},
    {"kind": "straight", "distance_m": 0.30, "label": "offset_west_5"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": 1, "label": "left_down_5"},
    {"kind": "straight", "distance_m": 4.0, "label": "south_5"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": 1, "label": "left_bottom_5"},
    {"kind": "straight", "distance_m": 0.30, "label": "offset_east_5"},
    {"kind": "turn", "angle_rad": 1.5707963267948966, "direction": 1, "label": "left_finish"},
)


def course_for_name(name):
    return LONG_CORRIDOR_COURSE if str(name) == 'long_corridor' else COURSE


def segment_goal(index):
    return COURSE[index] if 0 <= index < len(COURSE) else None
