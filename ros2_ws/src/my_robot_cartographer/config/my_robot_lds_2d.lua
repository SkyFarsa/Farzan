include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",

  -- Keep tracking on base_link for now, NOT imu_link. Get this TF-ownership
  -- fix verified working first, then re-enable IMU fusion as its own
  -- separate, isolated test (see notes at the bottom of this file).
  tracking_frame = "base_link",

  -- FIXED: this was "base_link" together with provide_odom_frame=true below,
  -- which made cartographer broadcast its OWN odom -> base_link transform
  -- every cycle - directly colliding with serial_odom_node.py, which also
  -- broadcasts odom -> base_link from wheel encoders. Two nodes publishing
  -- the same TF edge from two different pose estimates is what causes
  -- exactly the jumping/shaking you're seeing right now.
  published_frame = "odom",

  odom_frame = "odom",

  -- FIXED: was `true`. With published_frame = "odom", this must be false -
  -- cartographer now only owns map -> odom. serial_odom_node keeps owning
  -- odom -> base_link, so there's exactly one publisher per TF edge.
  provide_odom_frame = false,

  publish_frame_projected_to_2d = true,

  -- CONFIRMED WORKING: /odom verified at ~19.6Hz with sane data via
  -- `ros2 topic hz/echo/info -v` earlier in this debugging session.
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0

-- IMPORTANT: leave this false for THIS test, even though you just
-- reconnected the IMU. Re-enabling IMU fusion needs tracking_frame changed
-- to "imu_link" too, and that's a second variable - change one thing at a
-- time. Confirm the jump/shake is gone with just the TF-ownership fix
-- first, THEN come back and flip this + tracking_frame together as an
-- isolated follow-up test.
TRAJECTORY_BUILDER_2D.use_imu_data = false

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options
