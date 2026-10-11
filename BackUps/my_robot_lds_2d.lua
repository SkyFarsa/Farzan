include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",

  -- Track the robot base directly (no IMU for now — re-enable as a separate
  -- test once the TF conflict below is confirmed fixed).
  tracking_frame = "base_link",

  -- FIXED: this used to be "base_link" together with provide_odom_frame=true
  -- below, which made cartographer broadcast its OWN odom -> base_link
  -- transform every cycle — directly colliding with serial_odom_node.py,
  -- which already broadcasts odom -> base_link from wheel encoders. Two
  -- nodes publishing the same TF edge from two different pose estimates is
  -- what was almost certainly causing the jumping/shaking and, when the
  -- fight got bad enough to break cartographer's extrapolator, the
  -- "frame map does not exist" errors.
  --
  -- Now cartographer only owns map -> odom, and serial_odom_node keeps
  -- owning odom -> base_link, exactly like a normal map->odom->base_link
  -- tree should look.
  published_frame = "odom",

  -- Cartographer will create the odom frame
  odom_frame = "odom",

  -- FIXED: was `true`. With published_frame = "odom", provide_odom_frame
  -- must be false — cartographer now publishes map -> odom only, instead of
  -- also inserting itself as the odom -> base_link broadcaster.
  provide_odom_frame = false,

  publish_frame_projected_to_2d = true,

  -- FIXED: was `false`, with a stale comment claiming "your robot isn't
  -- publishing wheel odom yet" — it has been for a while now
  -- (serial_odom_node.py). Feeding real wheel odometry into cartographer
  -- gives it a much better motion prior between scan matches, instead of
  -- ignoring data you're already generating.
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

-- Leave this false until you re-enable the IMU as its own separate test.
-- Flip to true only once it's reconnected AND tracking_frame is changed
-- back to "imu_link" (use_imu_data requires the tracking frame to actually
-- have IMU data available).
TRAJECTORY_BUILDER_2D.use_imu_data = false

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options
