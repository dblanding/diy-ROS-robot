While running navigation with each of these luanch files:
* navigation.launch
* robot_nav.launch

I got the following warnings (but with tolerance: 0.2000):

[ WARN] [1638276005.609753544]: Costmap2DROS transform timeout. Current time: 1638276005.6096, global_pose stamp: 1638276005.1994, tolerance: 0.4000
[ WARN] [1638276005.679133297]: Could not get robot pose, cancelling reconfiguration

At first, I got these warnings when the value of transform_tolerance set at 0.2  in these two files:
* local_costmap_params.yaml
* global_costmap_params.yaml

So I bumped the value of transform_tolerance to 0.3 and even to 0.4.
But I still got warnings, though they were less frequent, I think.

But the strange thing is that even after I put the value back to 0.2, the tolerance value on the warnings was still 0.4000. Must be these values get cached in a parameter server? 

Here is a list of all the topics:

```
doug@raspi4:~$ rostopic list
/amcl/parameter_descriptions
/amcl/parameter_updates
/amcl_pose
/clicked_point
/cmd_vel
/diagnostics
/imu/data
/initialpose
/left_ticks
/map
/map_metadata
/map_updates
/move_base/DWAPlannerROS/cost_cloud
/move_base/DWAPlannerROS/global_plan
/move_base/DWAPlannerROS/local_plan
/move_base/DWAPlannerROS/parameter_descriptions
/move_base/DWAPlannerROS/parameter_updates
/move_base/DWAPlannerROS/trajectory_cloud
/move_base/NavfnROS/plan
/move_base/cancel
/move_base/current_goal
/move_base/feedback
/move_base/global_costmap/costmap
/move_base/global_costmap/costmap_updates
/move_base/global_costmap/footprint
/move_base/global_costmap/inflation_layer/parameter_descriptions
/move_base/global_costmap/inflation_layer/parameter_updates
/move_base/global_costmap/obstacle_layer/parameter_descriptions
/move_base/global_costmap/obstacle_layer/parameter_updates
/move_base/global_costmap/parameter_descriptions
/move_base/global_costmap/parameter_updates
/move_base/global_costmap/static_layer/parameter_descriptions
/move_base/global_costmap/static_layer/parameter_updates
/move_base/goal
/move_base/local_costmap/costmap
/move_base/local_costmap/costmap_updates
/move_base/local_costmap/footprint
/move_base/local_costmap/inflation_layer/parameter_descriptions
/move_base/local_costmap/inflation_layer/parameter_updates
/move_base/local_costmap/obstacle_layer/parameter_descriptions
/move_base/local_costmap/obstacle_layer/parameter_updates
/move_base/local_costmap/parameter_descriptions
/move_base/local_costmap/parameter_updates
/move_base/local_costmap/static_layer/parameter_descriptions
/move_base/local_costmap/static_layer/parameter_updates
/move_base/parameter_descriptions
/move_base/parameter_updates
/move_base/result
/move_base/status
/move_base_simple/goal
/odom
/particlecloud
/path
/right_ticks
/robot_pose_ekf/odom_combined
/rosout
/rosout_agg
/scan
/tf
/tf_static
```

and here are some of the topic rates:

```
doug@raspi4:~$ rostopic hz /odom
subscribed to [/odom]
average rate: 9.978

doug@raspi4:~$ rostopic hz /scan
subscribed to [/scan]
average rate: 6.953

doug@raspi4:~$ rostopic hz /imu/data
subscribed to [/imu/data]
average rate: 50.071

doug@raspi4:~$ rostopic hz /robot_pose_ekf/odom_combined
subscribed to [/robot_pose_ekf/odom_combined]
average rate: 17.407

doug@raspi4:~$ rostopic hz /right_ticks
subscribed to [/right_ticks]
average rate: 9.969
```

## Navigation tuning

The [Basic Navigation Tuning Guide](http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide) on the ROS wiki page lays out a sequence of steps in Navigation Tuning:
1. Make sure the robot is navigation ready. I believe my robot is ready.
2. Configure the Costmap
3. Tune the parameters of the Local Planner

12/2/21 Getting really frustrated trying to get nav to work. I haven't been able to get anywhere near the inital results. So I put all the initial parameters back to their initial values and got it to work again.

Here's the output of the run.

```
doug@raspi4:~$ roslaunch robot_nav robot_nav.launch
... logging to /home/doug/.ros/log/a79ed23c-535a-11ec-950d-1f9ea9f989bf/roslaunch-raspi4-48421.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://raspi4:43967/

SUMMARY
========

PARAMETERS
 * /amcl/gui_publish_rate: 10.0
 * /amcl/kld_err: 0.05
 * /amcl/kld_z: 0.99
 * /amcl/laser_lambda_short: 0.1
 * /amcl/laser_likelihood_max_dist: 2.0
 * /amcl/laser_max_beams: 30
 * /amcl/laser_model_type: likelihood_field
 * /amcl/laser_sigma_hit: 0.2
 * /amcl/laser_z_hit: 0.5
 * /amcl/laser_z_max: 0.05
 * /amcl/laser_z_rand: 0.5
 * /amcl/laser_z_short: 0.05
 * /amcl/max_particles: 5000
 * /amcl/min_particles: 500
 * /amcl/odom_alpha1: 0.2
 * /amcl/odom_alpha2: 0.2
 * /amcl/odom_alpha3: 0.8
 * /amcl/odom_alpha4: 0.2
 * /amcl/odom_alpha5: 0.1
 * /amcl/odom_frame_id: odom
 * /amcl/odom_model_type: diff
 * /amcl/recovery_alpha_fast: 0.0
 * /amcl/recovery_alpha_slow: 0.0
 * /amcl/resample_interval: 1
 * /amcl/transform_tolerance: 0.1
 * /amcl/update_min_a: 0.5
 * /amcl/update_min_d: 0.2
 * /move_base/TrajectoryPlannerROS/holonomic_robot: False
 * /move_base/TrajectoryPlannerROS/max_vel_theta: 3.0
 * /move_base/TrajectoryPlannerROS/max_vel_x: 0.3
 * /move_base/TrajectoryPlannerROS/meter_scoring: True
 * /move_base/TrajectoryPlannerROS/min_in_place_vel_theta: 2.5
 * /move_base/TrajectoryPlannerROS/min_vel_theta: -3.0
 * /move_base/TrajectoryPlannerROS/min_vel_x: 0.25
 * /move_base/TrajectoryPlannerROS/xy_goal_tolerance: 0.2
 * /move_base/TrajectoryPlannerROS/yaw_goal_tolerance: 0.5
 * /move_base/global_costmap/footprint: [[-0.175, -0.095]...
 * /move_base/global_costmap/global_costmap/global_frame: odom
 * /move_base/global_costmap/global_costmap/publish_frequency: 30.0
 * /move_base/global_costmap/global_costmap/resolution: 0.1
 * /move_base/global_costmap/global_costmap/transform_tolerance: 0.2
 * /move_base/global_costmap/global_costmap/update_frequency: 30.0
 * /move_base/global_costmap/global_frame: odom
 * /move_base/global_costmap/inflation_layer/inflation_radius: 0.2
 * /move_base/global_costmap/map_topic: /map
 * /move_base/global_costmap/obstacle_layer/laser_scan_sensor/clearing: True
 * /move_base/global_costmap/obstacle_layer/laser_scan_sensor/data_type: LaserScan
 * /move_base/global_costmap/obstacle_layer/laser_scan_sensor/marking: True
 * /move_base/global_costmap/obstacle_layer/laser_scan_sensor/sensor_frame: laser
 * /move_base/global_costmap/obstacle_layer/laser_scan_sensor/topic: scan
 * /move_base/global_costmap/obstacle_layer/observation_sources: laser_scan_sensor
 * /move_base/global_costmap/obstacle_range: 0.5
 * /move_base/global_costmap/plugins: [{'name': 'static...
 * /move_base/global_costmap/publish_frequency: 30.0
 * /move_base/global_costmap/raytrace_range: 0.5
 * /move_base/global_costmap/robot_base_frame: base_link
 * /move_base/global_costmap/rolling_window: False
 * /move_base/global_costmap/static_layer/map_topic: /map
 * /move_base/global_costmap/static_layer/subscribe_to_updates: False
 * /move_base/global_costmap/subscribe_to_updates: True
 * /move_base/global_costmap/update_frequency: 30.0
 * /move_base/local_costmap/footprint: [[-0.175, -0.095]...
 * /move_base/local_costmap/global_frame: odom
 * /move_base/local_costmap/inflation_layer/inflation_radius: 0.2
 * /move_base/local_costmap/local_costmap/height: 1.0
 * /move_base/local_costmap/local_costmap/inflation_radius: 0.1
 * /move_base/local_costmap/local_costmap/obstacle_layer/laser_scan_sensor/clearing: True
 * /move_base/local_costmap/local_costmap/obstacle_layer/laser_scan_sensor/data_type: LaserScan
 * /move_base/local_costmap/local_costmap/obstacle_layer/laser_scan_sensor/marking: True
 * /move_base/local_costmap/local_costmap/obstacle_layer/laser_scan_sensor/sensor_frame: laser
 * /move_base/local_costmap/local_costmap/obstacle_layer/laser_scan_sensor/topic: scan
 * /move_base/local_costmap/local_costmap/obstacle_layer/observation_sources: laser_scan_sensor
 * /move_base/local_costmap/local_costmap/plugins: [{'name': 'obstac...
 * /move_base/local_costmap/local_costmap/publish_frequency: 30.0
 * /move_base/local_costmap/local_costmap/resolution: 0.1
 * /move_base/local_costmap/local_costmap/rolling_window: True
 * /move_base/local_costmap/local_costmap/static_map: False
 * /move_base/local_costmap/local_costmap/transform_tolerance: 0.2
 * /move_base/local_costmap/local_costmap/update_frequency: 30.0
 * /move_base/local_costmap/local_costmap/width: 1.0
 * /move_base/local_costmap/map_topic: /map
 * /move_base/local_costmap/obstacle_layer/laser_scan_sensor/clearing: True
 * /move_base/local_costmap/obstacle_layer/laser_scan_sensor/data_type: LaserScan
 * /move_base/local_costmap/obstacle_layer/laser_scan_sensor/marking: True
 * /move_base/local_costmap/obstacle_layer/laser_scan_sensor/sensor_frame: laser
 * /move_base/local_costmap/obstacle_layer/laser_scan_sensor/topic: scan
 * /move_base/local_costmap/obstacle_layer/observation_sources: laser_scan_sensor
 * /move_base/local_costmap/obstacle_range: 0.5
 * /move_base/local_costmap/plugins: [{'name': 'static...
 * /move_base/local_costmap/publish_frequency: 30.0
 * /move_base/local_costmap/raytrace_range: 0.5
 * /move_base/local_costmap/robot_base_frame: base_link
 * /move_base/local_costmap/rolling_window: False
 * /move_base/local_costmap/static_layer/map_topic: /map
 * /move_base/local_costmap/static_layer/subscribe_to_updates: False
 * /move_base/local_costmap/subscribe_to_updates: True
 * /move_base/local_costmap/update_frequency: 30.0
 * /rosdistro: noetic
 * /rosversion: 1.15.13

NODES
  /
    amcl (amcl/amcl)
    map_server (map_server/map_server)
    map_to_odom (tf/static_transform_publisher)
    move_base (move_base/move_base)

ROS_MASTER_URI=http://localhost:11311

process[map_to_odom-1]: started with pid [48435]
process[map_server-2]: started with pid [48436]
process[amcl-3]: started with pid [48437]
process[move_base-4]: started with pid [48442]
[ WARN] [1638464196.169837099]: ignoring NAN in initial pose Yaw
[ INFO] [1638464196.287399492]: Requesting the map...
[ INFO] [1638464196.313759782]: Received a 416 X 640 map @ 0.050 m/pix

[ WARN] [1638464196.345304781]: ignoring NAN in initial pose Yaw
[ INFO] [1638464196.361098159]: Initializing likelihood field model; this can take some time on large maps...
[ WARN] [1638464196.397680202]: global_costmap: Pre-Hydro parameter "static_map" unused since "plugins" is provided
[ INFO] [1638464196.418392812]: global_costmap: Using plugin "static_layer"
[ INFO] [1638464196.424631443]: Done initializing likelihood field model.
[ INFO] [1638464196.501665986]: Requesting the map...
[ INFO] [1638464196.736119934]: Resizing costmap to 416 X 640 at 0.050000 m/pix
[ INFO] [1638464196.835334372]: Received a 416 X 640 map at 0.050000 m/pix
[ INFO] [1638464196.847959390]: global_costmap: Using plugin "obstacle_layer"
[ INFO] [1638464196.899974281]:     Subscribed to Topics: laser_scan_sensor
[ INFO] [1638464197.003377577]: global_costmap: Using plugin "inflation_layer"
[ WARN] [1638464197.209364390]: local_costmap: Pre-Hydro parameter "static_map" unused since "plugins" is provided
[ INFO] [1638464197.214969552]: local_costmap: Using plugin "static_layer"
[ INFO] [1638464197.244538406]: Requesting the map...
[ INFO] [1638464197.253843010]: Resizing costmap to 416 X 640 at 0.050000 m/pix
[ INFO] [1638464197.353163205]: Received a 416 X 640 map at 0.050000 m/pix
[ INFO] [1638464197.370877137]: local_costmap: Using plugin "obstacle_layer"
[ INFO] [1638464197.403102547]:     Subscribed to Topics: laser_scan_sensor
[ INFO] [1638464197.480452991]: local_costmap: Using plugin "inflation_layer"
[ INFO] [1638464197.658242199]: Created local_planner dwa_local_planner/DWAPlannerROS
[ INFO] [1638464197.757200327]: Sim period is set to 0.20
[ INFO] [1638464198.112770374]: Recovery behavior will clear layer 'obstacles'
[ INFO] [1638464198.141686037]: Recovery behavior will clear layer 'obstacles'
[ INFO] [1638464198.284117548]: odom received!
[ WARN] [1638464212.842819879]: Costmap2DROS transform timeout. Current time: 1638464212.8426, global_pose stamp: 1638464212.6399, tolerance: 0.2000
[ WARN] [1638464212.884832681]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464213.851451649]: Costmap2DROS transform timeout. Current time: 1638464213.8513, global_pose stamp: 1638464212.8384, tolerance: 0.2000
[ WARN] [1638464213.984292599]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464215.143252812]: Costmap2DROS transform timeout. Current time: 1638464215.1429, global_pose stamp: 1638464214.9397, tolerance: 0.2000
[ WARN] [1638464215.184205210]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464217.242917048]: Costmap2DROS transform timeout. Current time: 1638464217.2426, global_pose stamp: 1638464217.0398, tolerance: 0.2000
[ WARN] [1638464217.284707076]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464218.243514559]: Costmap2DROS transform timeout. Current time: 1638464218.2433, global_pose stamp: 1638464218.0400, tolerance: 0.2000
[ WARN] [1638464218.284937799]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464221.143795947]: Costmap2DROS transform timeout. Current time: 1638464221.1435, global_pose stamp: 1638464220.9398, tolerance: 0.2000
[ WARN] [1638464221.184432518]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464222.151450483]: Costmap2DROS transform timeout. Current time: 1638464222.1513, global_pose stamp: 1638464221.2398, tolerance: 0.2000
[ WARN] [1638464222.384176007]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464223.151450121]: Costmap2DROS transform timeout. Current time: 1638464223.1513, global_pose stamp: 1638464222.6399, tolerance: 0.2000
[ WARN] [1638464223.384715375]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464224.152252742]: Costmap2DROS transform timeout. Current time: 1638464224.1520, global_pose stamp: 1638464223.8398, tolerance: 0.2000
[ WARN] [1638464227.342870553]: Costmap2DROS transform timeout. Current time: 1638464227.3426, global_pose stamp: 1638464227.1397, tolerance: 0.2000
[ WARN] [1638464227.385268476]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464228.442879654]: Costmap2DROS transform timeout. Current time: 1638464228.4426, global_pose stamp: 1638464228.2397, tolerance: 0.2000
[ WARN] [1638464228.484623387]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464229.451585616]: Costmap2DROS transform timeout. Current time: 1638464229.4514, global_pose stamp: 1638464228.9398, tolerance: 0.2000
[ WARN] [1638464229.584818392]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464230.642883265]: Costmap2DROS transform timeout. Current time: 1638464230.6426, global_pose stamp: 1638464230.4397, tolerance: 0.2000
[ WARN] [1638464230.684697145]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464231.231281619]: Failed to transform initial pose in time (Lookup would require extrapolation -0.578186683s into the future.  Requested time 1638464230.726857901 but the latest data is at time 1638464230.148671150, when looking up transform from frame [base_link] to frame [odom])
[ INFO] [1638464231.231757757]: Setting pose (1638464231.231656): -0.016 0.015 -0.003
[ WARN] [1638464231.642924055]: Costmap2DROS transform timeout. Current time: 1638464231.6426, global_pose stamp: 1638464230.4397, tolerance: 0.2000
[ WARN] [1638464231.785980722]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464232.651441263]: Costmap2DROS transform timeout. Current time: 1638464232.6513, global_pose stamp: 1638464231.7400, tolerance: 0.2000
[ WARN] [1638464233.284161224]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464233.652227206]: Costmap2DROS transform timeout. Current time: 1638464233.6519, global_pose stamp: 1638464233.0399, tolerance: 0.2000
[ WARN] [1638464234.284200682]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464234.676149394]: Costmap2DROS transform timeout. Current time: 1638464234.6760, global_pose stamp: 1638464233.8398, tolerance: 0.2000
[ WARN] [1638464235.285087642]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464235.842836831]: Costmap2DROS transform timeout. Current time: 1638464235.8426, global_pose stamp: 1638464235.6398, tolerance: 0.2000
[ WARN] [1638464236.384353818]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464240.413772576]: Failed to transform the goal pose from map into the odom frame: Lookup would require extrapolation -0.120934254s into the future.  Requested time 1638464240.411875248 but the latest data is at time 1638464240.290941000, when looking up transform from frame [map] to frame [odom]
[ WARN] [1638464240.414422655]: Failed to transform the goal pose from map into the odom frame: Lookup would require extrapolation -0.120934254s into the future.  Requested time 1638464240.411875248 but the latest data is at time 1638464240.290941000, when looking up transform from frame [map] to frame [odom]
[ERROR] [1638464240.414839924]: The goal pose passed to this planner must be in the odom frame.  It is instead in the map frame.
[ INFO] [1638464240.814339427]: Got new plan
[ WARN] [1638464241.118340913]: Map update loop missed its desired rate of 30.0000Hz... the loop actually took 0.3002 seconds
[ WARN] [1638464241.119222061]: Control loop missed its desired rate of 5.0000Hz... the loop actually took 0.3051 seconds
[ WARN] [1638464241.366497437]: Map update loop missed its desired rate of 30.0000Hz... the loop actually took 0.2149 seconds
[ WARN] [1638464241.367177608]: Control loop missed its desired rate of 5.0000Hz... the loop actually took 0.3531 seconds
[ WARN] [1638464241.602062831]: Map update loop missed its desired rate of 30.0000Hz... the loop actually took 0.2021 seconds
[ WARN] [1638464241.603498598]: Control loop missed its desired rate of 5.0000Hz... the loop actually took 0.3890 seconds
[ WARN] [1638464241.865361062]: Map update loop missed its desired rate of 30.0000Hz... the loop actually took 0.2301 seconds
[ WARN] [1638464241.866151453]: Control loop missed its desired rate of 5.0000Hz... the loop actually took 0.4520 seconds
[ INFO] [1638464241.866392707]: Got new plan
[ WARN] [1638464242.102896804]: Map update loop missed its desired rate of 30.0000Hz... the loop actually took 0.2042 seconds
[ WARN] [1638464242.103524347]: Control loop missed its desired rate of 5.0000Hz... the loop actually took 0.2374 seconds
[ WARN] [1638464242.382397906]: Map update loop missed its desired rate of 30.0000Hz... the loop actually took 0.2462 seconds
[ WARN] [1638464242.383067744]: Control loop missed its desired rate of 5.0000Hz... the loop actually took 0.3169 seconds
[ WARN] [1638464242.614643813]: Map update loop missed its desired rate of 30.0000Hz... the loop actually took 0.1989 seconds
[ WARN] [1638464242.615280170]: Control loop missed its desired rate of 5.0000Hz... the loop actually took 0.3492 seconds
[ WARN] [1638464242.871001964]: Map update loop missed its desired rate of 30.0000Hz... the loop actually took 0.2229 seconds
[ WARN] [1638464242.871679802]: Control loop missed its desired rate of 5.0000Hz... the loop actually took 0.4055 seconds
[ INFO] [1638464242.871944056]: Got new plan
[ WARN] [1638464243.112923265]: Map update loop missed its desired rate of 30.0000Hz... the loop actually took 0.2088 seconds
[ WARN] [1638464243.113576900]: Control loop missed its desired rate of 5.0000Hz... the loop actually took 0.2419 seconds
[ WARN] [1638464243.242711873]: Costmap2DROS transform timeout. Current time: 1638464243.2426, global_pose stamp: 1638464243.0398, tolerance: 0.2000
[ WARN] [1638464243.284626510]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464243.431740650]: Control loop missed its desired rate of 5.0000Hz... the loop actually took 0.3600 seconds
[ WARN] [1638464243.432086791]: Map update loop missed its desired rate of 30.0000Hz... the loop actually took 0.2859 seconds
[ WARN] [1638464243.432376008]: Transform timeout for global_costmap. Current time: 1638464243.4320, pose stamp: 1638464243.0398, tolerance: 0.2000
[ERROR] [1638464243.433099067]: Could not get robot pose
[ERROR] [1638464243.433894773]: Could not get robot pose
[ WARN] [1638464243.434603295]: Unable to get starting pose of robot, unable to create global plan
[ WARN] [1638464243.472133208]: Unable to get starting pose of robot, unable to create global plan
[ WARN] [1638464243.672210240]: Unable to get starting pose of robot, unable to create global plan
[ WARN] [1638464243.872161238]: Unable to get starting pose of robot, unable to create global plan
[ WARN] [1638464244.072058866]: Unable to get starting pose of robot, unable to create global plan
[ WARN] [1638464244.242806816]: Costmap2DROS transform timeout. Current time: 1638464244.2426, global_pose stamp: 1638464243.3397, tolerance: 0.2000
[ WARN] [1638464244.271916810]: Unable to get starting pose of robot, unable to create global plan
[ WARN] [1638464244.384532974]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464244.471850067]: Transform timeout for global_costmap. Current time: 1638464244.4717, pose stamp: 1638464243.3397, tolerance: 0.2000
[ WARN] [1638464244.472148042]: Unable to get starting pose of robot, unable to create global plan
[ INFO] [1638464244.872079074]: Got new plan
[ WARN] [1638464245.252120663]: Map update loop missed its desired rate of 30.0000Hz... the loop actually took 0.3533 seconds
[ WARN] [1638464245.252813000]: Control loop missed its desired rate of 5.0000Hz... the loop actually took 0.3811 seconds
[ WARN] [1638464245.530619285]: Control loop missed its desired rate of 5.0000Hz... the loop actually took 0.4588 seconds
[ WARN] [1638464245.819963592]: Control loop missed its desired rate of 5.0000Hz... the loop actually took 0.2895 seconds
[ INFO] [1638464245.820910573]: Got new plan
[ WARN] [1638464245.826844432]: Map update loop missed its desired rate of 30.0000Hz... the loop actually took 0.5349 seconds
[ WARN] [1638464246.136268752]: Control loop missed its desired rate of 5.0000Hz... the loop actually took 0.4056 seconds
[ WARN] [1638464246.139770661]: Map update loop missed its desired rate of 30.0000Hz... the loop actually took 0.3167 seconds
[ WARN] [1638464246.401831604]: Control loop missed its desired rate of 5.0000Hz... the loop actually took 0.2657 seconds
[ WARN] [1638464246.403612252]: Map update loop missed its desired rate of 30.0000Hz... the loop actually took 0.2656 seconds
[ WARN] [1638464246.658940870]: Control loop missed its desired rate of 5.0000Hz... the loop actually took 0.3229 seconds
[ WARN] [1638464246.659286844]: Map update loop missed its desired rate of 30.0000Hz... the loop actually took 0.2567 seconds
[ WARN] [1638464246.945369866]: Control loop missed its desired rate of 5.0000Hz... the loop actually took 0.4090 seconds
[ WARN] [1638464246.946531398]: Map update loop missed its desired rate of 30.0000Hz... the loop actually took 0.2871 seconds
[ INFO] [1638464246.947581080]: Got new plan
[ WARN] [1638464247.207661193]: Control loop missed its desired rate of 5.0000Hz... the loop actually took 0.2626 seconds
[ WARN] [1638464247.207997334]: Map update loop missed its desired rate of 30.0000Hz... the loop actually took 0.2282 seconds
[ WARN] [1638464247.472013625]: Control loop missed its desired rate of 5.0000Hz... the loop actually took 0.3269 seconds
[ WARN] [1638464247.472630113]: Map update loop missed its desired rate of 30.0000Hz... the loop actually took 0.2646 seconds
[ INFO] [1638464247.745402594]: Got new plan
[ INFO] [1638464248.345337456]: Goal reached
[ WARN] [1638464265.442815785]: Costmap2DROS transform timeout. Current time: 1638464265.4426, global_pose stamp: 1638464265.2397, tolerance: 0.2000
[ WARN] [1638464265.684311764]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464270.342673706]: Costmap2DROS transform timeout. Current time: 1638464270.3425, global_pose stamp: 1638464270.1397, tolerance: 0.2000
[ WARN] [1638464270.384169945]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464271.342701349]: Costmap2DROS transform timeout. Current time: 1638464271.3426, global_pose stamp: 1638464270.4385, tolerance: 0.2000
[ WARN] [1638464271.384384215]: Could not get robot pose, cancelling reconfiguration
[ WARN] [1638464272.346996646]: Costmap2DROS transform timeout. Current time: 1638464272.3468, global_pose stamp: 1638464271.8398, tolerance: 0.2000
^C[move_base-4] killing on exit
[amcl-3] killing on exit
[map_to_odom-1] killing on exit
[map_server-2] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
```
Looking at the output above, it can be noticed that certain warnings occur in pairs:
```
Costmap2DROS transform timeout
Could not get robot pose, cancelling reconfiguration
```
and these occur roughly once every second or two, mostly before the robot receives its goal.

Also, an analysis of the above output shows a total of 24 of these warnings,
of which 12 only missed the .2 sec timeout by less than 0.01 sec. So just bumping the timeout
up to 0.21 will cut the number of these warnings in half.

Once the robot begins driving to the goal, most of the warnings were of the fololowing type (again occuring in pairs):
```
Map update loop missed its desired rate of 30.0000Hz...
Control loop missed its desired rate of 5.0000Hz...
```
Doing a google search turned up [Navigation Tuning](http://zdome.net/wiki/index.php/Navigation_Tuning) which shows this:

```
loop missed its desired rate

 [ WARN] [1472599582.846147341]: Map update loop missed its desired rate of 3.0000Hz... 
                                 the loop actually took 0.6902 seconds
 [ WARN] [1472599583.225435205]: Control loop missed its desired rate of 20.0000Hz... 
                                 the loop actually took 0.0500 seconds

I believe these went away by specifying the following in the files below. Values were blindly taken from the Linorobot. The Linorobot is using an Arm processor. The NUC is an i5. So these values need to be tuned.

File global_costmap_params.yaml:

 update_frequency: 1.0
 publish_frequency: 0.5

File local_costmap_params.yaml:

 update_frequency: 1.0
 publish_frequency: 5.0

File move_base_params.yaml:

 controller_frequency: 3.0
 planner_frequency: 1.0
```
Once the robot reached its goal, there was a period of nearly 20 seconds with no warnings.

In summary, it looks like:
* The map update frequency should be changed to 4 Hz (from 30 Hz) and
* The transform tolerance should be changed to 0.21 (from 0.2).
