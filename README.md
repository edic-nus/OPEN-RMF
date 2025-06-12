**DISCLAIMER**: A lot of the configuration files in this branch include absolute paths based on my machine, so do note that you will need to manually go through the files and change the absolutes paths to ensure that they point to the approriate files in your machine's directory.

**Full set of commands to launch physical Turtlebot3 with Free Fleet Adapter in E2A:**

`ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/PATH/TO/WORKSPACE/src/ff_tests_maps/maps/test_map_E2A/test_map_E2A.yaml`

`zenohd` (on different ros domain)

`zenoh-bridge-ros2dds -c /PATH/TO/WORKSPACE/src/ff_tests/zenoh_configs/tb3_client_zenoh_config.json5`

`ros2 launch ff_tests tb3_home_rmf_common.launch.xml` (on different ros domain)

`ros2 launch ff_tests tb3_home_fleet_adapter.launch.xml`  (on different ros domain) 

`ros2 run rmf_demos_tasks dispatch_patrol   -p goal tb3_charger   -n 1   -st 0` (on different ros_domain_id)

**Full set of commands to launch Gazebo Sim of E2A with Turtlebot3 and Free Fleet Adapter:**

`ros2 launch ff_tests_gz test_map_E2A.launch.xml headless:=0`

`ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=/PATH/TO/WORKSPACE/src/ff_tests_maps/maps/test_map_E2A/test_map_E2A.yaml`

*Localize the robot using AMCL in Rviz*

`zenohd` (on same ros domain)

`zenoh-bridge-ros2dds -c /PATH/TO/WORKSPACE/src/ff_tests/zenoh_configs/tb3_client_zenoh_config.json5` 

`ros2 launch ff_tests tb3_E2A_rmf_common.launch.xml` (on same ros domain)

`ros2 launch ff_tests tb3_E2A_fleet_adapter.launch.xml` (on same ros domain)

**Also do note that once the workspace is built**, you will need to manually add the following in your world file in the install directory (PATH-TO-WORKSPACE/install/share/ff_tests_maps/maps/test_map_E2A/test_map_E2A.world) in order to make scan and imu work for the Turtlebot3:
```
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"> <render_engine>ogre2</render_engine>
</plugin>
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu">
</plugin>
```
`ros2 run rmf_demos_tasks dispatch_patrol   -p goal tb3_charger   -n 1   -st 0` (on same ros domain)

**Lastly, in order to make a map of the environment whether in Gazebo or reality, run the following commands:**

`ros2 launch turtlebot3_navigation2 navigation2.launch.py`

`ros2 launch turtlebot3_cartographer cartographer.launch.py`

Run this command to save the map as a png (remove the argument if you want it as a pgm file):

`run nav2_map_server map_saver_cli -f ~/map --fmt png ros2`

Refer to Issues for troubleshooting regarding the free fleet adapter, RMF, Nav2, and Jazzy in general
