# ROS Environment for AMiRo
This catkin_ws is a ros environment which contains packages that handle the conversion from the outgoing rsb data (like odometry or laserscaner) from the AMiRo to ros and otherwise to control the AMiRo.

## How To
Make sure that you sourced the setup.bash like 'source devel/setup.bash'. Otherwise ros doesn't know the environment.
There are several parameter you have to adapt to your scenario in the 'start.launch'.

#####Parameter start.launch
|         Name         | Default |                                                         Description                                                          |
| -------------------- | ------- | ---------------------------------------------------------------------------------------------------------------------------- |
| rostimenow           | false   | If this is set to true ros::time::now will be used as timestmap otherwise the timestmap will be converted from the rsb data. |
| xboxController       | 0       | To controll the AMiRo with an xboxController                                                                                 |
| laserscaner          | 0       | If you want to use a laserscaner. With laserscan_sick and laserscan_hokuyo you can controll the which laserscan is used.     |
| laserscan_sick       | 0       | Decide to use the sick tim laserscaner.                                                                                      |
| laserscan_hokuyo     | 0       | Decide to use the hokuyo laserscanner.                                                                                       |
| twb                  | 0       | Use the TeleWorkBench data .                                                                                                 |
| robot_localization   | 0       | Use the robot_localization packge from ros which is a kalman-filter. It combines the AMiRo-odometry and the twb-data.        |
| dynamic_tf_with_odom | 0       | This is a dynamic transformer to update the tf from amiro_odom and amiro_base_link with AMiRo odometry data                  |
| dynamic_tf_with_ekf  | 0       | the same as dynamic-tf_with_odom but with the ekf data.                                                                      |
| static_tf_map_odom   | 0       | defines a static transform between map and amiro_odom                                                                        |

As default there are only converter for one AMiRo. If you want to add more AMiRo you ahve to edit the 'start.launch' and copy a block of the 'amiro.launch' as following and add it under:
```
<include file="$(env MUROX_ROS)/amiro.launch">
  <arg name="amiroNr" value="1"/>
  <arg name="markerId" value="1"/>

  <arg name="laserscan" value="$(arg laserscan)"/>
  <arg name="laserscan_sick" value="$(arg laserscan_sick)"/>
  <arg name="laserscan_hokuyo" value="$(arg laserscan_hokuyo)"/>
  <arg name="camera" value="0"/>

  <arg name="twb" value="$(arg twb)"/>
  <arg name="robot_localization" value="$(arg robot_localization)"/>

  <arg name="rostimenow" value="$(arg rostimenow)"/>

  <arg name="dynamic_tf_with_odom" value="$(arg dynamic_tf_with_odom)"/>
  <arg name="dynamic_tf_with_ekf" value="$(arg dynamic_tf_with_ekf)"/>
  <arg name="static_tf_map_odom" value="$(arg static_tf_map_odom)"/>

  <arg name="xbox_controller" value="$(arg xboxController)"/>
  <arg name="keyboard_controller" value="0"/>

  <arg name="nav_stack" value="0"/>
  <arg name="no_static_map" value="0"/>

  <arg name="hector_mapping" value="0"/>
  <arg name="octomap" value="0"/>
  <arg name="gmapping" value="0"/>

</include>
```

#####Parameter amiro.launch
|         Name         | Default |                                                   Description                                                   |
| -------------------- | ------- | --------------------------------------------------------------------------------------------------------------- |
| amiroNr              | 1       | Set the amiroId as namespace for all bridges.                                                                   |
| markerId             | 1       | Set the markerId from the twb data.                                                                             |
| laserscan            | 0       | Toggle the laserscaner.                                                                                         |
| laserscan_sick       | 0       | Toggle the sick tim laserscanner.                                                                               |
| laserscan_hokuyo     | 0       | Toggle the hokuyo laserscanner.                                                                                 |
| camera               | 0       | The the vision bridge for the inbuild AMiRo camera.                                                             |
| twb                  | 0       | Toggle the bridge for the twb data.                                                                             |
| robot_localization   | 0       | Toggle the extended kalman filter for AMiRo odometry and twb data.                                              |
| rostimeno            | 0       | Toggle the header stimestamps of each message with rostimenow or the normal rsb timestamps.                     |
| dynamic_tf_with_odom | 0       | This is a dynamic transformer to update the tf from amiro_odom and amiro_base_link with AMiRo odometry data     |
| dynamic_tf_with_ekf  | 0       | the same as dynamic-tf_with_odom but with the ekf data.                                                         |
| static_tf_map_odom   | 0       | defines a static transform between map and amiro_odom                                                           |
| xbox_controller      | 0       | Toggle the bridges to control the AMiRo with an xbox controller.                                                |
| keyboard_controller  | 0       | Toggle the bridges to control the AMiRo with a keyboard.                                                        |
| nav_stack            | 0       | Toggle the ros navigation stack for the AMiRo.                                                                  |
| no_static_map        | 0       | If there is a dynamic map while SLAM'ing this has to be set 0 otherwise if there is a static map set this to 1. |
| hector_mapping       | 0       | Toggle SLAM'ing with the ros package hector_mapping.                                                            |
| octomap              | 0       | Toggle the ros package octomap with creates a map from the AMiRo odometry or the combined ekf data.             |
| gmapping             | 0       | Toggle SLAM'ing with the ros package gmapping.                                                                  |

### MapEvaluation
There is also a detailed custom coded package to evaluate the creates maps from the mapping algorithms. This contains also a README.md for more informations.
