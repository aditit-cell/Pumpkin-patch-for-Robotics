# Pumpkin-patch-for-Robotics
Robotics Project
Terminal-1:open gazebo with a grassland environment:
````bash
pkill -9 gzserver
pkill -9 gzclient
ros2 launch gazebo_ros gazebo.launch.py world:=/home/aditipk/ros2_ws/src/crop_field/worlds/grass_world.sdf

````
Terminal-2 for spwan entity:
````bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run farm_world_spawner spawn_farm_environment
````
Terminal-3:for drone generation :
````bash
source /opt/ros/humble/setup.bash
ros2 run gazebo_ros spawn_entity.py \
  -entity my_drone_cam \
  -file ~/.gazebo/models/iris_static_cam/model.sdf \
  -x 0 -y 0 -z 1.0
````

For drone view (camera):
````bash
 ros2 run rqt_image_view rqt_image_view
````
Detection and spray:
````bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
 ros2 run pumpkin_sprayer spray_node
````
