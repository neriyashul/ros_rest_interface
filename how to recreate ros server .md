
# 1. install ros

```
source /opt/ros/noetic/setup.sh
```
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

# 2. install ros_rest_interface2 package

create package:
```
catkin_create_pkg ros_rest_interface2 roscpp geometry_msgs message_generation message_runtime roslint
```

build package: 
```
cd ..
catkin_make
```

# 3. Install Casablanca (cpprestsdk) 
```
sudo apt-get install libcpprest-dev
```

4. catkin_make && roslaunch ros_rest_interface ros_rest_interface.launch
