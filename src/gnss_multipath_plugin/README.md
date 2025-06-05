# gnss_multipath_plugin
GPS Multipath Plugin in Gazebo

Depends on libpredict: https://github.com/la1k/libpredict 
```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
```
Clone the libpredict library into the workspace as it is a dependency 
```
git clone https://github.com/la1k/libpredict.git
```
Clone the project:
```
git clone https://github.com/kpant14/gnss_multipath_plugin.git
cd ../
colcon build
```
Source the environment before executing the launch file
```
source ./install/setup.bash
```

Launch the project using either the HongKong model or the Purdue model 
```
ros2 launch gnss_multipath_plugin hong_kong.launch.py
ros2 launch gnss_multipath_plugin purdue.launch.py
```
