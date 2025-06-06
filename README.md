# FAA_Recognition_Draft
***
## Dependencies
The Gazebo Simulations of Detect and Avoid (DAA), focused on generating detection confidence levels under varying visual environmental factors (such as visual clutter, fog, and camera noise), require several Python libraries and ROS 2 dependencies:

### ROS 2 Dependencies
The project requires ROS 2 Humble and Gazebo Classic software for the implementation of the simulations.

### OpenCV
This project depends on OpenCV as a bridge between ROS 2 camera messages and RGB image formats. OpenCV is used to convert sensor_msgs/Image messages into formats that can be processed by the YOLO object detection model, and to generate videos of the camera sensors during simulations.

```bash
pip install opencv-python
```

### YOLO
The simulations use YOLO as the visual detection system for identifying aircraft in the camera field of view of the own-ship.

```bash
pip install ultralytics
```
### Gazebo-dev
gazebo-dev is used as a dependency to build custom Gazebo plugins and install them into the Gazebo plugin library.

```bash
sudo apt update
sudo apt install libgazebo11-dev
```

### Unzipping larger files:
The Installation of the models used in the worlds downloads big zip files from Google Drive, gdown is required to accept the download when the setup.sh is running.
```bash
pip install gdown
```

### Recursive Sub-modules
The simulations used a customized version of the plugin [GPS Multipath Plugin](https://github.com/landwy/gnss_multipath_plugin) for Gazebo, redesigned to be added to any model and to calculate a velocity estimation. This plugin depends on [libpredict](https://github.com/la1k/libpredict). Both of these packages are added as recursive sub-modules to the repository.



***
## Installation 

The installation process requires cloning the repository with its recursive sub-modules and running the `setup.sh` script to install additional large files into the `src` directory.

### Steps:

```bash
git clone --recurse-submodules https://github.com/Darkjrcy/FAA_Recognition_Draft.git
cd FAA_Recognition_Draft
bash setup.sh
```

***
## Generate New Worlds
The process to generate new worlds for you Gazebo Simulation is presented in [World Generator](src/plane_bringup/README.md)


***
## Run the Detection Simulations
The Repository contains two main detection simulations models:

### One Camera Detection Simulation
The first simulation uses a single camera to detect airplanes with YOLO, using the closest approach point from the MIT Detect and Avoid (DAA) trajectories. The results are saved in DATA/MIT_One_Camera_Recognition. Inside this directory, the Test folder contains two detection result files:

* Detection_events.csv: a description of detection events
* Detection_process.csv: simultaneous detection process result

Run this launch file to run the simulation:

```bash
ros2 launch plane_bringup MIT_one_camera_recognition.launch.py 
```

### Casia X Camera Detection Simulation
There is an additional detection simulation based on the closest approach point from the MIT Detect and Avoid (DAA) trajectories, which uses a realistic camera configuration of 5 cameras (Casia X). The resulting detection files follow the same structure as the previous simulation and are saved in FOLDER/MIT_Recognition. To run this simulation, use the next command.

```bash
ros2 launch plane_bringup MIT_two_airplanes_with_recog.launch.py
```

### Modification inside the Simulations
There are 4 easier changes that can be done to the simulation.

#### 1. World
If you want to change the world used by one of the launch files, open the corresponding launch file, locate the world configuration parameters, and change the file name.

![P1 (29)](https://github.com/user-attachments/assets/4576228f-bbd7-4a07-9357-03e9a67a0fff)


#### 2. Fog density
The fog density value used for the data recollection is  changed in the same section that the world file is changed. However, to adjust the fog density in the simulation, open the world file, navigate to the <scene> section, and modify the fog density value to match the input specified in the launch file parameters. For exmaple:

```xml
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
      <fog>
        <color>0.7 0.7 0.7 1</color>
        <type>exp</type>
        <density>0.0</density> <!-- Change this value so it matches with the input of the Characteristics-->
      </fog>
    </scene>
```

### 3. Camera noise
To change the zero mean standard deviation of the camera noise go to the gazebo.xacro file of the Airplane model locates in src/plane_description/urdf/.../airplane_gazebo.xacro and change the camera_noise_stddev property, so it matches the input specified in the launch file parameters. For exmaple:

```xml
<?xml version="1.0"?>
<robot name="airplane_sim" xmlns:xacro="http://ros.org/wiki/xacro">

    <xacro:arg name="robot_name" default="airplane_X"/>
    <xacro:arg name="robot_imu" default="imu_X"/>
    <xacro:property name="camera_noise_stddev" default="0.05"/> <!-- Change this value to add or decrease the zero mean stadard deviation noise of the camera -->
```






