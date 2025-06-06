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
he first simulation uses a single camera to detect airplanes with YOLO, using the closest approach point from the MIT Detect and Avoid (DAA) trajectories. The results are saved in DATA/MIT_One_Camera_Recognition. Inside this directory, the Test folder contains two detection result files:

* Detection_events.csv: a description of detection events
* Detection_process.csv: simultaneous detection process result

