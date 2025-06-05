# FAA_Recognition_Draft
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
