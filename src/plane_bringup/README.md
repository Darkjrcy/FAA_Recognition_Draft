# Adding a city to Gazebo
## 1. Generate the 3D model of the city
There are several ways to generate cities inside Gazebo from OpenStreetMaps (OSM). Here is a simpler and fast way to generate these cities into a Gazebo world, to use them for simulations. The first step to add the OSM file into the Gazebo world, is to generate the city 3D model using blender (Steps: https://www.youtube.com/watch?v=ZsLMt3Ka8UA&t=700s) 

## 2. Spawn the 3D model in Gazebo
There are several ways to spawn a 3D model into Gazebo, the best two are by generating a model configured to be used inside the Gazebo software (recommended), or spawn the 3D model directly in your URDF file (easier).  

### 2.1 Generate the model directly in Gazebo (Recommended)

First, create a folder called models into the same package you saved your worlds, be sure you install the folder inside your CMake list. 

```txt
cmake_minimum_required(VERSION 3.8)
project(plane_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Add models into the installation folder here:
install(
  DIRECTORY 
  launch 
  worlds
  models 
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

```

Inside your models folder generate a new folder with the name you want to call your model inside the Gazebo software. Copy your dae file with their textures inside your new folder, and create two files: a sdf file and model.config

![cmake_minimum_required(VERSION 3 8) project(plane_bringup) if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES Clang) add_compile_options(-Wall -Wextra -Wpedantic) endif() # find dependen](https://github.com/user-attachments/assets/fc85f568-716e-4353-9891-e6c63b56e4c0)

#### Model sdf file

The sdf file is responsible of telling Gazebo which dae file is used to generate the visual and collision properties of your model from your dae file. Here is a template, where we use a Daytona.dae.

```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="Daytona">
    <!-- Pose of the model respect to the origin-->
    <pose>0 0 0 0 0 0</pose>
    <!-- Make the city static -->
    <static>true</static>
    <!-- Name of the link you want to add the 3D model-->
    <link name="city">
      <!-- Generate the mesh so the city is visible using the .dae format-->
      <visual name="visual">
        <!-- Geometry is the property that finds the 3D modeling of the city,where the path
            to the dae file starts from the models folder-->
        <geometry>
          <mesh><uri>model://Daytona/Daytona.dae</uri></mesh>
        </geometry>
        <pose>0 0 -3.5 0 0 0</pose>
      </visual>
      <!-- Collision gives the de file mass and inertia-->
      <collision name="collision">
        <geometry>
          <mesh><uri>model://Daytona/Daytona.dae</uri></mesh>
        </geometry>
        <pose>0 0 -3.5 0 0 0</pose>
      </collision>
    </link>
  </model>
</sdf>
```
#### Config file:
The config file makes it possible to read the sdf file directly from the Gazebo program. Here is a template using the Daytona model. 
```xml
<?xml version="1.0"?>

<model>
  <name>Daytona</name>
  <version>1.0</version>
  <sdf version="1.5">Daytona.sdf</sdf>
</model>
```
### Add the model to a Gazebo World
Finally, to open the model into Gqazebo, open the Gazebo software, and go to insert.

![cmake_minimum_required(VERSION 3 8) project(plane_bringup) if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES Clang) add_compile_options(-Wall -Wextra -Wpedantic) endif() # find dependen(1)](https://github.com/user-attachments/assets/34b313db-e9b1-4313-8b3a-245a59b38a37)

Using Add Paths open your models folder.

![cmake_minimum_required(VERSION 3 8) project(plane_bringup) if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES Clang) add_compile_options(-Wall -Wextra -Wpedantic) endif() # find dependen(2)](https://github.com/user-attachments/assets/3ff8ea77-27b4-4903-94a2-f4b018e4d5fa)

You now have access to all the 3D models within the models folder. To create a custom world, simply add the desired 3D models into Gazebo, arrange them as needed, and save the world file into the worlds folder. This file can be used later into simulations to spawn your robots.

![cmake_minimum_required(VERSION 3 8) project(plane_bringup) if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES Clang) add_compile_options(-Wall -Wextra -Wpedantic) endif() # find dependen(3)](https://github.com/user-attachments/assets/454b1f53-008b-418a-b55c-f46e2c77b340)





### 2.2 Use a URDF file to spawn the Gazebo model into a World
First, add the city model in stl or dae format to your meshes folder. Then, create a joint between a dummy link and your city inside your URDF file. 
```xml
  <!-- LINKS OF THE URDF-->
    <!-- Base dummy link-->
    <link name="base_link"/>
    <!-- Body CIty Link-->
    <link name="city_link">
        <!-- Generate the mesh so the city is visible using the .dae format-->
        <visual>
            <!-- Geometry is the property that finds the 3D modeling of the city,where city_description 
            is the package name where your meshes folder is located and installed-->
            <geometry>
                <mesh filename="file://$(find city_description)/meshes/city.dae" scale="1 1 1"/> <!-- In case of .dae file-->
                <!-- Alternative: <mesh filename="package://city_description/meshes/city.stl" scale="1 1 1"/>-->
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        </visual>
        <collision>
            <!-- Generate the collision so the buildings interact to other models int he simulation 
            with your dae file-->
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
            <geometry>
                <mesh filename="file://$(find city_description)/meshes/city.dae" scale="0.5 0.5 0.5"/>
            </geometry>
        </collision>
        <!-- You need to add intertia to your city so Gazebo knows is a real object; however, if you want
         to have realistic values is preferable to generate the models directly into Gazebo-->
        <inertial>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
            <mass value="100.0"/>
            <inertia ixx="1.0" ixy="-1.0" ixz="-1.0" iyy="1.0" iyz="-1.0" izz="1.0"/>
        </inertial>
    </link>


    <!--JOINTS OF THE URDF-->
    <!-- City joint to the Base Link that helps to see where the city is located with respect to the origin-->
    <joint name="airplane_joint" type="fixed">
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <parent link="base_link"/>
        <child link="city_link"/>
    </joint>
```
Now, when you spawn your URDF file the city woudl be spawned in the origin. If you have questions in how to launch an URDF file into Gazebo see: https://www.youtube.com/watch?v=w6Kvq7ac-J8
