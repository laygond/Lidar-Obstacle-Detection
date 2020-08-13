# Sensor Fusion Self-Driving Car Course
Create 3D bounding boxes to enclose vehicles by implementing 3D RANSAC for segmentation and KD-Tree & Euclidean algorithms for clustering.

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />


## Directory Structure
```
.Lidar-Obstacle-Detection
├── build.sh
├── run.sh
├── CMakeLists.txt
├── README_images
│   └── ...
├── README.md
└── src
    ├── environment.cpp
    ├── processPointClouds.cpp
    ├── processPointClouds.h
    ├── quiz
    │   ├── cluster
    │   │   ├── cluster.cpp
    │   │   ├── CMakeLists.txt
    │   │   └── kdtree.h
    │   └── ransac
    │       ├── CMakeLists.txt
    │       └── ransac2d.cpp
    ├── render
    │   ├── box.h
    │   ├── render.cpp
    │   └── render.h
    └── sensors
        ├── data/pcd
        │   ├── data_1
		│	│	└── ...
        │   ├── data_2
		│	│	└── ...
        │   └── simpleHighway.pcd
        └── lidar.h
```

Code Structure
Top-level CMakeLists.txt
Readme
src
render
box.h - this file has the struct definitions for box objects
render.h
render.cpp - this file, along with the header, define the classes and methods for rendering objects.
sensors
data - this directory contains pcd data used in the course.
lidar.h - has functions using ray casting for creating pcd.
environment.cpp - the main file for using pcl viewer and processing and visualizing pcd.
processPointClouds.h
processPointClouds.cpp - Functions for filtering, segmenting, clustering, boxing, loading, and saving pcd.
Starter Repo Walkthrough

You will mostly be working out of two main files, which are environment.cpp and processPointClouds.cpp. The environment.cpp file contains the main function and will generate the runnable executable. The processPointClouds.cpp file will contain all your function placeholders to process the pcd.

There are some other files worth mentioning, like sensors/lidar.h, which simulates lidar sensing and creates point cloud data. Also render.cpp and render.h which have functions for rendering objects onto the screen.

(lidar and environment are simulated so data generation (point cloud) is simulated as well)

This repo creates its own [function templates](http://www.cplusplus.com/doc/oldtutorial/templates/) to process any type of PCD: PointXYZ, PointXYZI, etc

point cloud data which could be very large. By instatinating on the heap, we have more memory to work with than the 2MB on the stack. However, it takes longer to look up objects on the heap, while stack lookup is very fast.

## Simulator
Here you have a simple highway simulator environment with the ego car in green in the center lane (thats your car), and the other traffic cars in blue. Everything is rendered using PCL with simple boxes, lines, and colors. You can move around your environment with the mouse. Try holding the left mouse button to orbit around the scene. You can also pan around the scene by holding the middle mouse button and moving. To zoom, use the middle scroll mouse button or the right mouse button while moving.
Zoom: hold the right mouse key and move the mouse forward/backwards, or use your mouse scroller.
Pan: Hold down the middle mouse button (the scroller) and move the mouse.
Rotate: Hold the left mouse button and move the mouse.

Some functions that use the pcl viewer inside environment.cpp are initCamera and initHighway. The initCamera function helps you set up different viewing angles in your window. There are five different options: XY, TopDown, Side, and FPS. XY gives a 45 degree angle view, while FPS is First Person Sense and gives the sensation of being in the car’s driver seat.

PCL viewer is for graphics. it renders points and shapes. initCamera and initHighway from environment.cpp use it.

### Welcome to the Sensor Fusion course for self-driving cars.

In this course we will be talking about sensor fusion, whch is the process of taking data from multiple sensors and combining it to give us a better understanding of the world around us. we will mostly be focusing on two sensors, lidar, and radar. By the end we will be fusing the data from these two sensors to track multiple cars on the road, estimating their positions and speed.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

**Radar** data is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction. This ability makes radars a very pratical sensor for doing things like cruise control where its important to know how fast the car infront of you is traveling. Radar sensors are also very affordable and common now of days in newer cars.

**Sensor Fusion** by combing lidar's high resoultion imaging with radar's ability to measure velocity of objects we can get a better understanding of the sorrounding environment than we could using one of the sensors alone.


## Installation

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
$> cd SFND_Lidar_Obstacle_Detection
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```

### Windows 

http://www.pointclouds.org/downloads/windows.html

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh/)
2. update homebrew 
	```bash
	$> brew update
	```
3. add  homebrew science [tap](https://docs.brew.sh/Taps) 
	```bash
	$> brew tap brewsci/science
	```
4. view pcl install options
	```bash
	$> brew options pcl
	```
5. install PCL 
	```bash
	$> brew install pcl
	```

#### Prebuilt Binaries via Universal Installer
http://www.pointclouds.org/downloads/macosx.html  
NOTE: very old version 

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)
