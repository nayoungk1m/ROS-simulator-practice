# ROS-simulator-practice  
ROS practice using MORAI simulator  

## Overview  
The scenario files, sensor files, network configuration files, and weight files for the models used in the ROS packages are available on [Google Drive](https://drive.google.com/drive/folders/1yVIHXtS4b5llzoeRquM-bcdDc4H53_MF?usp=sharing).  
These packages primarily focus on **perception tasks** and are compatible with **ROS1 Noetic**.  

---

## Directory Structure 

ROS-simulator-practice/


├── get_GT/ &emsp;&emsp;  # Scripts for extracting GT data

├── pedestrian_detection/   &emsp;&emsp;  # ROS package folder for object detection

├── traffic_classification/ &emsp;&emsp;  # ROS package folder for image classification

---

## Directory Details  

### `get_GT`
This folder contains scripts for extracting ground truth (GT) data from the simulator:  
- **`capture_sensor_data.py`**: Leverages the sensor capture feature of the MORAI simulator.  
- **`save_traffic_img.py`**: Subscribes to traffic information topics, organizes traffic data into folders categorized by traffic classes, and saves the images for classification tasks.

---

### `pedestrian_detection`
This is a ROS package:  
- **Preparation**:  
  - Before running, place the `ped_weight.pt` file (download from Google Drive) into the `src` folder.  
- **Functionality**:  
  - The `bbox_publisher.py` script subscribes to image topics, processes them using a YOLO model, and publishes real-time bounding box information.

---

### `traffic_classification`
This is another ROS package:  
- **Preparation**:  
  - Before running, place the `traffic_weight.pt` file (download from Google Drive) into the `src` folder.  
- **Functionality**:  
  - The `traffic_publisher.py` script subscribes to image topics, processes them using the AlexNet model, and publishes real-time traffic classification information.

---

## How to Use ROS Packages  
Below is an example for using the `pedestrian_detection` package:  

1. Copy the package to your ROS workspace:  
   ```bash
   cp -r pedestrian_detection/ {your_ros_workspace}/src
   ```
2. Build the package:
    ```bash
    cd {your_ros_workspace} && catkin_make && source devel/setup.bash
    ```
3. Launch the package:
    ```bash
    roslaunch pedestrian_detection run.launch
    ```
The same workflow applies to the `traffic_classification` package.

---

