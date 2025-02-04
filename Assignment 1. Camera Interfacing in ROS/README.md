# 1st Assignment: Camera Interfacing in ROS2

**Author:** [Artem Bisliouk](https://github.com/abisliouk)  
**Course:** EEL 5934 - Autonomous Robots (Spring 2025)  
**University:** University of Florida  

---

## 🖥️ System Information
- **Hardware:** MacBook M1
- **Operating System:** macOS 14.6.1 (23G93)
- **Virtual Machine:** Parallels Desktop 18
- **Guest OS:** Ubuntu 22.04 (Jammy)
- **ROS Version:** ROS 2 Humble

---

## 📌 Overview
In this assignment, we developed a ROS 2 package (`image_processor`) for **camera interfacing in ROS**, including:
- **Capturing images from a USB camera** (`usb_cam` package).
- **Converting ROS images to OpenCV format** (`cv_bridge`).
- **Detecting faces and drawing bounding boxes** (`OpenCV` Haar cascade).
- **Publishing the processed images as `/out/image`**.
- **Creating a launch file** to start all nodes.

## 📂 Files & Folders
- `image_processor/` - ROS 2 package with face detection.
- `launch/face_detection_launch.py` - Launch file for running all nodes.
- `task description.pdf` - Assignment description.
- `demo_screenshots/` - Screenshots of execution results.
- `report.pdf` - Report describing methodology.

---

## 🛠 Setup Instructions
#### **1. Clone the Repository**
```bash
git clone https://github.com/abisliouk/EEL5934-autonomous-robots.git
cd EEL5934-autonomous-robots/"Assignment 1. Camera Interfacing in ROS"
```

#### **2. Install Dependencies**
```bash
sudo apt update
sudo apt install ros-humble-usb-cam ros-humble-cv-bridge python3-opencv
```

#### **3. Build the Package**
```bash
colcon build --packages-select image_processor
source install/setup.bash
```

---

## 🚀 Running the Program

### **1️⃣ Running Everything via Launch File (Recommended)**
The easiest way to run everything is to use the provided launch file, which: 
- ✅ Starts the USB camera node (`usb_cam`)
- ✅ Starts the face detection node (`face_detection`)
- ✅ Opens `rqt_image_view` for visualization

```bash
ros2 launch image_processor face_detection_launch.py
```

### **2️⃣ Running Step by Step (Manually via ROS 2 Workspace)**
If you prefer to start each node separately, follow these __manual steps__:

#### **Step 1: Start the USB Camera**
```bash
ros2 launch usb_cam camera.launch.py
```
This will start the USB camera node, which will publish images to the topic `/camera1/image_raw`

#### **Step 2: Verify the Camera Output**
To check if the camera is publishing images, run:
```bash
ros2 topic list
```
Expected output:
```bash
/camera1/image_raw
/parameter_events
/rosout
```
Then, echo the raw image topic to confirm it is active:
```bash
ros2 topic echo /camera1/image_raw
```
If you see __data appearing__, the camera is working.


#### **Step 3: Start the Face Detection Node**
In a new terminal, navigate to the __ROS 2 workspace__(in this case it's `ros2_ws` folder):
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run image_processor face_detection
```

#### **Step 4: View the Processed Image**
In another terminal, run:
```bash
ros2 run rqt_image_view rqt_image_view
```
- Select `/camera1/image_raw` to verify the original image.
- Select `/out/image` to check the processed image with face detection.


### **📜 References**
- [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- [ROS 2 Tutorial](https://www.youtube.com/playlist?list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy)
- [cv_bridge Tutorial](https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython)