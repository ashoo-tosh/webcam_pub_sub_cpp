🖥️ ROS 2 Webcam Publisher & Subscriber (C++)

This project demonstrates a simple publisher–subscriber model in ROS 2 Humble (C++) using OpenCV and cv_bridge.
The publisher node captures frames from your webcam and publishes them as sensor_msgs/Image messages.
The subscriber node receives and displays those frames in real-time.

Project Structure
ros2_cpp_assignment/
├── src/
│   └── webcam_cpp/
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── src/
│           ├── webcam_publisher.cpp
│           └── webcam_subscriber.cpp

Dependencies

Make sure you have the following installed:

ROS 2 Humble

OpenCV

cv_bridge

image_transport

sensor_msgs

You can install them using:

sudo apt install ros-humble-cv-bridge ros-humble-image-transport ros-humble-sensor-msgs

⚙️ Build Instructions

Navigate to your workspace:

cd ~/ros2_cpp_assignment


Build the package:

colcon build --packages-select webcam_cpp


Source the setup file:

source install/setup.bash

Running the Nodes

Run the Publisher:

ros2 run webcam_cpp webcam_publisher


This node:

Opens the system webcam (/dev/video0)

Captures frames using OpenCV

Publishes them on the /webcam_image topic

Run the Subscriber (in a new terminal):

source install/setup.bash
ros2 run webcam_cpp webcam_subscriber


This node:

Subscribes to /webcam_image

Displays the received frames in an OpenCV window

How It Works

webcam_publisher.cpp

Initializes a ROS 2 node called "webcam_publisher_cpp".

Captures frames using OpenCV.

Converts frames to ROS sensor_msgs::msg::Image messages using cv_bridge.

Publishes the frames at ~30 FPS.

webcam_subscriber.cpp

Initializes a ROS 2 node called "webcam_subscriber_cpp".

Subscribes to the same topic (/webcam_image).

Converts received ROS image messages back to OpenCV cv::Mat.

Displays them using cv::imshow.

Example Output


When you run both nodes, you’ll see:

In the publisher terminal:

[INFO] [webcam_publisher_cpp]: Webcam Publisher Started
[INFO] [webcam_publisher_cpp]: Published a frame


In the subscriber terminal:

[INFO] [webcam_subscriber_cpp]: Webcam Subscriber Started


and a live webcam feed window will appear.
<img width="1920" height="1080" alt="Screenshot from 2025-11-02 20-55-34" src="https://github.com/user-attachments/assets/72bb9681-df0a-4777-95c8-b49308045e2f" />

Author

Ashootosh Raja
GitHub: ashoo-tosh
