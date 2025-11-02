// Include the main ROS2 C++ API header
#include "rclcpp/rclcpp.hpp"

// Include message type for images (used to publish camera frames)
#include "sensor_msgs/msg/image.hpp"

// Include OpenCV–ROS bridge to convert between cv::Mat and ROS Image messages
#include "cv_bridge/cv_bridge.h"

// Include OpenCV for video capture and image processing
#include <opencv2/opencv.hpp>

// Define a class that inherits from rclcpp::Node
// Each ROS2 node runs independently and handles its own publishers/subscribers
class WebcamPublisher : public rclcpp::Node
{
public:
    // Constructor — initializes the node when the object is created
    WebcamPublisher()
        : Node("webcam_publisher_cpp")  // The node name shown in ROS2
    {
        // Create a publisher that publishes messages of type sensor_msgs::msg::Image
        // Topic name: "webcam_image", Queue size: 10
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("webcam_image", 10);

        // Open the default webcam device (index 0)
        // Use the Video4Linux2 backend (CAP_V4L2) for Linux
        cap_.open(0, cv::CAP_V4L2);

        // Check if webcam opened successfully
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open webcam");
            return;
        }

        // Set camera resolution (width × height)
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 400);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 400);

        // Create a timer that triggers the callback periodically
        // Here, it runs every 33 milliseconds (~30 frames per second)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&WebcamPublisher::timer_callback, this));

        // Log message to indicate successful node startup
        RCLCPP_INFO(this->get_logger(), "Webcam Publisher Started");
    }

private:
    // Timer callback function — called every 33ms to capture and publish a frame
    void timer_callback()
    {
        cv::Mat frame;     // Create an OpenCV matrix to hold the image
        cap_ >> frame;     // Capture one frame from the webcam

        // If the frame is empty (e.g., no camera or read error), skip publishing
        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Empty frame capture");
            return;
        }

        // Convert the OpenCV image (cv::Mat) to a ROS2 Image message
        // "bgr8" is the standard color format for OpenCV images
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        // Publish the converted ROS Image message to the "webcam_image" topic
        publisher_->publish(*msg);

        // Optional: Log each published frame
        RCLCPP_INFO(this->get_logger(), "Published a frame");
    }

    // Declare a publisher handle for sending Image messages
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    // Declare a timer handle — used to call timer_callback() periodically
    rclcpp::TimerBase::SharedPtr timer_;

    // Declare an OpenCV VideoCapture object to access webcam frames
    cv::VideoCapture cap_;
};

// Main function — the program starts executing here
int main(int argc, char *argv[])
{
    // Initialize the ROS2 client library
    rclcpp::init(argc, argv);

    // Create an instance of our WebcamPublisher node
    auto node = std::make_shared<WebcamPublisher>();

    // Spin the node — keeps it alive and processing callbacks (e.g., timers, messages)
    rclcpp::spin(node);

    // Cleanly shut down the ROS2 client library
    rclcpp::shutdown();

    // Return success
    return 0;
}
