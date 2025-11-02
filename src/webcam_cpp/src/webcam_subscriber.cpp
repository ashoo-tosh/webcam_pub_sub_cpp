// Include the core ROS2 C++ client library
#include "rclcpp/rclcpp.hpp"

// Include the standard ROS message type for images
#include "sensor_msgs/msg/image.hpp"

// Include the bridge to convert between ROS Image messages and OpenCV images
#include "cv_bridge/cv_bridge.h"

// Include the OpenCV main header (for cv::Mat, imshow, etc.)
#include <opencv2/opencv.hpp>

// Define a class that represents the subscriber node
// It inherits from rclcpp::Node (the base class for all ROS2 nodes)
class WebcamSubscriber : public rclcpp::Node
{
public:
    // Constructor — runs when the node is created
    WebcamSubscriber()
        : Node("webcam_subscriber_cpp")  // Node name as seen in ROS2
    {
        // Create a subscription to the "webcam_image" topic
        // - Message type: sensor_msgs::msg::Image
        // - Queue size: 10
        // - Callback function: image_callback()
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "webcam_image",                              // Topic name
            10,                                           // Queue size
            std::bind(&WebcamSubscriber::image_callback,  // Callback function
                      this,                               // Pointer to this object
                      std::placeholders::_1));            // Placeholder for the message argument

        // Print a message to indicate that the subscriber has started successfully
        RCLCPP_INFO(this->get_logger(), "Webcam Subscriber Started");
    }

private:
    // Callback function called whenever a new message is received on "webcam_image"
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Convert the incoming ROS Image message to an OpenCV image (cv::Mat)
            // "bgr8" is the color encoding used in OpenCV
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            // Display the received image in a window named "Webcam Feed (C++)"
            cv::imshow("Webcam Feed (C++)", frame);

            // Process any GUI window events (required for imshow to refresh)
            // waitKey(1) means: wait 1 ms, then continue (non-blocking)
            cv::waitKey(1);
        }
        catch (cv_bridge::Exception &e)
        {
            // Catch any exceptions from cv_bridge (e.g., format conversion errors)
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    // Declare a ROS2 subscription handle to receive Image messages
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

// The main function — entry point of the node
int main(int argc, char *argv[])
{
    // Initialize the ROS2 system
    rclcpp::init(argc, argv);

    // Create a shared pointer to the WebcamSubscriber node and start it
    rclcpp::spin(std::make_shared<WebcamSubscriber>());

    // Cleanly shut down the ROS2 system once done
    rclcpp::shutdown();

    // Exit the program successfully
    return 0;
}
