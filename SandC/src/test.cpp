// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.hpp>
// #include <opencv2/opencv.hpp>

// #include <camera_info_manager/camera_info_manager.hpp>
// #include <sensor_msgs/msg/camera_info.hpp>

// class GoProPublisher : public rclcpp::Node
// {
// public:
//     GoProPublisher() : Node("gopro_publisher")
//     {
//         // Create an Image publisher
//         image_publisher_ = image_transport::create_publisher(this, "/camera/image_raw");

//         camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "gopro");
//         camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", 10);

//         publish_camera_info();

//         // Open the GoPro stream using OpenCV
//         video_capture_.open("https://video-weaver.tyo03.hls.ttvnw.net/v1/playlist/CtwE-hCFxZefyUvpda7aMhyzcL8Xc9-MYEP_JbfN6CoOXDbGaNVS8woznuRU6GAtCwV04ytMcwYHYup2jvrf9Rq9FXy1M_9u0BUCgDSlr6SelVl9tDe2vsIJ1PEUYRP5TBnjtM8k4cRuraB4FcNJHh9OfVRHnBFlPdIzMWpqKRv0UJh7uIMEfqxSwGDuFoeGDQ9wR5amqyYDvi9EdY68J4Xr4EacRdH4TUHcFGHR4sFmTfmUcNC0RhYfmgxbCN97Ozf7_QugqtRdJk2jR6JMP-iqdHchQ9Jb59g0FesSX4pJek1jotX8-lwDXzBQP-DOvyYtSBjlBKJyv3XDSa9NxvK0jKWYt2gtr-JAJlsKwMPXcBntuHEL3ZjVqnzdtVN4NwP17_Y2OpiMukSgVTgDRBotx-_taYaa5ZqAsye4hs6Ob4VOWKZNCK6lH1nZ6YNgt6OE8CQHzTsmOTd5dzylqFVcXynro6amuVPuihCaB5-9dUadP7edt2A8CBsRAAES3y5peNH9JaLrBmtf0xarWC3742yICu-oNJs5tZnZBDf5Ldn9kiUiz2FMGhlLrZQ4JZqqpbJd64YJVuRjVFjV905tn4BxWfsD7jRkz5L2B9vtdK8DMmxez_ngI-U_1LKlG8TgtgAfg2O9bt6IRc-F9h7ExnxqlHZELepoZTdyuM3Gbb3HwnEVwTF0G_BRdNuW6NkzIaBmNfwsJ6RCVKnMIAtKuaEK0ISKZtCmveuXZGC4dWJUK7T8xhfxWN9gyZzIYG4xwXvO3Zmk89EOJ5lL0w9HxLergWjaBIo1toWUvRoMPZ7ENnc29uJnKTboIAEqCXVzLXdlc3QtMjDWCg.m3u8");

//         if (!video_capture_.isOpened())
//         {
//             RCLCPP_ERROR(this->get_logger(), "Failed to open GoPro stream");
//             return;
//         }

//         // Set timer to capture and publish frames
//         timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
//                                          std::bind(&GoProPublisher::timer_callback, this));
//     }

// private:
//     void timer_callback()
//     {
//         cv::Mat frame;

//         // Capture frame-by-frame from the GoPro stream
//         if (video_capture_.read(frame))
//         {
//             // Convert the OpenCV frame (Mat) to ROS Image message
//             std_msgs::msg::Header header;
//             header.stamp = this->get_clock()->now();
//             header.frame_id = "gopro_frame";

//             // Convert OpenCV image (Mat) to ROS message
//             sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

//             // Publish the message
//             image_publisher_.publish(*msg);
//         }
//         else
//         {
//             RCLCPP_WARN(this->get_logger(), "Failed to capture frame from GoPro");
//         }
//     }

//     void publish_camera_info()
//     {
//         auto camera_info_msg = camera_info_manager_->getCameraInfo();
//         camera_info_publisher_->publish(camera_info_msg);
//         RCLCPP_INFO(this->get_logger(), "Published camera info");
//     }

//     // ROS 2 node publisher for images
//     image_transport::Publisher image_publisher_;

//     rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;

//     std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

//     // OpenCV VideoCapture object to read the GoPro stream
//     cv::VideoCapture video_capture_;

//     // Timer for periodically capturing and publishing frames
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<GoProPublisher>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
