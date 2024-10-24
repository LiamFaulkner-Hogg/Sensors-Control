#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>

class GoProPublisher : public rclcpp::Node
{
public:
    GoProPublisher() : Node("gopro_publisher")
    {
        // Create an Image publisher
        image_publisher_ = image_transport::create_publisher(this, "/camera/image_raw");

        camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "gopro");
        camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", 10);

        publish_camera_info();

        // Create the SetCameraInfo service
        set_camera_info_service_ = this->create_service<sensor_msgs::srv::SetCameraInfo>(
            "camera/set_camera_info",
            std::bind(&GoProPublisher::set_camera_info_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Open the GoPro stream using OpenCV
        video_capture_.open("https://video-weaver.ams03.hls.ttvnw.net/v1/playlist/CskEVf9AHYahQy_k4Z0X-p1eI6S4MQPYiMgyJBCFhx1pxK1NNURGcuu9Xv-6WSXtCtKXWM_5RPYFk27iZFrrKFFq1cT3LJ6eRCio3g4S-lJv7aUOHDv9rfRhea7K7FBoRZSOZGrdC6ufzuy7PZ4BzrOKAxwgSp-3c5Wsm1224OECY2DESTYQX0OR0PRU9s6ycBtb37L8d19N7yhqbAcIp8DblYe7CIbjjagD4uJpLA1_FkFUJxBxELB8zCJaV3BbV-5eVNXE1NlU0Gz5e725BQarazTSkxlVyq-_uhFSR2ev4lBIHeG-jYHt3uiog1vKvqgu_RPv4O_GDfdjuKTiCFuaiK8UJ-m6v7Fu4djSdy4J9xeiK7x9syNsouGGa5kb_tMuNraTEUTzTndsULtCJkuQSKpvyL7CNfGpl679jfowLm99L4DjnXQ1RWEBplgGqMi6DnKVCZT8p0U5D6rYF7j41AU0vqMVH_fdHbf_sLW6QdM9Dk8wZ98yYcJERCAvV7H2cpRci46iINFDazV5ntIqGPNX-w9lxVd_TU7eZPvmJUnVElo3f2FzH2TwIYIGqk_pIstPyu5p6dd6ZKBIgJakrlI-fW3Lh4AuieRVrL4aUsovS_9tXeT4FFQkmCMqpLVn9rKvIErX-at1MlNLl3yq48kNrSnH_lBqoM05927swbHZGFDGZ-CLFfdqdXpgr-q4d59Q3-Y3NNxwgJ4zqOIJaa8d9vDoFDrni72ouv6EB6ieUsASlAy6vhXx90rguY9FYQlhH_Ig_NVrGgxbzJHiPiSQKFUccdUgASoJdXMtd2VzdC0yMNgK.m3u8");

        if (!video_capture_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open GoPro stream");
            return;
        }

        // Set timer to capture and publish frames
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1),
                                         std::bind(&GoProPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        cv::Mat frame;

        // Capture frame-by-frame from the GoPro stream
        if (video_capture_.read(frame))
        {
            // Convert the OpenCV frame (Mat) to ROS Image message
            std_msgs::msg::Header header;
            header.stamp = this->get_clock()->now();
            header.frame_id = "gopro_frame";

            // Convert OpenCV image (Mat) to ROS message
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

            // Publish the message
            image_publisher_.publish(*msg);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to capture frame from GoPro");
        }
    }

    void publish_camera_info()
    {
        auto camera_info_msg = camera_info_manager_->getCameraInfo();
        camera_info_publisher_->publish(camera_info_msg);
        RCLCPP_INFO(this->get_logger(), "Published camera info");
    }

    void set_camera_info_callback(
        const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> request,
        std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "SetCameraInfo called");
        response->success = true; // Indicate success
    }

    // ROS 2 node publisher for images
    image_transport::Publisher image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_camera_info_service_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    cv::VideoCapture video_capture_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoProPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// #include <rclcpp_components/register_node_macro.hpp>
// RCLCPP_COMPONENTS_REGISTER_NODE(GoProPublisher)