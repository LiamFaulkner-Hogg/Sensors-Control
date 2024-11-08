#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include "std_msgs/msg/string.hpp"

#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>

class GoProPublisher : public rclcpp::Node
{
public:
    GoProPublisher() : Node("gopro_publisher"), scanner()
    {
        // Declare the stream_url parameter
        this->declare_parameter<std::string>("stream_url", "https://ase11-0.playlist.ttvnw.net/v1/playlist/Cs4E8UB5M09U82Gse8jCKWDSfODxcYzOawB9W03jpWDd8UAeP9vktI1KHqZ5KltAF6aYiNSaxrtu9zIS8Ykw1dGjgeKjygkV7hsK9uPQSKc1Z0-fFjC2spGm-APgFX3i0DsXNBnhG2t8griXWGoIQfnSA3HooLiLrrhXDgZYs1wLf_FTSagivxf761hRn-Runy936mjK7As0pB13wt9GMMoOkmgoIVS_SzbPuU4TE9OBaL4IJMZvZnLv_EaW8E-Gp-dFLTrnm5BIrFGTNOftMC4TFpSJ-QryQTBqWtN9b1ULaUOtqopKJCuAHCFA-m-f070Vud6mZYu9YtaEEssUVWPaJi1e-BuxhLlifLjKQAQVi6Tf_CCaCSzXtHhZsetpmHnvGnfhG5uOsxlSSJ3Ta77fFsFDVpYsg_K6m5vYBvyn_tVJSXBnHby0851Vo5Hk1NQaVLKZPt0FxG_1EheYgIdfOjSuY3IIjkAa47pBkMCD7-Ba7FNa-u5GJmvx9gCEdpIpVq_wp_r7iggg5tfUAZpBXphEt0ybIidJYgfjw3I7vyX8bSyupbnwVetdwO5mFZ0F5m-Pw--RA_4jG_GPgI9B9Jf8Mc4xk-SJwo1DJKYAoBljIyBePA06nkCMpljAJziiyMmo_apBahvGMKk4P7mRisp89_rba9EfkeHQtFPLcVRTFfVKBBJF8EgTiq-2aUEpU33kGMGyh017b2KkWChtpplOI-FtL3eKvcrhxzO0J9a3NQudQwKjQy8dKSe4tJzts96kUzqmWL9wlPnMZU4aDHTt7_ryoFA5KihcvyABKgl1cy13ZXN0LTIw5wo.m3u8");

        // Get the stream URL from parameter
        stream_url_ = this->get_parameter("stream_url").as_string();

        if (stream_url_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No stream URL provided. Please provide a URL using: --ros-args -p stream_url:=\"YOUR_URL\"");
            return;
        }

        // Create publishers
        image_publisher_ = image_transport::create_publisher(this, "/camera/image_raw");
        processed_image_publisher_ = image_transport::create_publisher(this, "/processed_image");
        qr_position_publisher_ = this->create_publisher<std_msgs::msg::String>("/qr_code_positions", 10);

        camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "gopro");
        camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", 10);

        publish_camera_info();

        // Configure ZBar scanner
        scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
        RCLCPP_INFO(this->get_logger(), "ZBar scanner configured");

        RCLCPP_INFO(this->get_logger(), "Attempting to open stream at: %s", stream_url_.c_str());

        // Open the stream using OpenCV
        video_capture_.open(stream_url_);

        if (!video_capture_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open stream");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully opened stream");

        // Set timer to capture and process frames
        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), // ~30 FPS
                                         std::bind(&GoProPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        if (!video_capture_.read(frame))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to capture frame from stream");
            return;
        }

        // Publish raw frame
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = "camera_frame";
        sensor_msgs::msg::Image::SharedPtr raw_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        image_publisher_.publish(*raw_msg);

        // Process frame for QR codes
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Convert OpenCV image to ZBar image
        zbar::Image zbar_image(frame.cols, frame.rows, "Y800", gray.data, frame.cols * frame.rows);

        // Scan the image for QR codes
        scanner.scan(zbar_image);

        // Create a string to hold all QR code positions
        std::stringstream positions_json;
        positions_json << "{";
        bool first_qr = true;

        // Process detected QR codes
        for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
             symbol != zbar_image.symbol_end();
             ++symbol)
        {

            if (symbol->get_type() == zbar::ZBAR_QRCODE)
            {
                std::string qr_data = symbol->get_data();

                // Get QR code corners
                std::vector<cv::Point> points;
                for (int i = 0; i < symbol->get_location_size(); i++)
                {
                    points.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
                }

                // Calculate centers and offsets
                cv::Moments moments = cv::moments(points);
                cv::Point2f qr_center(moments.m10 / moments.m00, moments.m01 / moments.m00);
                cv::Point2f frame_center(frame.cols / 2.0f, frame.rows / 2.0f);
                float offset_x = qr_center.x - frame_center.x;
                float offset_y = qr_center.y - frame_center.y;

                // Add to JSON string
                if (!first_qr)
                    positions_json << ",";
                positions_json << "\"" << qr_data << "\":{\"x\":" << offset_x << ",\"y\":" << offset_y << "}";
                first_qr = false;

                // Draw visualizations
                cv::polylines(frame, std::vector<std::vector<cv::Point>>{points}, true, cv::Scalar(0, 255, 0), 2);
                cv::circle(frame, qr_center, 5, cv::Scalar(0, 0, 255), -1);
                cv::circle(frame, frame_center, 5, cv::Scalar(255, 0, 0), -1);
                cv::line(frame, frame_center, qr_center, cv::Scalar(255, 0, 0), 2);

                // Add text overlays
                cv::putText(frame, "QR: " + qr_data, cv::Point(10, 30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

                std::stringstream offset_text;
                offset_text << "Offset: X=" << static_cast<int>(offset_x)
                            << " Y=" << static_cast<int>(offset_y);
                cv::putText(frame, offset_text.str(), cv::Point(10, 60),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

                RCLCPP_INFO(this->get_logger(),
                            "QR Code detected: %s, Offset from center: X=%f, Y=%f",
                            qr_data.c_str(), offset_x, offset_y);
            }
        }

        // Close JSON and publish positions
        positions_json << "}";
        auto position_msg = std_msgs::msg::String();
        position_msg.data = positions_json.str();
        qr_position_publisher_->publish(position_msg);

        // Publish processed frame
        sensor_msgs::msg::Image::SharedPtr processed_msg =
            cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        processed_image_publisher_.publish(*processed_msg);
    }

    void publish_camera_info()
    {
        auto camera_info_msg = camera_info_manager_->getCameraInfo();
        camera_info_publisher_->publish(camera_info_msg);
        RCLCPP_INFO(this->get_logger(), "Published camera info");
    }

    std::string stream_url_;
    cv::VideoCapture video_capture_;
    image_transport::Publisher image_publisher_;
    image_transport::Publisher processed_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_position_publisher_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    rclcpp::TimerBase::SharedPtr timer_;
    zbar::ImageScanner scanner;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoProPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}