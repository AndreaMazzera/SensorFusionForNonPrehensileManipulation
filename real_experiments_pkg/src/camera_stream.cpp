#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraStream : public rclcpp::Node
{
    public:
        CameraStream()
        : Node("camera_stream")
        {
            // Hardcoded paths for video and camera info file
            video_path = "src/data/Experiments/right50/right50.mp4";

            // Set up publishers
            image_publisher_ = this->create_publisher
                <sensor_msgs::msg::Image>("/camera/image_raw", 10);
            
            camera_info_publisher_ = this->create_publisher
                <sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10);

            // Load camera info
            loadCameraInfo();

            // Open video capture
            cap_.open(video_path);
            if (!cap_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open video: %s", video_path.c_str());
                rclcpp::shutdown();
            }

            // Get video properties
            fps_ = cap_.get(cv::CAP_PROP_FPS);
            if (fps_ <= 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to get FPS from video.");
                rclcpp::shutdown();
            }

            // Create timer to publish frames
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(33), 
                std::bind(&CameraStream::timerCallback, this)
            );
        }

    private:
        void loadCameraInfo()
        {
            // Directly set camera info values
            camera_info_msg_.width = 720;
            camera_info_msg_.height = 1280;
            camera_info_msg_.distortion_model = "plumb_bob";

            camera_info_msg_.k = {992.908586, 0.0, 360.814135, 0.0, 989.028867, 642.846931, 0.0, 0.0, 1.0};
            camera_info_msg_.d = {0.079306, -0.231030};
            camera_info_msg_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
            camera_info_msg_.p = {992.908586, 0.0, 360.814135, 0.0, 0.0, 989.028867, 642.846931, 0.0, 0.0, 0.0, 1.0, 0.0};

            camera_info_msg_.roi.x_offset = 0;
            camera_info_msg_.roi.y_offset = 0;
            camera_info_msg_.roi.height = 0;
            camera_info_msg_.roi.width = 0;
            camera_info_msg_.roi.do_rectify = 0;
        }

        void timerCallback()
        {
            cv::Mat frame;
            cap_ >> frame;
            if (frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "Video stream ended or cannot read frame.");
                rclcpp::shutdown();
                return;
            }

            // Calculate current frame time in seconds
            double frame_time = cap_.get(cv::CAP_PROP_POS_MSEC) / 1000.0;

            // Log the timestamp to the terminal
            RCLCPP_INFO(this->get_logger(), "Current Video Time: %.2f s", frame_time);

            // Convert OpenCV image to ROS Image message
            std_msgs::msg::Header header;
            header.stamp = this->get_clock()->now();
            sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

            // Update and publish camera info
            camera_info_msg_.header.stamp = img_msg->header.stamp;

            image_publisher_->publish(*img_msg);
            camera_info_publisher_->publish(camera_info_msg_);
        }

        std::string video_path;
        cv::VideoCapture cap_;
        double fps_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
        sensor_msgs::msg::CameraInfo camera_info_msg_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraStream>());
    rclcpp::shutdown();
    return 0;
}
