//_____________________Object pose errors calculator______________________ 
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point.hpp>

using namespace std;
using namespace placeholders;

class ErrorCalculatorNode : public rclcpp::Node
{
    public:
        ErrorCalculatorNode() : Node("ObjPoseError_PubNode")
        {
            // Topic for object real pose
            real_pose_subscriber = this->create_subscription
                <geometry_msgs::msg::Point>("/real_objpose", 1,
                std::bind(&ErrorCalculatorNode::realPoseCallback, this, _1));

            // Topic for FTSensor object pose
            ftsensor_objpose_subscriber = this->create_subscription
                <geometry_msgs::msg::Point>("/ftsensor_objpose", 1,
                std::bind(&ErrorCalculatorNode::ftSensorPoseCallback, this, _1));

            // Topic for Camera object pose
            camera_objpose_subscriber = this->create_subscription
                <geometry_msgs::msg::Point>("/camera_objpose", 10,
                std::bind(&ErrorCalculatorNode::camPoseCallback, this, _1));
            
            // Topic for estimated object pose
            estimated_state_subscriber = this->create_subscription
            <geometry_msgs::msg::Point>("/estimated_objpose", 1, 
            std::bind(&ErrorCalculatorNode::estimatedStateCallback, this, _1));

            // Publish error along x and y
            error_publisher = this->create_publisher
                <geometry_msgs::msg::Point>("/obj_pose_error", 1);

            // Publish camera error
            camera_objposerror_publisher = this->create_publisher
                <geometry_msgs::msg::Point>("/camera_objpose_error", 1);

            // Publish FT sensor error
            ftsensor_objposerror_publisher = this->create_publisher
                <geometry_msgs::msg::Point>("/ftsensor_objpose_error", 1);
 
            // Call member function
            auto timercallback = [this]() -> void 
            {
                computeAndPublishErrors();
            };
            timer = this->create_wall_timer(std::chrono::milliseconds(1), timercallback);
        }

    private:
        rclcpp::TimerBase::SharedPtr timer;

        geometry_msgs::msg::Point real_pose;
        geometry_msgs::msg::Point ftsensor_objpose;
        geometry_msgs::msg::Point camera_objpose;
        geometry_msgs::msg::Point estimated_state;

        geometry_msgs::msg::Point pose_error;
        geometry_msgs::msg::Point cam_error;
        geometry_msgs::msg::Point ft_sensor_error;

        // Subscribers and Publisher
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr real_pose_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ftsensor_objpose_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr camera_objpose_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr estimated_state_subscriber;        

        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ftsensor_objposerror_publisher;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr camera_objposerror_publisher;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr error_publisher;

        // Function to save real object pose
        void realPoseCallback(const geometry_msgs::msg::Point::SharedPtr msg)
        {
            real_pose.x = msg->x;
            real_pose.y = msg->y;
            real_pose.z = msg->z;
        }

        // Function to save object pose computed by ft sensor
        void ftSensorPoseCallback(const geometry_msgs::msg::Point::SharedPtr msg)
        {
            ftsensor_objpose.x = msg->x;
            ftsensor_objpose.y = msg->y;
            ftsensor_objpose.z = msg->z;
        }

        // Function to save object pose computed by camera
        void camPoseCallback(const geometry_msgs::msg::Point::SharedPtr msg)
        {
            // Storage camera pose
            camera_objpose.x = msg->x;
            camera_objpose.y = msg->y;
            camera_objpose.z = msg->z;
        }

        // Function to save estimated object pose computed by EKF
        void estimatedStateCallback(const geometry_msgs::msg::Point::SharedPtr msg)
        {
            estimated_state.x = msg->x;
            estimated_state.y = msg->y;
            estimated_state.z = msg->z;
        }

        // Function to publish pose errors
        void computeAndPublishErrors()
        {
            // Compute error between real and estimated pose
            auto x_error = std::abs(real_pose.x - estimated_state.x);
            auto y_error = std::abs(real_pose.y - estimated_state.y);

            auto error_msg = geometry_msgs::msg::Point();
            error_msg.x = x_error;
            error_msg.y = y_error;
            error_msg.z = 0.0;
            error_publisher->publish(error_msg);

            // Compute error between real and camera pose
            auto cam_x_error = std::abs(real_pose.x - camera_objpose.x);
            auto cam_y_error = std::abs(real_pose.y - camera_objpose.y);
            auto cam_yaw_error = std::abs(real_pose.z - camera_objpose.z);

            auto cam_error_msg = geometry_msgs::msg::Point();
            cam_error_msg.x = cam_x_error;
            cam_error_msg.y = cam_y_error;
            cam_error_msg.z = cam_yaw_error;
            camera_objposerror_publisher->publish(cam_error_msg);

            // Compute error between real and FT sensor pose
            auto ft_x_error = std::abs(real_pose.x - ftsensor_objpose.x);
            auto ft_y_error = std::abs(real_pose.y - ftsensor_objpose.y);

            auto ft_sensor_error_msg = geometry_msgs::msg::Point();
            ft_sensor_error_msg.x = ft_x_error;
            ft_sensor_error_msg.y = ft_y_error;
            ft_sensor_error_msg.z = 0.0;
            ftsensor_objposerror_publisher->publish(ft_sensor_error_msg);
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ErrorCalculatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
