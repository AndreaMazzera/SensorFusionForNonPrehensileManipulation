//______________Object pose calculator with camera______________________ 
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <cmath>

using namespace std;
using namespace placeholders;

class CameraObjPose : public rclcpp::Node 
{
    public:
        CameraObjPose() : Node("CamSensor_PoseObj_PubNode") 
        {    
            // Vector for position (x,y,z) of AprilTag
            tag_position.resize(3);

            // Vector for end-effector pose
            ee_pose.resize(6);

            // Initialize the correction factor at 10%
            percentage_correction_factor = 0.1;  

            // Subscribe to get pose of manipulator's end-effector
            ArmPose_Subscriber = this->create_subscription
                <std_msgs::msg::Float64MultiArray>("/arm_pose", 10,
                std::bind(&CameraObjPose::ArmPoseCallback, this, _1));

            // Subscriber to obtain AprilTag Pose
            AprilTag_Subscriber = this->create_subscription
                <apriltag_msgs::msg::AprilTagDetectionArray>("/apriltag_detections", 10,
                std::bind(&CameraObjPose::tagDetectionCallback, this, _1));           

            // Setup TF listener
            tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);    
            
            // Initialize publisher
            CamPoseObj_Publisher = this->create_publisher
                <geometry_msgs::msg::Point>("/camera_objpose", 1);

            // Timer to periodically check for the transform and publish the pose
            timer = this->create_wall_timer(
                std::chrono::milliseconds(20),
                std::bind(&CameraObjPose::publish_transform, this));
        }

    private:
        // AprilTag ID
        int tag_id;
        // Correction factor
        double percentage_correction_factor;
        vector<double> tag_position;
        vector<double> ee_pose;

        rclcpp::TimerBase::SharedPtr timer;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;

        // Subscribers and Publisher
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ArmPose_Subscriber;
        rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr AprilTag_Subscriber;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr CamPoseObj_Publisher;

        // Function to obtain pose (x,y,z,R,P,Y) of end-effector
        void ArmPoseCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            if (msg->data.size() >= 6) 
            { 
                for (size_t i = 0; i < msg->data.size(); ++i) 
                    ee_pose[i] = msg->data[i];
            }
            else
                RCLCPP_WARN(get_logger(), "Received invalid data");
        }

        // Function to obtain pose of AprilTag respect to camera frame
        void tagDetectionCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) 
        {
            if (!msg->detections.empty()) 
            {
                tag_id = msg->detections[0].id;
            } 
            else
            {
                tag_id = 0;
            }
        }

        // Function to publish object pose
        void publish_transform() 
        {
            geometry_msgs::msg::TransformStamped transformStamped;
            try {
                // Lookup the transform from tray_frame to tag_frame
                transformStamped = tf_buffer->lookupTransform("tray", "tag_frame",
                                                            tf2::TimePointZero);
                // Convert transform to a Point message
                geometry_msgs::msg::Point point_msg;
                if (tag_id != 0) 
                {
                    tf2::Quaternion q;
                    q.setRPY(ee_pose[3], ee_pose[4], ee_pose[5]);
                        
                    tf2::Matrix3x3 m(q);

                    // Transform the point from the tray's frame to the world frame
                    tf2::Vector3 p(
                        transformStamped.transform.translation.x,
                        transformStamped.transform.translation.y,
                        transformStamped.transform.translation.z
                    );

                    tf2::Quaternion q_obj(
                        transformStamped.transform.rotation.x,
                        transformStamped.transform.rotation.y,
                        transformStamped.transform.rotation.z,
                        transformStamped.transform.rotation.w
                    );
                    double roll, pitch, yaw;
                    tf2::Matrix3x3 m_obj(q_obj);
                    m_obj.getRPY(roll, pitch, yaw);

                    // if the object is too close to the camera, 
                    // which occurs when its x-coordinate with respect 
                    // to the center of the tray is positive, 
                    // a 10% correction of x must be applied. 
                    // The camera datasheet states that below 30 cm 
                    // (and above 3 m) the accuracy decreases.
                    double correction_factor;
                    if (p.x()>=0)
                        correction_factor = percentage_correction_factor * p.x();
                    else
                        correction_factor = 0.0;

                    // Apply the correction factor to the x component
                    point_msg.x = p.x() - correction_factor;
                    point_msg.y = p.y();
                    point_msg.z = yaw * (180/M_PI) + 3.0;
                } 
                else 
                {
                    // If tag_id is zero, set point_msg to NaN
                    point_msg.x = std::numeric_limits<double>::quiet_NaN();
                    point_msg.y = std::numeric_limits<double>::quiet_NaN();
                    point_msg.z = std::numeric_limits<double>::quiet_NaN();
                }
                // Publish the point
                CamPoseObj_Publisher->publish(point_msg);
            } 
            catch (tf2::TransformException &ex) 
            {
                RCLCPP_WARN(this->get_logger(), "Could not transform tray_frame to tag_frame: %s", ex.what());
            }
        }
};

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraObjPose>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
