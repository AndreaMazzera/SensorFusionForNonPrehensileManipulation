//___________________REAL OBJECT POSE CALCULATOR______________________ 
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker.hpp>

#include <cmath>

using namespace std;
using namespace placeholders;

class RealObjPosePublisher : public rclcpp::Node
{
    public:
        RealObjPosePublisher() : Node("RealPoseObj_PubNode")
        {
            ee_pose.resize(6);
            obj_worldpose.resize(3);
            
            // _______________________________________________________________________
            // _________________________SUBSCRIBERS___________________________________ 
            
            ModelState_Subscriber = this->create_subscription
                <gazebo_msgs::msg::ModelStates>("/demo/model_states_demo", 10, 
                std::bind(&RealObjPosePublisher::modelStatesCallback, this, ::_1));
    
            ArmPose_Subscriber = this->create_subscription
                <std_msgs::msg::Float64MultiArray>("/arm_pose", 10,
                std::bind(&RealObjPosePublisher::EEPoseCallback, this, _1));

            //___________________________________________________________________________
            // ____________________________PUBLISHER_____________________________________ 

            RealPoseObject_Publisher = this->create_publisher
                <geometry_msgs::msg::Point>("/real_objpose", 1);
            
            RealPoseMarkerPublisher = this->create_publisher
                <visualization_msgs::msg::Marker>("/RealObjPose_Frame", 1);
            //___________________________________________________________________________
            //___________________________________________________________________________

            timer = this->create_wall_timer(
                std::chrono::milliseconds(20),
                std::bind(&RealObjPosePublisher::publish_transform, this));
        }

    private:
        vector<double> ee_pose;
        vector<double> obj_worldpose;

        rclcpp::TimerBase::SharedPtr timer;

        rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr ModelState_Subscriber;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ArmPose_Subscriber;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr RealPoseObject_Publisher;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr RealPoseMarkerPublisher;

        void EEPoseCallback (const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            for (size_t i = 0; i < msg->data.size(); ++i) 
                ee_pose[i] = msg->data[i];
        }

        int getIndex(std::vector<std::string> v, std::string value)
        {
            for(int i = 0; i < v.size(); i++)
            {
                if(v[i].compare(value) == 0)
                    return i;
            }
            return -1;
        }

        void modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
        {
            // Find the index of the CUBOID model
            int index = getIndex(msg->name, "CUBOID");
   
            // Extract the position (x, y)
            obj_worldpose[0] = msg->pose[index].position.x;  // x position
            obj_worldpose[1] = msg->pose[index].position.y;  // y position

            // Extract orientation (quaternion) and convert to yaw
            tf2::Quaternion q(
                msg->pose[index].orientation.x,
                msg->pose[index].orientation.y,
                msg->pose[index].orientation.z,
                msg->pose[index].orientation.w);

            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);  // Get yaw angle (rotation around z-axis)

            obj_worldpose[2] = yaw;  // Store the yaw angle

            // Publish transformed pose as visualization_msgs::msg::Marker (optional)
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = this->get_clock()->now();
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = msg->pose[index].position;
            marker.pose.orientation = msg->pose[index].orientation;    
            marker.scale.x = 0.05;  // Adjust size as needed
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.a = 1.0;  // Alpha
            marker.color.r = 0.0;  // Red
            marker.color.g = 1.0;  // Green
            marker.color.b = 0.0;  // Blue
            RealPoseMarkerPublisher->publish(marker);
        }

        void publish_transform ()
        {
            double x_tray       = ee_pose[0];
            double y_tray       = ee_pose[1];
            double z_tray       = ee_pose[2];
            double roll_tray    = ee_pose[3];
            double pitch_tray   = ee_pose[4];
            double yaw_tray     = ee_pose[5];
            
            // Get pose of cuboid
            double x_cuboid = obj_worldpose[0];
            double y_cuboid = obj_worldpose[1];
            double yaw_cuboid_rad = obj_worldpose[2] - yaw_tray; 

            // Convert yaw to degrees
            double yaw_cuboid_deg = yaw_cuboid_rad * (180.0 / M_PI);

            //if (yaw_cuboid_deg > 180)
            //     yaw_cuboid_deg -= 360.0;
            //else if (yaw_cuboid_deg < -180)
            //     yaw_cuboid_deg += 360.0;

            // Compute pose of object respect tray (not rotated)
            double x_cuboid_tray = x_cuboid - x_tray;
            double y_cuboid_tray = y_cuboid - y_tray;

            // Compute pose of object considering rotation of tray
            double x_cuboid_tray_rotated = x_cuboid_tray * cos(-yaw_tray) - y_cuboid_tray * sin(-yaw_tray);
            double y_cuboid_tray_rotated = x_cuboid_tray * sin(-yaw_tray) + y_cuboid_tray * cos(-yaw_tray);

            // Publish transformed pose
            geometry_msgs::msg::Point real_pose_tray_object_msg;
            real_pose_tray_object_msg.x = x_cuboid_tray_rotated;
            real_pose_tray_object_msg.y = y_cuboid_tray_rotated;
            real_pose_tray_object_msg.z = yaw_cuboid_deg ;
            RealPoseObject_Publisher->publish(real_pose_tray_object_msg);
        }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealObjPosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
