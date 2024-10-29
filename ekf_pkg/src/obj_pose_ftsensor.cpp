//______________Object pose calculator with ft sensor______________________ 
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

using namespace std;
using namespace placeholders;

class FTSensorObjPose : public rclcpp::Node
{
    public:
        FTSensorObjPose() : Node("FTSensor_PoseObj_PubNode")
        {
            tray_pose.resize(6);

            // Subscriber topic of ft_sensor
            Wrench_Subscriber = this->create_subscription
                <geometry_msgs::msg::WrenchStamped>("/wrench", 1, 
                std::bind(&FTSensorObjPose::WrenchCallback, this, _1));

            // Subscribe to get pose of manipulator's end-effector
            ArmPose_Subscriber = this->create_subscription
                <std_msgs::msg::Float64MultiArray>("/arm_pose", 10,
                std::bind(&FTSensorObjPose::ArmPoseCallback, this, _1));

            // Publisher pose of object respect tray
            MassObj_Publisher = this->create_publisher
                <std_msgs::msg::Float64>("/mass_object", 1);

            // Publisher pose of object respect tray
            FTSensor_ObjPose_Publisher = this->create_publisher
                <geometry_msgs::msg::Point>("/ftsensor_objpose", 1);

            // De-comment if you want use a parameter
            // this->declare_parameter<double>("m_obj", 1.0);
        }

    private:
        vector<double> tray_pose;

        // De-comment if you want use a parameter
        // rclcpp::Parameter mass_object_param;

        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr Wrench_Subscriber;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ArmPose_Subscriber;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr MassObj_Publisher;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr FTSensor_ObjPose_Publisher;

        void ArmPoseCallback (const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            if (msg->data.size() >= 6) 
            { 
                for (size_t i = 0; i < msg->data.size(); ++i) 
                    tray_pose[i] = msg->data[i];
            }
            else
                RCLCPP_WARN(get_logger(), "Received invalid data");
        }

        void WrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
        {
            // Parameters
            const double mass_tray = 1.0;
            const double mass_camera = 0.026;
            const double mass_tray_camera = mass_tray + mass_camera; 
            const double g = 9.81;        

            // Get force along z
            double fz = msg->wrench.force.z;
            
            // Get orientation of tray
            double roll = tray_pose[3];
            double pitch = tray_pose[4];
            double yaw = tray_pose[5];

            // Calculate mass of object = total_mass - tray_camera_mass                          
            double mass_object = -fz / (g*cos(roll) * cos(pitch)) - mass_tray_camera;
            
            // if the mass is not very small then the object is on the tray
            if (mass_object > 1e-6) 
            {     
                // Publish the value of estimated mass
                auto message = std_msgs::msg::Float64();               
                message.data = mass_object;
                MassObj_Publisher->publish(message);
                
                // De-comment if you want use a parameter
                // this->set_parameter(rclcpp::Parameter("m_obj", mass_object));

                // Calculate force components produced by mass of camera 
                // Rrp = Rroll * Rpitch
                // F = Rrp * [0; 0; -mass_camera * g]
                double F_x = -mass_camera * g * sin(pitch);
                double F_y = mass_camera * g * sin(roll) * cos(pitch);
                double F_z = -mass_camera * g * cos(roll) * cos(pitch);

                // Arms of forces, which correspond position of camera
                double d_x = 0.15; 
                double d_y = 0.0; 
                double d_z = 0.05;

                // Calculate torques produced by camera
                // Cross product: F x d = [Fx;Fy;Fz] x [dx;dy;dz] 
                double tau_x_camera = -F_y * d_z - F_z * d_y;
                double tau_y_camera = F_x * d_z - F_z * d_x;
                double tau_z_camera = -F_x * d_y - F_y * d_x;

                // Get torques around x and y be ft_sensor
                double tau_x = msg->wrench.torque.x;
                double tau_y = msg->wrench.torque.y;

                // Compute torque produced by object only
                double tau_x_real = tau_x - tau_x_camera;
                double tau_y_real = tau_y - tau_y_camera;
                    
                // Compute position of object
                double x = tau_y_real / (mass_object * g);
                double y = tau_x_real / (-mass_object * g);

                // Publish object pose
                auto pose_msg = std::make_unique<geometry_msgs::msg::Point>();
                pose_msg->x = x;
                pose_msg->y = y;
                pose_msg->z = 0.0; 
                FTSensor_ObjPose_Publisher->publish(std::move(pose_msg));
            } 
            else 
            {
                // Publish a very small value to understand the absence of object
                auto message = std_msgs::msg::Float64();               
                message.data = 0.0001;
                MassObj_Publisher->publish(message);

                // Object is fallen and its coordinate are equal to Nan values
                auto pose_msg = std::make_unique<geometry_msgs::msg::Point>();
                pose_msg->x = std::numeric_limits<double>::quiet_NaN(); 
                pose_msg->y = std::numeric_limits<double>::quiet_NaN();
                pose_msg->z = std::numeric_limits<double>::quiet_NaN();
                FTSensor_ObjPose_Publisher->publish(std::move(pose_msg));
            }
        }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FTSensorObjPose>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
