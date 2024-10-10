//________________________OBJECT RESPAWNER________________________
// This script in case of a crash or incorrect launch of the robot + object 
// launcher deletes and respawns the model of the latter in the correct position. 
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gazebo_msgs/msg/model_states.hpp>

#include <iostream>
#include <cmath>

using namespace std;
using namespace placeholders;

class ObjectRespawner : public rclcpp::Node 
{
    public:
        ObjectRespawner() : Node("ObjRespawner_Node") 
        {
            ee_pose.resize(6);
            ee_vel.resize(6);
            // _______________________________________________________________________
            // _________________________SUBSCRIBERS___________________________________      
            // Subscriber to the manipulator's end-effector pose topic
            ArmPose_Subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/arm_pose", 10, std::bind(&ObjectRespawner::ArmPoseCallback, this, _1));
            
            // Subscriber to the manipulator's end-effector velocity topic
            EEVel_Subscriber = this->create_subscription
                <geometry_msgs::msg::Twist>("/ee_vel", 1, 
                std::bind(&ObjectRespawner::eevelCallback, this, _1));
            
            // Subscribe to the /model_states topic
            ModelState_Subscriber = this->create_subscription
                <gazebo_msgs::msg::ModelStates>("/demo/model_states_demo", 10, 
                std::bind(&ObjectRespawner::modelStatesCallback, this, ::_1));
            // _______________________________________________________________________
        }

    private:
        vector<double> ee_pose;
        vector<double> ee_vel;

        // Subscribers
        rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr ModelState_Subscriber;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ArmPose_Subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr EEVel_Subscriber;

        // Function to obtain end-effector pose
        void ArmPoseCallback (const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            if (msg->data.size() >= 6) 
            { 
                for (size_t i = 0; i < msg->data.size(); ++i) 
                    ee_pose[i] = msg->data[i];
            }
            else
                RCLCPP_WARN(get_logger(), "Received invalid data");
        }

        // Function to obtain end-effector velocity
        void eevelCallback (const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            ee_vel[0] = msg->linear.x;
            ee_vel[1] = msg->linear.y;
            ee_vel[2] = msg->linear.z;
            ee_vel[3] = msg->angular.x;
            ee_vel[4] = msg->angular.x;
            ee_vel[5] = msg->angular.x;            
        }

        // Function to select correct model in list Gazebo Models
        int getIndex(std::vector<std::string> v, std::string value)
        {
            for(int i = 0; i < v.size(); i++)
            {
                if(v[i].compare(value) == 0)
                    return i;
            }
            return -1;
        }
        
        // Function to respawn object
        void modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
        {
            int index = getIndex(msg->name, "CUBOID");

            double z = msg->pose[index].position.z;

            // If these three conditions are respect realize the object respawn
            // A: the height of the object from the ground (z) is less than a certain threshold 
            // B: the tray is not very inclined
            // C: the tray is stop

            if (z < 0.5 && abs(ee_pose[3]) < 0.2 && abs(ee_pose[4]) < 0.2 && abs(ee_vel[0]) < 0.01 && abs(ee_vel[1]) < 0.01) 
            {
                RCLCPP_INFO(this->get_logger(), "Object is fall on the ground.");

                //cout << "Press Enter to respawn the object..." << endl;
                //cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

                std::string delete_command = "gz model --delete -m CUBOID";
                int delete_result = system(delete_command.c_str());

                if (delete_result != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to delete object.");
                    return;
                }
                
                std::string spawn_command = "gz model --spawn-file=/home/andrea/catkin_ws/src/iiwa_description/models/cuboid_object/model.sdf ";
                spawn_command += "--model-name=CUBOID";
                spawn_command += " -x " + std::to_string(ee_pose[0]-0.04);
                spawn_command += " -y " + std::to_string(ee_pose[1]);
                spawn_command += " -z " + std::to_string(ee_pose[2]+0.04);
                spawn_command += " -R " + std::to_string(ee_pose[3]);
                spawn_command += " -P " + std::to_string(1.57);
                spawn_command += " -Y " + std::to_string(ee_pose[5]);

                int spawn_result = system(spawn_command.c_str());
                if (spawn_result != 0) 
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to spawn object.\n");
                    return;
                }
                else
                    RCLCPP_INFO(this->get_logger(), "Object respawned!");
            }
        }
};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectRespawner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
