#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <fstream>
#include <chrono>
#include <csignal>
#include <vector>

using namespace std;
using namespace placeholders;
using namespace chrono_literals;

class Monitor : public rclcpp::Node 
{
    public:
        Monitor() : Node("monitor"), start_time(this->now()) 
        {
            // ___________________________SUBSCRIBER FOR ARM__________________________

            // Manipulator's end-effector pose subscriber
            EEPose_Subscriber = this->create_subscription
                <std_msgs::msg::Float64MultiArray>("/arm_pose", 1,
                    std::bind(&Monitor::ArmPoseCallback, this, _1));

            // Joint positions subscriber
            JointStates_Subscriber = this->create_subscription
                <sensor_msgs::msg::JointState>("/joint_states", 1, 
                bind(&Monitor::jointStateCallback, this, _1));
            
            // Control torques subscriber
            ControlTorques_Subscriber = this->create_subscription
                <std_msgs::msg::Float64MultiArray>("/arm_controller/commands", 1,
                std::bind(&Monitor::controlTorquesCallback, this, _1));
            // _______________________________________________________________
            // ___________________SUBSCRIBERS FOR OBJECT_______________________

            // Object pose by camera
            camera_pose_subscriber = this->create_subscription
                <geometry_msgs::msg::Point>("/camera_objpose", 1, 
                std::bind(&Monitor::cameraPoseCallback, this, _1));
            
            camera_pose_error_subscriber = this->create_subscription
                <geometry_msgs::msg::Point>("/camera_objpose_error", 1, 
                std::bind(&Monitor::cameraPoseErrorCallback, this, _1));

            // Obect pose by force/torque sensor
            ftsensor_subscriber = this->create_subscription
                <geometry_msgs::msg::Point>("/ftsensor_objpose", 1, 
                std::bind(&Monitor::ftsensorCallback, this, _1));

            ftsensor_error_subscriber = this->create_subscription
                <geometry_msgs::msg::Point>("/ftsensor_objpose_error", 1, 
                std::bind(&Monitor::ftsensorErrorCallback, this, _1));

            // Object real pose subscriber
            real_pose_subscriber = this->create_subscription
                <geometry_msgs::msg::Point>("/real_objpose", 1, 
                std::bind(&Monitor::realPoseCallback, this, _1));

            // Topic for estimated object pose
            estimated_state_subscriber = this->create_subscription
                <geometry_msgs::msg::Point>("/estimated_objpose", 1, 
                std::bind(&Monitor::estimatedStateCallback, this, _1));

            // Object pose error subscriber
            error_subscriber = this->create_subscription
                <geometry_msgs::msg::Point>("/obj_pose_error", 1, 
                bind(&Monitor::ErrorCallback, this, _1));

            // _______________________________________________________________
            // ___________________CSV FILE FOR ARM____________________________

            // Arm pose
            csv_arm_pose.open("/home/" + std::string(getenv("USER")) 
                                    + "/noprehensileman_ws/src/data/csv_arm_pose.csv");
            
            csv_arm_pose << "Timestamp,x,y,z,R,P,Y\n";

            // Joint positions q
            csv_joint_positions.open("/home/" + std::string(getenv("USER")) 
                                            + "/noprehensileman_ws/src/data/joint_pos.csv");
            
            csv_joint_positions << "Timestamp,q1,q2,q3,q4,q5,q6,q7\n";

            // Control torques
            csv_control_torques.open("/home/" + std::string(getenv("USER")) 
                                            + "/noprehensileman_ws/src/data/control_torques.csv");
            
            csv_control_torques << "Timestamp,tau1,tau2,tau3,tau4,tau5,tau6,tau7\n";
            
            // ______________________________________________________________
            // ___________________CSV FILE FOR OBJECT____________________________
            
            // Estimation object pose error
            csv_obj_pose_error.open("/home/" + std::string(getenv("USER")) 
                                            + "/noprehensileman_ws/src/data/obj_pose_error.csv");
            csv_obj_pose_error << "Timestamp,X_Error,Y_Error\n";

            // Real pose object
            csv_real_obj_pose.open("/home/" + std::string(getenv("USER")) 
                                            + "/noprehensileman_ws/src/data/obj_pose_real.csv");
            csv_real_obj_pose << "Timestamp,X_obj,Y_obj\n";            

            // Estimated object pose
            csv_est_obj_pose.open("/home/" + std::string(getenv("USER")) 
                                            + "/noprehensileman_ws/src/data/obj_pose_est.csv");
            csv_est_obj_pose << "Timestamp,X_hat,Y_hat\n";   

            // Object pose by force/torque sensor
            csv_ftsensor_obj_pose.open("/home/" + std::string(getenv("USER")) 
                                            + "/noprehensileman_ws/src/data/obj_pose_ftsensor.csv");
            csv_ftsensor_obj_pose << "Timestamp,X_obj,Y_obj\n"; 

            // Object pose by force/torque sensor
            csv_ftsensor_obj_pose_error.open("/home/" + std::string(getenv("USER")) 
                                            + "/noprehensileman_ws/src/data/obj_pose_error_ftsensor.csv");
            csv_ftsensor_obj_pose_error << "Timestamp,X_obj,Y_obj\n"; 

            // Object pose by camera sensor
            csv_camera_obj_pose.open("/home/" + std::string(getenv("USER")) 
                                            + "/noprehensileman_ws/src/data/obj_pose_camera.csv");
            csv_camera_obj_pose << "Timestamp,X_obj,Y_obj\n"; 

            // Object pose by camera sensor
            csv_camera_obj_pose_error.open("/home/" + std::string(getenv("USER")) 
                                            + "/noprehensileman_ws/src/data/obj_pose_error_camera.csv");
            csv_camera_obj_pose_error << "Timestamp,X_objerr,Y_objerr\n"; 
            // ______________________________________________________________

            // Register signal handler for SIGINT (Ctrl+C)
            signal(SIGINT, signalHandler);
        }

    private:
        double x_error;
        double y_error;
        vector<double> q;

        rclcpp::Time start_time;

        static std::ofstream csv_real_obj_pose;
        static std::ofstream csv_est_obj_pose;
        static std::ofstream csv_joint_positions;
        static std::ofstream csv_obj_pose_error;
        static std::ofstream csv_control_torques;
        static std::ofstream csv_arm_pose;
        static std::ofstream csv_ftsensor_obj_pose;
        static std::ofstream csv_ftsensor_obj_pose_error;
        static std::ofstream csv_camera_obj_pose;
        static std::ofstream csv_camera_obj_pose_error;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr JointStates_Subscriber;  
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr EEPose_Subscriber;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ControlTorques_Subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr error_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr real_pose_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr estimated_state_subscriber;        
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr camera_pose_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr camera_pose_error_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ftsensor_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ftsensor_error_subscriber;

        void ArmPoseCallback (const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            if (msg->data.size() >= 6) 
            { 
                auto elapsed_time = this->now() - start_time;
                csv_arm_pose << elapsed_time.seconds();
                for (int i = 0; i < msg->data.size(); ++i) 
                {
                    csv_arm_pose << "," << msg->data[i];
                }
                csv_arm_pose << "\n";
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Received invalid data");
            }
        }

        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) 
        {    
            if (msg->position.size() >= 7) 
            { 
                auto elapsed_time = this->now() - start_time;
                csv_joint_positions << elapsed_time.seconds();
                for (int i = 0; i < 7; ++i) 
                {
                    csv_joint_positions << "," << msg->position[i];
                }
                csv_joint_positions << "\n";
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Received invalid number of joint positions");
            }
        }

        void controlTorquesCallback (const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            if (msg->data.size() >= 7) 
            { 
                auto elapsed_time = this->now() - start_time;
                csv_control_torques << elapsed_time.seconds();
                for (int i = 0; i < msg->data.size(); ++i) 
                {
                    csv_control_torques << "," << msg->data[i];
                }
                csv_control_torques << "\n";
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Received invalid number of joint positions");
            }
        }

        void cameraPoseCallback(const geometry_msgs::msg::Point::SharedPtr msg) 
        {
            auto elapsed_time = this->now() - start_time;
            csv_camera_obj_pose << elapsed_time.seconds() << "," << msg->x << "," << msg->y << "," << msg->z << endl;
        }

        void cameraPoseErrorCallback(const geometry_msgs::msg::Point::SharedPtr msg) 
        {
            auto elapsed_time = this->now() - start_time;
            csv_camera_obj_pose_error << elapsed_time.seconds() << "," << msg->x << "," << msg->y << "," << msg->z << endl;
        }

        void ftsensorCallback(const geometry_msgs::msg::Point::SharedPtr msg) 
        {
            auto elapsed_time = this->now() - start_time;
            csv_ftsensor_obj_pose << elapsed_time.seconds() << "," << msg->x << "," << msg->y << "," << msg->z << endl;
        }

        void ftsensorErrorCallback(const geometry_msgs::msg::Point::SharedPtr msg) 
        {
            auto elapsed_time = this->now() - start_time;
            csv_ftsensor_obj_pose_error << elapsed_time.seconds() << "," << msg->x << "," << msg->y << "," << msg->z << endl;
        }

        void realPoseCallback(const geometry_msgs::msg::Point::SharedPtr msg)
        {
            auto elapsed_time = this->now() - start_time;
            csv_real_obj_pose << elapsed_time.seconds() << "," << msg->x << "," << msg->y << "," << msg->z << endl;
        }

        void estimatedStateCallback(const geometry_msgs::msg::Point::SharedPtr msg)
        {
            auto elapsed_time = this->now() - start_time;
            csv_est_obj_pose << elapsed_time.seconds() << "," << msg->x << "," << msg->y << "," << msg->z << endl;            
        }

        void ErrorCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
            
            auto elapsed_time = this->now() - start_time;
            csv_obj_pose_error << elapsed_time.seconds();
            csv_obj_pose_error << "," << msg->x << "," << msg->y << "\n";
        }

        static void signalHandler(int signum) {
        if (signum == SIGINT) {
            csv_real_obj_pose.close();
            csv_joint_positions.close();
            csv_obj_pose_error.close();
            csv_control_torques.close();
            csv_arm_pose.close();
            csv_ftsensor_obj_pose.close();
            csv_camera_obj_pose.close();
            csv_ftsensor_obj_pose_error.close();
            csv_camera_obj_pose_error.close();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received SIGINT signal. Closing CSV files and shutting down node.");
            rclcpp::shutdown();
        }
    }
};

std::ofstream Monitor::csv_arm_pose;
std::ofstream Monitor::csv_joint_positions;
std::ofstream Monitor::csv_control_torques;
std::ofstream Monitor::csv_camera_obj_pose;
std::ofstream Monitor::csv_camera_obj_pose_error;
std::ofstream Monitor::csv_ftsensor_obj_pose;
std::ofstream Monitor::csv_ftsensor_obj_pose_error;
std::ofstream Monitor::csv_real_obj_pose;
std::ofstream Monitor::csv_est_obj_pose;
std::ofstream Monitor::csv_obj_pose_error;

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Monitor>());
    rclcpp::shutdown();
    return 0;
}
