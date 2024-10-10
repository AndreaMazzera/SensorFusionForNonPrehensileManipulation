#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <limits>

using namespace std;
using namespace placeholders;

class Planner : public rclcpp::Node 
{
    public:
        Planner() : Node("new_client") 
        {
            xe_0        = Eigen::VectorXd::Zero(6);
            waypoints   = Eigen::MatrixXd::Zero(0,0);

            arm_pose_subscriber_ = this->create_subscription
                <std_msgs::msg::Float64MultiArray>("/arm_pose", 10, 
                std::bind(&Planner::ArmPoseCallback, this, _1));

            command_publisher_ = this->create_publisher
                <std_msgs::msg::Float64MultiArray>("/command", 10);

                    // Create a timer that periodically checks for user input
            auto timercallback = [this]() -> void 
            {
                displayMenu();
            };
            timer = this->create_wall_timer(std::chrono::milliseconds(1), timercallback);
        }

    private:
        Eigen::VectorXd xe_0;
        Eigen::MatrixXd waypoints;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pose_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
        
        void ArmPoseCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) 
        {
            if (msg->data.size() == 6) 
            {
                for (size_t i = 0; i < 6; ++i) 
                    xe_0(i) = msg->data[i];
            } 
            else 
                RCLCPP_ERROR(this->get_logger(), "Received pose with incorrect size");
            
        }

        void displayMenu() 
        {
            bool ok = false;
            while (ok==false) 
            {
                cout << "Select new trajectory:\n";
                cout << "1. Home\n";
                cout << "2. Point to Point\n";
                cout << "3. Circular\n";
                cout << "4. Send predefined waypoints\n";
                cout << "5. Stop node\n";
                cout << "Enter choice (1-3): ";
                int choice;
                cin >> choice;

                switch (choice) 
                {
                    case 1:
                        waypoints = Eigen::MatrixXd(2,6);
                        waypoints.row(0) << xe_0[0], xe_0[1], xe_0[2], xe_0[3], xe_0[4], xe_0[5];
                        waypoints.row(1) << 0.38, 0.0, 0.911, 0.0, 0.0, 0.0;            
                        ok = true;
                        break;
                    case 2:
                        PtPTraj();
                        ok = true;
                        break;
                    case 3:
                        CircTraj(90.0, true,false);
                        ok = true;
                        break;
                    case 4:
                        LinearTraj();
                        ok = true;
                        break;
                    case 5: 
                        cout << "Kill node" << endl;
                        rclcpp::shutdown(); // Make sure to shutdown ROS properly
                        exit(0); // Exit the
                    default:
                        cout << "Invalid choice. Please try again.\n";
                        break;
                }
                system("clear");
            }

            if (waypoints.rows()==0)
            {
                std_msgs::msg::Float64MultiArray default_msg;
                default_msg.data.resize(6, std::numeric_limits<double>::quiet_NaN());
                
                command_publisher_->publish(default_msg);
            }
            else
            {
                cout << "Waypoints: " << endl;
                cout << waypoints << endl;
                cout << "___________________________________" << endl;
                
                for (int i=0; i<waypoints.rows(); i++) 
                {
                    std_msgs::msg::Float64MultiArray msg;
                    msg.data = {waypoints(i,0),waypoints(i,1),waypoints(i,2),waypoints(i,3),waypoints(i,4),waypoints(i,5)};        
                    command_publisher_->publish(msg);
                }
                waypoints   = Eigen::MatrixXd::Zero(0,0);

                std_msgs::msg::Float64MultiArray default_msg;
                default_msg.data.resize(6, std::numeric_limits<double>::quiet_NaN());
                
                command_publisher_->publish(default_msg);
            }
        }

        // Simple point-to-point trajectory
        void PtPTraj() 
        {
            Eigen::VectorXd target(6);
            std::cout << "Enter target position (x, y, z): ";
            std::cin >> target(0) >> target(1) >> target(2);
            std::cout << "Enter target orientation (roll, pitch, yaw): ";
            std::cin >> target(3) >> target(4) >> target(5);

            waypoints = Eigen::MatrixXd(2,6);
            waypoints.row(0) << xe_0[0], xe_0[1], xe_0[2], xe_0[3], xe_0[4], xe_0[5];
            waypoints.row(1) << target(0), target(1), target(2), target(3), target(4), target(5);               
        }

        // Linear trajectory
        void LinearTraj() 
        {
            waypoints = Eigen::MatrixXd(3,6);
            waypoints.row(0) << xe_0[0], xe_0[1], xe_0[2], xe_0[3], xe_0[4], xe_0[5];
            waypoints.row(1) << 0.4, 0.37, 0.911, 0.0, 0.0, 0.0;
            waypoints.row(2) << 0.4, -0.37, 0.911, 0.0, 0.0, 0.0;
        }

        // Circular Trajectory
        void CircTraj(double final_angle_deg, bool clockwise,bool inclined_tray)
        {
            waypoints = Eigen::MatrixXd(10,6);

            // Extract initial pose components
            double x = xe_0[0];
            double y = xe_0[1];
            double z = xe_0[2];
            double roll = xe_0[3];
            double pitch = xe_0[4];
            double yaw = xe_0[5];

            // Compute radius as the square root of the sum of squares of x and y
            double radius = sqrt(x * x + y * y);

            // Compute initial angle from x and y coordinates
            double initial_angle = atan2(y, x);

            // Convert final angle from degrees to radians
            double final_angle_rad = final_angle_deg * M_PI / 180.0;

            // Define the number of waypoints for the circular trajectory
            int num_waypoints = 10;

            // Generate circular trajectory waypoints
            double direction;
            if (clockwise == true ) direction = -1.0;
            else direction = 1.0;
            
            for (int i = 0; i < num_waypoints; ++i)
            {
                double theta = initial_angle + i * (final_angle_rad / (num_waypoints-1)) * direction; 
                waypoints(i,0) = radius * cos(theta);                         
                waypoints(i,1) = radius * sin(theta);                         
                waypoints(i,2) = z;  
                if (inclined_tray == false)                                            
                    waypoints(i,3) = roll;       
                else 
                    waypoints(i,3) = 0.45;
                waypoints(i,4) = pitch;                                          
                waypoints(i,5) = yaw;                             
            }
        }
};

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
