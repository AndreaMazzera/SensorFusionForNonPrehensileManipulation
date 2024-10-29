//_____________________________USER MENU___________________________________
// This script provides a user menu on the terminal where the user can 
// select the desired trajectory
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <iostream>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

#include "iiwa_msgs/srv/command.hpp"

using namespace std;
using namespace Eigen;
using Command = iiwa_msgs::srv::Command;

class UserMenu : public rclcpp::Node
{
    public:
        UserMenu() : Node("service_client_node")
        {
            // Create a client for the iiwa_msgs::srv::Command service
            client = create_client<Command>("/desired_pose");

            // Publisher to visualize in RViz pose send controller
            pose_pub = this->create_publisher
                <geometry_msgs::msg::PoseStamped>("/desired_pose_visualization", 10);

            // Wait for the service to become available
            while (!client->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
            }

            get_user_input();
        }

    private:
        rclcpp::TimerBase::SharedPtr timer;

        // Publisher
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;

        // Client
        rclcpp::Client<Command>::SharedPtr client;
        
        // Function to draw the user menu in the terminal
        void get_user_input()
        {
            bool close_script = false;
            int choose;
            int tf;
            while (close_script == false)
            {
                cout << "Menu:" << endl;
                cout << "1. Home configuration." << endl;
                cout << "2. Vertical configuration." << endl;
                cout << "3. Linear trajectory." << endl;
                cout << "4. Square trajectory." << endl;
                cout << "5. Circular trajectory Horizontal Tray." << endl;
                cout << "6. Circular trajectory Inclined Tray." << endl;  
                cout << "7. Eight trajectory." << endl;
                cout << "8. Eight trajectory Inclined Tray." << endl;
                cout << "9. Exit" << endl;
                cout << "Command: ";
                cin >> choose;

                // Integer value xf specify a trajectory and tf is execution time
                if (choose >= 1 && choose <= 8)
                {
                    cout << "Choose time to execute trajectory: ";
                    cin >> tf;
                    system("clear");
                    call_service(choose,tf);
                }
                else if(choose == 9)
                {
                    // Kill node
                    RCLCPP_INFO(this->get_logger(), "Node killed by user.");
                    close_script = true;
                }
                else
                {
                    cout << "Invalid choice. Exiting." << std::endl;
                    system("clear");
                }
            }
            rclcpp::shutdown(); 
            exit(0); 
        }

        void call_service(const int& xf,const int& tf)
        { 
            cout << "Time to trajectory execution: " << tf << endl;
            auto request = std::make_shared<Command::Request>();
            request->xf = xf;
            request->tf = tf;

            auto result = client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = result.get();
                if (response->finish == true)
                    RCLCPP_INFO(this->get_logger(), "Service call successful.");
                else
                    RCLCPP_ERROR(this->get_logger(), "Service call failed.");
            }
            else{ RCLCPP_ERROR(this->get_logger(), "Failed to call service.");} 

            get_user_input();
        }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UserMenu>());
    rclcpp::shutdown();
    return 0;
}
