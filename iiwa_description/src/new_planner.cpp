// Include necessary headers
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "moveit_msgs/msg/cartesian_point.hpp"
#include <vector>
#include <iostream>
#include <limits>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;
using namespace placeholders;

class NewPlanner : public rclcpp::Node
{
    public:
        NewPlanner() : Node("new_planner")
        {
            nan_data = true;
            ready = false;
            arm_pose = Eigen::VectorXd::Zero(6);

            arm_pose_subscriber = this->create_subscription
                <std_msgs::msg::Float64MultiArray>("/arm_pose", 10, 
                std::bind(&NewPlanner::ArmPoseCallback, this, _1));
        
            command_subscriber = this->create_subscription
                <std_msgs::msg::Float64MultiArray>("/command", 10, 
                std::bind(&NewPlanner::getWaypoints, this, _1));

            ref_traj_publisher = this->create_publisher
                <moveit_msgs::msg::CartesianPoint>("/reference_trajectory", 1);

            auto timercallback = [this]() -> void 
            {
                generate_trajectory();
            };
            timer = this->create_wall_timer(std::chrono::milliseconds(1), timercallback);
        }

    private:
        bool nan_data;
        bool ready;
        vector<vector<double>> waypoints;
        Eigen::VectorXd arm_pose;

        rclcpp::TimerBase::SharedPtr timer;

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_subscriber;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pose_subscriber;
        rclcpp::Publisher<moveit_msgs::msg::CartesianPoint>::SharedPtr ref_traj_publisher;

        void ArmPoseCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            if (msg->data.size() >= 6)
            {
                for (int i = 0; i < 6; ++i)
                {
                    arm_pose[i] = msg->data[i];
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Received invalid data");
            }
        }

        void getWaypoints(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            nan_data = true;
            for (size_t i = 0; i < msg->data.size(); ++i)
            {
                if (!std::isnan(msg->data[i]))
                {
                    nan_data = false;
                    break;
                }
            }

            if (!nan_data)
            {
                waypoints.push_back(msg->data);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Received waypoints matrix with dimensions: %lu x 6", waypoints.size());
                for (const auto &waypoint : waypoints)
                {
                    for (double value : waypoint)
                    {
                        std::cout << value << " ";
                    }
                    std::cout << std::endl;
                }
                nan_data = true;
                ready = true;
            }
        }

        void generate_trajectory()
        {
            if (ready == true)
            {
                // Insert waypoints in a Eigen::MatrixXd 
                size_t rows = waypoints.size();
                size_t cols = waypoints[0].size();
                Eigen::MatrixXd waypoints_matrix(rows, cols);

                for (size_t i = 0; i < rows; ++i)
                {
                    for (size_t j = 0; j < cols; ++j)
                    {
                        waypoints_matrix(i, j) = waypoints[i][j];
                    }
                }


                Eigen::VectorXd velocity = Eigen::VectorXd::Zero(6);
                Eigen::VectorXd acceleration = Eigen::VectorXd::Zero(6);
                Eigen::VectorXd pose = Eigen::VectorXd::Zero(6);

                int T = 10;
                int num_segments = waypoints_matrix.rows() - 1;
                double duration = static_cast<double>(T) / num_segments;


                Eigen::VectorXd v0;
                Eigen::VectorXd vf;

                for (size_t i = 0; i < rows - 1; ++i)
                {
                    Eigen::VectorXd p0 = waypoints_matrix.row(i);
                    Eigen::VectorXd pf = waypoints_matrix.row(i + 1);
                    if (i==0)
                    {
                        v0 = Eigen::VectorXd::Zero(6);
                    }
                    else
                    {
                        v0 = velocity;
                    }
                    
                    Eigen::VectorXd vf;
                    if (i == rows - 2) {
                        vf = Eigen::VectorXd::Zero(6); // Zero final velocity for last waypoint
                    } else {
                        vf = (pf - p0) / duration - v0;
                    }
                    
                    for (double t = 0; t <= duration; t += 0.01) // Adjust the step size as necessary
                    {
                        for (int j = 0; j < 6; ++j)
                        {
                            std::tie(pose[j], velocity[j], acceleration[j]) = interpolate(duration, t, p0[j], pf[j], v0[j], vf[j], 0, 0);
                        }
                        publish_trajectory(pose, velocity, acceleration);
                        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(duration * 1000 / (duration / 0.01))));
                    }
                }

                // 3. Return to initial condition to make ready the node to obtain another set of waypoints
                ready = false;   
                waypoints.clear();
            }
            else
            {
                Eigen::VectorXd zero_velocity = Eigen::VectorXd::Zero(6);
                Eigen::VectorXd zero_acceleration = Eigen::VectorXd::Zero(6);
                publish_trajectory(arm_pose, zero_velocity, zero_acceleration);
            }
        }

        std::tuple<double, double, double> interpolate(double duration, double t, double x0, double xf, double v0, double vf, double a0, double af)
        {
            Eigen::MatrixXd M(6, 6);
            M << 1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 2, 0, 0, 0,
                1, duration, pow(duration, 2), pow(duration, 3), pow(duration, 4), pow(duration, 5),
                0, 1, 2 * duration, 3 * pow(duration, 2), 4 * pow(duration, 3), 5 * pow(duration, 4),
                0, 0, 2, 6 * duration, 12 * pow(duration, 2), 20 * pow(duration, 3);

            Eigen::VectorXd b(6);
            b << x0, v0, a0, xf, vf, af;

            Eigen::VectorXd a = M.inverse()*b;

            double x = a(0) + a(1) * t + a(2) * pow(t, 2) + a(3) * pow(t, 3) + a(4) * pow(t, 4) + a(5) * pow(t, 5);
            double v = a(1) + 2 * a(2) * t + 3 * a(3) * pow(t, 2) + 4 * a(4) * pow(t, 3) + 5 * a(5) * pow(t, 4);
            double a_t = 2 * a(2) + 6 * a(3) * t + 12 * a(4) * pow(t, 2) + 20 * a(5) * pow(t, 3);

            return {x, v, a_t};
        }

        void publish_trajectory(const Eigen::VectorXd &pose,const Eigen::VectorXd &velocity,const Eigen::VectorXd &accelleration)
        {
            moveit_msgs::msg::CartesianPoint x_ref;

            // Convert Roll,Pitch,Yaw in quaternion
            tf2::Quaternion quat;
            quat.setRPY(pose(3),pose(4),pose(5));

            x_ref.pose.position.x = pose(0);
            x_ref.pose.position.y = pose(1);
            x_ref.pose.position.z = pose(2);
            x_ref.pose.orientation.x = quat.getX();
            x_ref.pose.orientation.y = quat.getY();
            x_ref.pose.orientation.z = quat.getZ();
            x_ref.pose.orientation.w = quat.getW();

            x_ref.velocity.linear.x = velocity(0);
            x_ref.velocity.linear.y = velocity(1);
            x_ref.velocity.linear.z = velocity(2);
            x_ref.velocity.angular.x = velocity(3);
            x_ref.velocity.angular.y = velocity(4);
            x_ref.velocity.angular.z = velocity(5);

            x_ref.acceleration.linear.x = accelleration[0];
            x_ref.acceleration.linear.y = accelleration[1];
            x_ref.acceleration.linear.z = accelleration[2];
            x_ref.acceleration.angular.x = accelleration[3];
            x_ref.acceleration.angular.y = accelleration[4];
            x_ref.acceleration.angular.z = accelleration[5];

            // Publish pose, velocity, accelleration
            ref_traj_publisher->publish(x_ref);
        }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NewPlanner>());
    rclcpp::shutdown();
    return 0;
}
