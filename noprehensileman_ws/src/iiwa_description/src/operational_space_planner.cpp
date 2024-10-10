//_____________________OPERATIONAL SPACE PLANNER__________________________
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/cartesian_point.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <iostream>
#include <cmath>
#include <fstream>
#include <chrono>
#include <Eigen/Dense>

#include "iiwa_msgs/srv/command.hpp"

using namespace std;
using namespace placeholders;
using namespace chrono_literals;

class Planner : public rclcpp::Node
{
    public:
        Planner() : Node("Op_Planner")
        {
            T = 5.0;
            xf = 0.0;
            xe_0.resize(6);
 
            x_ref = moveit_msgs::msg::CartesianPoint();

            // _______________________________________________________________________
            // _________________________SUBSCRIBERS___________________________________ 
            // Subscribe to get pose of manipulator's end-effector
            ArmPose_Subscriber = this->create_subscription
                <std_msgs::msg::Float64MultiArray>("/arm_pose", 10,
                    std::bind(&Planner::EEPoseCallback, this, _1));
            //___________________________________________________________________________
            // ____________________________PUBLISHER_____________________________________ 
            // Publisher for publish reference trajectory 
            RefTraj_Publisher = this->create_publisher
                <moveit_msgs::msg::CartesianPoint>("/reference_trajectory", 1);

            // Publisher to desired pose for visualize in RViz
            VisualPath_Publisher = this->create_publisher
                <visualization_msgs::msg::Marker>("/RefPath_Frame", 10);

            // Service to receive order by client
            pose_service = this->create_service
                <iiwa_msgs::srv::Command>("/desired_pose", 
                std::bind(&Planner::handle_pose_service, this, _1, _2));
            // _______________________________________________________________________

            // Timer to publish the default pose when there are no requests
            timer = this->create_wall_timer(1ms, std::bind(&Planner::publish_default_pose, this));           
        }

    private:
        int xf;
        int T;
        Eigen::VectorXd xe_0;
        moveit_msgs::msg::CartesianPoint x_ref;
        rclcpp::TimerBase::SharedPtr timer;
        
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ArmPose_Subscriber;
        rclcpp::Publisher<moveit_msgs::msg::CartesianPoint>::SharedPtr RefTraj_Publisher;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr VisualPath_Publisher;
        rclcpp::Service<iiwa_msgs::srv::Command>::SharedPtr pose_service;

        // _____________Member Functions_________________
        
        // Function to obtain initial pose of end-effector
        void EEPoseCallback (const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            if (msg->data.size() >= 6) 
            { 
                for (int i = 0; i < msg->data.size(); ++i) 
                    xe_0[i] = msg->data[i];
            }
            else
                RCLCPP_WARN(get_logger(), "Received invalid data");
        }

        // Function to publish the default pose when there are no requests
        void publish_default_pose()
        {
            tf2::Quaternion quat;
            quat.setRPY(xe_0[3], xe_0[4], xe_0[5]);

            x_ref.pose.position.x = xe_0[0];
            x_ref.pose.position.y = xe_0[1];
            x_ref.pose.position.z = xe_0[2];
            x_ref.pose.orientation.x = quat.getX();
            x_ref.pose.orientation.y = quat.getY();
            x_ref.pose.orientation.z = quat.getZ();
            x_ref.pose.orientation.w = quat.getW();

            x_ref.velocity.linear.x = 0.0;
            x_ref.velocity.linear.y = 0.0;
            x_ref.velocity.linear.z = 0.0;
            x_ref.velocity.angular.x = 0.0;
            x_ref.velocity.angular.y = 0.0;
            x_ref.velocity.angular.z = 0.0;

            x_ref.acceleration.linear.x = 0.0;
            x_ref.acceleration.linear.y = 0.0;
            x_ref.acceleration.linear.z = 0.0;
            x_ref.acceleration.angular.x = 0.0;
            x_ref.acceleration.angular.y = 0.0;
            x_ref.acceleration.angular.z = 0.0;

            // Publish the default pose
            RefTraj_Publisher->publish(x_ref);
        }

        // Service callback to handle incoming pose requests and inform trajectory completion
        void handle_pose_service (const std::shared_ptr<iiwa_msgs::srv::Command::Request> request,
                                        std::shared_ptr<iiwa_msgs::srv::Command::Response> response)
        {
            // Set desired pose
            xf       = request->xf;      
            T        = request->tf;

            cout << "Received request " << xf << endl;

            Eigen::MatrixXd dest;
            bool end_traj = false;

            // Home configuration
            if (xf == 1)
            {
                dest = Eigen::MatrixXd(2,6);
                dest.row(0) << xe_0[0], xe_0[1], xe_0[2], xe_0[3], xe_0[4], xe_0[5];
                dest.row(1) << 0.38, 0.0, 0.911, 0.0, 0.0, 0.0;
                PTPTraj(dest);
            }
            // Vertical configuration
            else if (xf == 2)
            {
                dest = Eigen::MatrixXd(2,6);
                dest.row(0) << xe_0[0], xe_0[1], xe_0[2], xe_0[3], xe_0[4], xe_0[5];
                dest.row(1) << 0.0, 0.0, 1.29, 0.0, 0.0, 0.0;
                PTPTraj(dest);
            }
            // Horizontal Segment
            else if (xf == 3)
            {
                dest = Eigen::MatrixXd(3,6);
                dest.row(0) << xe_0[0], xe_0[1], xe_0[2], xe_0[3], xe_0[4], xe_0[5];
                dest.row(1) << 0.4, 0.37, 0.911, 0.0, 0.0, 0.0;
                dest.row(2) << 0.4, -0.37, 0.911, 0.0, 0.0, 0.0;
                PTPTraj(dest);
            }
            // Square Segment 
            // TODO: probably wrong waypoints
            else if (xf == 4)
            {
                dest = Eigen::MatrixXd(6,6);
                dest.row(0) << xe_0[0], xe_0[1], xe_0[2], xe_0[3], xe_0[4], xe_0[5];
                dest.row(1) << 0.3, 0.3, 0.911, 0.0, 0.0, 0.0;
                dest.row(2) << 0.3, 0.37, 0.911, 0.0, 0.0, 0.0;
                dest.row(3) << 0.3, -0.38, 0.911, 0.0, 0.0, 0.0;
                dest.row(4) << 0.3, -0.3, 0.911, 0.0, 0.0, 0.0;
                dest.row(5) << 0.3, 0.3, 0.911, 0.0, 0.0, 0.0;
                PTPTraj(dest);
            }
            // Circular Trajectory Without Tray Inclination
            else if (xf == 5)
            {
                double final_angle_deg = 90.0; // Set the desired final angle in degrees
                bool clockwise = true;     // Set the desired direction
                bool inclined_tray = false;
                Eigen::MatrixXd waypoints = CircTraj(final_angle_deg, clockwise,inclined_tray);
                // Optionally, you can pub
                dest = Eigen::MatrixXd(waypoints.rows(),6);
                dest << waypoints;
                PTPTraj(dest);
            }
            // Circular Trajectory With Tray Inclination
            else if (xf == 6)
            {
                double final_angle_deg = 90.0; // Set the desired final angle in degrees
                bool clockwise = true;     // Set the desired direction
                bool inclined_tray = true;
                Eigen::MatrixXd waypoints = CircTraj(final_angle_deg, clockwise,inclined_tray);
                dest = Eigen::MatrixXd(waypoints.rows(),6);
                dest << waypoints;
                PTPTraj(dest);
            }
            // Eight-Shape Trajectory Without Tray Inclination
            else if (xf == 7) 
            {
                EightTraj(false);
            }
            // Eight-Shape Trajectory With Tray Inclination
            else if (xf == 8) 
            {
                EightTraj(true);
            }

            // Reply to client
            response->finish = true;
        }
        
        // Quintic Polynomial Interpolation
        Eigen::Vector3d QuinticPolInterp (  const double &t, const double &T, 
                                            const double &xi, const double &xf, 
                                            const double &vi, const double &vf,
                                            const double &ai, const double &af)
        {
            Eigen::Vector3d interpolation;

            // Define A matrix
            Eigen::MatrixXd A(6, 6);
            A << 1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 2, 0, 0, 0,
                1, T, pow(T, 2), pow(T, 3), pow(T, 4), pow(T, 5),
                0, 1, 2 * T, 3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4),
                0, 0, 2, 6 * T, 12 * pow(T, 2), 20 * pow(T, 3);

            // Define b vector
            Eigen::VectorXd b(6);
            b << xi, vi, ai, xf, vf, af;

            // Solve for x
            Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);

            // Extract coefficients a0, a1, a2, a3, a4, a5
            double a0 = x(0);
            double a1 = x(1);
            double a2 = x(2);
            double a3 = x(3);
            double a4 = x(4);
            double a5 = x(5);

            // Calculate position, velocity, and acceleration
            interpolation[0] = a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) + a4 * pow(t, 4) + a5 * pow(t, 5);
            interpolation[1] = a1 + 2 * a2 * t + 3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3) + 5 * a5 * pow(t, 4);
            interpolation[2] = 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2) + 20 * a5 * pow(t, 3);

            return interpolation;
        }

        // Function that plan Poin-To-Point trajectory
        void PTPTraj(const Eigen::MatrixXd& dest)
        {
            //________________________________________________________
            // Line strip with width (scale) 0.01 and color red
            visualization_msgs::msg::Marker line_strip;
            line_strip.header.frame_id = "world";
            line_strip.header.stamp = this->now();
            line_strip.ns = "trajectory_path";
            line_strip.action = visualization_msgs::msg::Marker::ADD;
            line_strip.pose.orientation.w = 1.0;
            line_strip.id = 0;
            line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_strip.scale.x = 0.01; 
            line_strip.color.r = 1.0;
            line_strip.color.a = 1.0;
            //________________________________________________________

            vector<double> pose_buffer(6);
            vector<double> vel_buffer(6);
            vector<double> acc_buffer(6);
            
            int num_segments = dest.rows() - 1;
            double T_segment = static_cast<double>(T) / num_segments;

            Eigen::VectorXd v0;
            Eigen::VectorXd vf;
            
            // Loop through each waypoint
            for (int i = 0; i < dest.rows()-1; i++)
            {
                // Calculate trajectory duration
                std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                std::chrono::steady_clock::time_point end = begin 
                        + std::chrono::milliseconds(static_cast<int>((T_segment)* 1000));
     
                Eigen::VectorXd p0 = dest.row(i);
                Eigen::VectorXd pf = dest.row(i + 1);
    
                // Initial velocity
                if (i==0)
                    v0 = Eigen::VectorXd::Zero(6);
                else
                    v0 << vel_buffer[0],vel_buffer[1],vel_buffer[2],vel_buffer[3],vel_buffer[4],vel_buffer[5];
                // Final velocity
                if (i == dest.rows() - 2) 
                    vf = Eigen::VectorXd::Zero(6); 
                else 
                    vf = (pf - p0)/T_segment;
                
                    
                while (std::chrono::steady_clock::now() < end)
                {
                    double t = std::chrono::duration<double>
                                (std::chrono::steady_clock::now() - begin).count();

                    for (int j = 0; j < 6; ++j) 
                    {
                        Eigen::Vector3d interpolation = QuinticPolInterp(t, T_segment, p0[j],pf[j], v0[j], vf[j], 0.0, 0.0);
                        pose_buffer[j]  = interpolation[0];
                        vel_buffer[j]   = interpolation[1];
                        acc_buffer[j]   = interpolation[2];
                    }

                    tf2::Quaternion quat;
                    quat.setRPY(pose_buffer[3], pose_buffer[4], pose_buffer[5]);

                    x_ref.pose.position.x = pose_buffer[0];
                    x_ref.pose.position.y = pose_buffer[1];
                    x_ref.pose.position.z = pose_buffer[2];
                    x_ref.pose.orientation.x = quat.getX();
                    x_ref.pose.orientation.y = quat.getY();
                    x_ref.pose.orientation.z = quat.getZ();
                    x_ref.pose.orientation.w = quat.getW();

                    // Add the current pose to the line strip for RViz visualization
                    geometry_msgs::msg::Point p;
                    p.x = pose_buffer[0];
                    p.y = pose_buffer[1];
                    p.z = pose_buffer[2];
                    line_strip.points.push_back(p);

                    x_ref.velocity.linear.x = vel_buffer[0];
                    x_ref.velocity.linear.y = vel_buffer[1];
                    x_ref.velocity.linear.z = vel_buffer[2];
                    x_ref.velocity.angular.x = vel_buffer[3];
                    x_ref.velocity.angular.y = vel_buffer[4];
                    x_ref.velocity.angular.z = vel_buffer[5];

                    x_ref.acceleration.linear.x = acc_buffer[0];
                    x_ref.acceleration.linear.y = acc_buffer[1];
                    x_ref.acceleration.linear.z = acc_buffer[2];
                    x_ref.acceleration.angular.x = acc_buffer[3];
                    x_ref.acceleration.angular.y = acc_buffer[4];
                    x_ref.acceleration.angular.z = acc_buffer[5];

                    // Publish
                    RefTraj_Publisher->publish(x_ref);
                }
            }
            // Publish the trajectory path as a line strip marker
            VisualPath_Publisher->publish(line_strip);
        } 
   
        // Function that plan circular trajectory
        Eigen::MatrixXd CircTraj(double final_angle_deg, bool clockwise,bool inclined_tray)
        {
            // Extract initial pose components
            double x = xe_0(0);
            double y = xe_0(1);
            double z = xe_0(2);
            double roll = xe_0(3);
            double pitch = xe_0(4);
            double yaw = xe_0(5);

            // Compute radius as the square root of the sum of squares of x and y
            double radius = sqrt(x * x + y * y);

            // Compute initial angle from x and y coordinates
            double initial_angle = atan2(y, x);

            // Convert final angle from degrees to radians
            double final_angle_rad = final_angle_deg * M_PI / 180.0;

            // Define the number of waypoints for the circular trajectory
            int num_waypoints = 10;

            // Generate circular trajectory waypoints
            Eigen::MatrixXd waypoints(6, num_waypoints); 
            double direction;
            if (clockwise == true ) direction = -1.0;
            else direction = 1.0;
            
            for (int i = 0; i < num_waypoints; ++i)
            {
                double theta = initial_angle + i * (final_angle_rad / (num_waypoints-1)) * direction; 
                waypoints(0, i) = radius * cos(theta);                         
                waypoints(1, i) = radius * sin(theta);                         
                waypoints(2, i) = z;  
                if (inclined_tray == false)                                            
                    waypoints(3, i) = roll;       
                else 
                    waypoints(3, i) = 0.47;
                waypoints(4, i) = pitch;                                          
                waypoints(5, i) = yaw;                             
            }

            // Return the transposed matrix of waypoints
            return waypoints.transpose();
        }

        // Function that plan eight-shape trajectory
        void EightTraj(bool dynamic_rpy)
        {
            //________________________________________________________
            // Line strip with width (scale) 0.01 and color red
            visualization_msgs::msg::Marker line_strip;
            line_strip.header.frame_id = "world";
            line_strip.header.stamp = this->now();
            line_strip.ns = "trajectory_path";
            line_strip.action = visualization_msgs::msg::Marker::ADD;
            line_strip.pose.orientation.w = 1.0;
            line_strip.id = 0;
            line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_strip.scale.x = 0.01; 
            line_strip.color.r = 1.0;
            line_strip.color.a = 1.0;
            //________________________________________________________

            // Extract initial pose components
            double x0 = xe_0(0);
            double y0 = xe_0(1);
            double z0 = xe_0(2);
            double roll0 = xe_0(3);
            double pitch0 = xe_0(4);
            double yaw0 = xe_0(5);

            // Define parameters
            double a = 0.2;
            double omega = 2 * M_PI / T;
            double dt = 0.001;
            int num_points = static_cast<int>(T/ dt) + 1;

            for (int i = 0; i < num_points; ++i)
            {
                double t = i * dt;
                
                // Compute position
                std::vector<double> pose_buffer(6);
                pose_buffer[0] = x0 + a * sin(omega * t) * cos(omega * t);
                pose_buffer[1] = y0 + a * sin(omega * t);
                pose_buffer[2] = z0;
                pose_buffer[3] = roll0;
                pose_buffer[4] = pitch0;
                pose_buffer[5] = yaw0;
                if (dynamic_rpy)
                {
                    pose_buffer[3] = 0.5*cos(omega*t); 
                    pose_buffer[4] = 0.5*sin(omega*t); 
                }
                // Compute velocity
                std::vector<double> vel_buffer(6);
                vel_buffer[0] = a * omega* (cos(omega*t)*cos(omega*t)-sin(omega*t)*sin(omega*t));
                vel_buffer[0] = a * omega* cos(omega*t);
                vel_buffer[2] = 0.0;
                vel_buffer[3] = 0.0;
                vel_buffer[4] = 0.0;
                vel_buffer[5] = 0.0;
       
                // Compute acceleration
                std::vector<double> acc_buffer(6);
                acc_buffer[0] = -4 * a * omega * omega * std::cos(omega * t) * std::sin(omega * t);
                acc_buffer[0] = -a * omega * omega * std::sin(omega * t);
                acc_buffer[2] = 0.0;
                acc_buffer[3] = 0.0;
                acc_buffer[4] = 0.0;
                acc_buffer[5] = 0.0;
           
                tf2::Quaternion quat;
                quat.setRPY(pose_buffer[3], pose_buffer[4], pose_buffer[5]);

                x_ref.pose.position.x = pose_buffer[0];
                x_ref.pose.position.y = pose_buffer[1];
                x_ref.pose.position.z = pose_buffer[2];
                x_ref.pose.orientation.x = quat.getX();
                x_ref.pose.orientation.y = quat.getY();
                x_ref.pose.orientation.z = quat.getZ();
                x_ref.pose.orientation.w = quat.getW();

                // Add the current pose to the line strip for RViz visualization
                geometry_msgs::msg::Point p;
                p.x = pose_buffer[0];
                p.y = pose_buffer[1];
                p.z = pose_buffer[2];
                line_strip.points.push_back(p);

                x_ref.velocity.linear.x = vel_buffer[0];
                x_ref.velocity.linear.y = vel_buffer[1];
                x_ref.velocity.linear.z = vel_buffer[2];
                x_ref.velocity.angular.x = vel_buffer[3];
                x_ref.velocity.angular.y = vel_buffer[4];
                x_ref.velocity.angular.z = vel_buffer[5];

                x_ref.acceleration.linear.x = acc_buffer[0];
                x_ref.acceleration.linear.y = acc_buffer[1];
                x_ref.acceleration.linear.z = acc_buffer[2];
                x_ref.acceleration.angular.x = acc_buffer[3];
                x_ref.acceleration.angular.y = acc_buffer[4];
                x_ref.acceleration.angular.z = acc_buffer[5];

                // Publish
                RefTraj_Publisher->publish(x_ref);
                // Delay to simulate real-time
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            RCLCPP_INFO(this->get_logger(), "Trajectory completed.");
            // Publish the trajectory path as a line strip marker
            VisualPath_Publisher->publish(line_strip);
        }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}