#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

using namespace std;
using namespace chrono_literals;

class Planner : public rclcpp::Node
{
public:
    Planner() : Node("joint_space_planner_node")
    {
        // Load URDF from file
        string urdf_file_path = "src/iiwa_description/urdf/kuka_iiwa.urdf";
        string urdf_string = load_urdf(urdf_file_path);
        if (urdf_string.empty()) {
            RCLCPP_ERROR(get_logger(), "Failed to load URDF from file");
            return;
        }

        // Parse URDF string and initialize KDL Chain
        if (!kdl_parser::treeFromString(urdf_string, kdl_tree)) {
            RCLCPP_ERROR(get_logger(), "Failed to parse URDF");
            return;
        }

        // Specify the chain of the manipulator (assuming it's named "manipulator")
        if (!kdl_tree.getChain("base_link", "tray", kdl_chain)) {
            RCLCPP_ERROR(get_logger(), "Failed to get KDL chain");
            return;
        }

        q_init.resize(kdl_chain.getNrOfJoints(), 0.0);
        q_des.resize(kdl_chain.getNrOfJoints(), 0.0);
        q_dot_des.resize(kdl_chain.getNrOfJoints(), 0.0);
        q_dotdot_des.resize(kdl_chain.getNrOfJoints(), 0.0);

        // Initizialize FK solver
        fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain);

        // Initialize IK solver:
        // - First parameter: kdl chain
        // - Second paramter (double): desired accuracy in task space
        // - Third parameter (int): maximum number of iterations
        // - Fourth paramter (double): tolerance for joint angle increments
        ik_solver = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain, 1e-6, 100, 1e-12);

        // Subscriber for get current configuration of manipulator
        joint_states_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, bind(&Planner::jointStateCallback, this, placeholders::_1));

        // Publisher for publish reference trajectory 
        references_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("/reference_trajectory", 10);

        // Start functions
        auto timercallback = [this]() -> void 
        {
            get_user_input();
            generate_trajectory();
        };
        timer = this->create_wall_timer(chrono::milliseconds(1), timercallback);
    }

private:
    vector<double> q_init;
    vector<double> q_des;
    vector<double> q_dot_des;
    vector<double> q_dotdot_des;
    geometry_msgs::msg::Pose x_e_des;
    geometry_msgs::msg::Pose x_init;
    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr references_publisher;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber;
    rclcpp::TimerBase::SharedPtr timer;

    // _____________Member Functions_________________

    // Function to read file urdf of robot
    string load_urdf(const string& urdf_file_path) {
        
        ifstream file(urdf_file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open URDF file");
            return "";
        }
        stringstream buffer;
        buffer << file.rdbuf();
        file.close();
        return buffer.str();
    }

    // Function to obtain by user desired configuration of robot or tray pose
    void get_user_input()
    {
        int choose;
        cout << "Menu:" << endl;
        cout << "1. Home configuration."  << endl;
        cout << "2. Vertical configuration."  << endl;
        cout << "3. Custom configuration."  << endl;
        cout << "4. Custom tray pose." << endl;
        cout << "Command: ";
        cin >> choose;

        if (choose == 1)
        {
            // Home configuration
            q_des = {2.15, 0.08, -2.22, -1.6, 0.06, -1.55, 0.07};
        }
        else if (choose == 2)
        {
            // Vertical configuration
            q_des = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        }
        else if (choose == 3)
        {
            // Show current configuration of robot (in radians and degrees)
            cout << "Current Configuration (radians): [";
            for (size_t i = 0; i < kdl_chain.getNrOfJoints(); ++i)
            {
                cout << " " << q_init[i];
            }
            cout << " ]" << endl;
            cout << "Current Configuration (degrees): [";
            for (size_t i = 0; i < kdl_chain.getNrOfJoints(); ++i)
            {
                double angle_in_degrees = q_init[i] * (180.0 / M_PI); // Convert radians to degrees
                // Normalize the angle between -180 and 180
                angle_in_degrees = fmod(angle_in_degrees + 180.0, 360.0) - 180.0;
                if (angle_in_degrees < -180.0) {
                    angle_in_degrees += 360.0;  // Correct the range to be -180 to 180
                }
                cout << " " << angle_in_degrees; // Display angles in degrees
            }
            cout << " ]" << endl;

            // Insert desired configuration
            cout << "Please enter desired joint positions for 7 joints:" << endl;
            for (size_t i = 0; i < 7; ++i)
            {
                string joint_name = "joint_" + to_string(i + 1);
                cout << joint_name << " : ";
                cin >> q_des[i];
            }
        }
        else if (choose == 4)
        {
            // Prompt user for desired end-effector pose
            std::cout << "Enter desired position (x, y, z): ";
            std::cin >> x_e_des.position.x >> x_e_des.position.y >> x_e_des.position.z;

            double roll, pitch, yaw;
            std::cout << "Enter desired orientation (roll, pitch, yaw in degrees): ";
            std::cin >> roll >> pitch >> yaw;

            // Convert roll, pitch, yaw to quaternion
            tf2::Quaternion quat;
            quat.setRPY(roll * M_PI / 180.0, pitch * M_PI / 180.0, yaw * M_PI / 180.0);
            tf2::convert(quat, x_e_des.orientation);

            // Inverse Kinematics
            inverse_kinematics(x_e_des, q_des);
        }
    }

    // Function to compute inverse kinematics in case user select desired tray pose
    void inverse_kinematics(const geometry_msgs::msg::Pose& x_e_des, vector<double>& q_des)
    {
        // Define the end-effector frame
        KDL::Frame end_effector_frame;
        tf2::Quaternion q(x_e_des.orientation.x, x_e_des.orientation.y, x_e_des.orientation.z, x_e_des.orientation.w);
        tf2::Vector3 p(x_e_des.position.x, x_e_des.position.y, x_e_des.position.z);
        end_effector_frame.M = KDL::Rotation::Quaternion(q.getX(), q.getY(), q.getZ(), q.getW());
        end_effector_frame.p = KDL::Vector(p.getX(), p.getY(), p.getZ());
       
        // Define joint positions vector
        KDL::JntArray q_init_kdl(kdl_chain.getNrOfJoints());
        for (size_t i = 0; i < q_init.size(); ++i)
        {    
            q_init_kdl(i) = q_init[i];
        }
        // Define joint positions result vector
        KDL::JntArray q_result(kdl_chain.getNrOfJoints());

        // Perform IK calculation
        int ik_result = ik_solver->CartToJnt(q_init_kdl, end_effector_frame, q_result);
        
        if (ik_result < 0) {
            RCLCPP_ERROR(get_logger(), "IK calculation failed");
            return;
        }

        // Store IK result in q_des
        for (size_t i = 0; i < q_des.size(); ++i)
        {    
            q_des[i] = q_result(i);
            cout << "Joint " << i+1 << ": "<< q_des[i] << endl;
        }
    }

    // Function to obtain current configuration of manipulator
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            q_init[i] = msg->position[i];
        }
    }
    
    // Function to compute and send reference position, velocity and accelleration trajectories
    void generate_trajectory()
    {   
        // Total duration of trajectory in seconds
        double T = 5.0; 
        // Time step in seconds
        double dt = 0.1; 
        // Number of steps in the trajectory
        int steps = static_cast<int>(T / dt); 

        for (int i = 0; i < steps; ++i)
        {
            double t = i * dt;
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.time_from_start = rclcpp::Duration::from_seconds(t);
            for (int j = 0; j < 7; ++j) // For each joint
            {
                // Cubic spline coefficients
                double a0 = q_init[j];
                double a1 = 0;
                double a2 = 3 * (q_des[j] - q_init[j]) / std::pow(T, 2);
                double a3 = -2 * (q_des[j] - q_init[j]) / std::pow(T, 3);

                // Calculate position, velocity, and acceleration
                double position = a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3);
                double velocity = a1 + 2 * a2 * t + 3 * a3 * std::pow(t, 2);
                double acceleration = 2 * a2 + 6 * a3 * t;

                // Add a new point to trajectory
                point.positions.push_back(position);
                point.velocities.push_back(velocity);
                point.accelerations.push_back(acceleration);
            }
            // Publish
            references_publisher->publish(point);
        }
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
