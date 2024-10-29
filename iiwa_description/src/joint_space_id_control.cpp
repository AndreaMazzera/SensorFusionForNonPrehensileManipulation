#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <iostream>
#include <fstream>

using namespace std;
using namespace chrono_literals;

class IDControllerClass : public rclcpp::Node {
public:
    IDControllerClass() : Node("id_control_node", rclcpp::NodeOptions()), Kp(20), Kd(20) 
    { 
        // Load URDF from file
        string urdf_file_path = "src/iiwa_description/urdf/kuka_iiwa.urdf";
        string urdf_string = loadURDFFromFile(urdf_file_path);
        if (urdf_string.empty()) {
            RCLCPP_ERROR(get_logger(), "Failed to load URDF from file");
            return;
        }

        // Parse URDF string and initialize KDL Chain
        if (!kdl_parser::treeFromString(urdf_string, kdl_tree_)) {
            RCLCPP_ERROR(get_logger(), "Failed to parse URDF");
            return;
        }

        // Specify the chain of the manipulator (assuming it's named "manipulator")
        if (!kdl_tree_.getChain("base_link", "tray", kdl_chain_)) {
            RCLCPP_ERROR(get_logger(), "Failed to get KDL chain");
            return;
        }

        // Initialize KDL dynamics parameters solver
        gravity_.resize(kdl_chain_.getNrOfJoints());
        dyn_param_solver_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, KDL::Vector(0, 0, -9.81));
        
        // Initialize q and q_dot
        q.resize(kdl_chain_.getNrOfJoints());
        q_dot.resize(kdl_chain_.getNrOfJoints());

        q_des.resize(kdl_chain_.getNrOfJoints());
        q_dot_des.resize(kdl_chain_.getNrOfJoints());
        q_dotdot_des.resize(kdl_chain_.getNrOfJoints());

        // Initializa desired configuration q_des
        InitializationDesiredPosition();

        // Subscribe to joint states
        joint_state_subscriber = create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, bind(&IDControllerClass::jointStateCallback, this, placeholders::_1));

        // Subscribe to desired joint positions
        desired_joints_subscriber = create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
            "reference_trajectory", 10, bind(&IDControllerClass::desiredJointPositionCallback, this, placeholders::_1));

        // Publisher for torque commands
        torque_command_publisher = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/arm_controller/commands", 10); 

        // Call functions
        auto timercallback = [this]() -> void 
        {
            ComputeControlTorque();
        };
        timer = this->create_wall_timer(chrono::milliseconds(1), timercallback);
    }

private:
    // Proportional and Derivative Gains
    double Kp; 
    double Kd; 

    rclcpp::TimerBase::SharedPtr timer;

    // KDL elements
    KDL::JntArray q_des; 
    KDL::JntArray q_dot_des; 
    KDL::JntArray q_dotdot_des; 
    KDL::JntArray q;
    KDL::JntArray q_dot;
    KDL::JntArray gravity_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    unique_ptr<KDL::ChainDynParam> dyn_param_solver_;

    // Publisher and Subscriber
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr desired_joints_subscriber;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_command_publisher;
    
    // Member Functions
    string loadURDFFromFile(const string& urdf_file_path) {
        
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

    // Private member function to initialize q_des
    void InitializationDesiredPosition() {
        
        // Declare parameters for each joint value
        declare_parameter("joint_a1", 0.0);
        declare_parameter("joint_a2", 0.0);
        declare_parameter("joint_a3", 0.0);
        declare_parameter("joint_a4", 0.0);
        declare_parameter("joint_a5", 0.0);
        declare_parameter("joint_a6", 0.0);
        declare_parameter("joint_a7", 0.0);

        // Store the parameter values in desired_joints_positions
        q_des.resize(7);
        q_des(0) = get_parameter("joint_a1").as_double();
        q_des(1) = get_parameter("joint_a2").as_double();
        q_des(2) = get_parameter("joint_a3").as_double();
        q_des(3) = get_parameter("joint_a4").as_double();
        q_des(4) = get_parameter("joint_a5").as_double();
        q_des(5) = get_parameter("joint_a6").as_double();
        q_des(6) = get_parameter("joint_a7").as_double();

        // Optionally, you can print the stored values
        RCLCPP_INFO(get_logger(), "Initial Configuration:");
        for (int i = 0; i < q_des.rows(); ++i) {
            RCLCPP_INFO(get_logger(), "Joint %d: %f", i+1, q_des(i));
        }
    }

    // Private member function to get q_des from topic /desired_position_joints
    void desiredJointPositionCallback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr desired_joints_msg) {
        
        if (desired_joints_msg->positions.size() != kdl_chain_.getNrOfJoints() ||
            desired_joints_msg->velocities.size() != kdl_chain_.getNrOfJoints() ||
            desired_joints_msg->accelerations.size() != kdl_chain_.getNrOfJoints()) {
            RCLCPP_ERROR(get_logger(), "Desired trajectory message size does not match the number of joints in the chain");
            return;
        }

        // Update desired joint positions, velocities, and accelerations
        for (size_t i = 0; i < desired_joints_msg->positions.size(); ++i) {
            q_des(i) = desired_joints_msg->positions[i];
            q_dot_des(i) = desired_joints_msg->velocities[i];
            q_dotdot_des(i) = desired_joints_msg->accelerations[i];
        }
    }
    
    // Prive member function to get q and q_dot from topic /joint_states 
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg) 
    {    
        if (joint_state_msg->position.size() != kdl_chain_.getNrOfJoints() ||
            joint_state_msg->velocity.size() != kdl_chain_.getNrOfJoints()) 
        {
            RCLCPP_ERROR(get_logger(), "Joint state message size does not match the number of joints in the chain");
            return;
        }

        for (size_t i = 0; i < joint_state_msg->position.size(); ++i) 
        {
            q(i) = joint_state_msg->position[i];
            q_dot(i) = joint_state_msg->velocity[i];
        }
    }

    void ComputeControlTorque() {
        
        // Calculate position error
        KDL::JntArray position_error(kdl_chain_.getNrOfJoints());
        KDL::JntArray velocity_error(kdl_chain_.getNrOfJoints());

        for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i) 
        {
            position_error(i) = q_des(i) - q(i);
            velocity_error(i) = q_dot_des(i) - q_dot(i);
        }

        // PD control law 
        KDL::JntArray pd_control_torques(kdl_chain_.getNrOfJoints());
        for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i) 
        {  
            pd_control_torques(i) = q_dotdot_des(i) + Kp * position_error(i) + Kd * velocity_error(i);                     
        }

        // Mass matrix
        KDL::JntSpaceInertiaMatrix M(kdl_chain_.getNrOfJoints());
        dyn_param_solver_->JntToMass(q, M);

        // Coriolis terms
        KDL::JntArray coriolis_torques(kdl_chain_.getNrOfJoints());
        dyn_param_solver_->JntToCoriolis(q, q_dot, coriolis_torques);
        
        // Compute gravity compensation torques
        KDL::JntArray gravity_comp_torques(kdl_chain_.getNrOfJoints());
        dyn_param_solver_->JntToGravity(q, gravity_comp_torques);   

        auto torque_command_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
        torque_command_msg->data.resize(kdl_chain_.getNrOfJoints());

        for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i) 
        {
            for (size_t j = 0; j < kdl_chain_.getNrOfJoints(); ++j) 
            {
                torque_command_msg->data[i] += M(i, j) * pd_control_torques(j);
            }
            torque_command_msg->data[i] += gravity_comp_torques(i) + coriolis_torques(i);

            // Saturation
            if (torque_command_msg->data[i] <= -50)
            {
                torque_command_msg->data[i] = -50;
            }
            else if (torque_command_msg->data[i] >= 50)
            {
                torque_command_msg->data[i] = 50;
            }
        }
        torque_command_publisher->publish(*torque_command_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IDControllerClass>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
