//_____________________OPERATIONAL SPACE INVERSE DYNAMICS CONTROLLER__________________________
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/msg/cartesian_point.hpp>

#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>

using namespace std;
using namespace placeholders;
using namespace chrono_literals;

class IDControllerClass : public rclcpp::Node 
{
    public:
        IDControllerClass() : Node("OpID_Controller", rclcpp::NodeOptions()) 
        { 
            // Declare parameters gains of EKF
            declare_parameter("Kp", 1.0);
            declare_parameter("Kd", 1.0);
            
            // Get gains of EKF
            Kp = get_parameter("Kp").as_double();
            Kd = get_parameter("Kd").as_double();

            cout << "CONTROLLER GAINS" << endl;
            cout << "Kp: " << Kp << endl; 
            cout << "Kd: " << Kd << endl;

            // Initialize x_e and x_e_des as geometry_msgs::msg::Pose
            x_e = moveit_msgs::msg::CartesianPoint();
            x_e_des = moveit_msgs::msg::CartesianPoint();

            // Initializa desired pose x_des
            init_x_des();

            // ______________________Initialization of KDL elements_____________________

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

            // Specify the chain of the manipulator 
            if (!kdl_tree_.getChain("base_link", "tray", kdl_chain)) {
                RCLCPP_ERROR(get_logger(), "Failed to get KDL chain");
                return;
            }

            // Save number of joints 
            NJoints = kdl_chain.getNrOfJoints();
            dt = 0.01;
            // Initializations q and q_dot
            q.resize(NJoints);
            q_dot.resize(NJoints);

            // Initialization Jacobian
            J_kdl = KDL::Jacobian(NJoints);

            // Initializations solvers
            jac_solver = std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain);
            dyn_solver = std::make_unique<KDL::ChainDynParam>(kdl_chain, KDL::Vector(0, 0, -9.81));

            //__________________________________________________________________________
            //___________________________Matries for dynamic model______________________
            I = Eigen::Matrix<double,7,7>::Identity();
            M = Eigen::MatrixXd::Zero(NJoints,NJoints);
            C = Eigen::VectorXd::Zero(NJoints);
            g = Eigen::VectorXd::Zero(NJoints);
            J = Eigen::MatrixXd::Zero(6,NJoints);
            J_transpose = Eigen::MatrixXd::Zero(NJoints,6);
            J_pse = Eigen::MatrixXd::Zero(NJoints,6);
            Jdotqdot = Eigen::VectorXd::Zero(6);

            q_dot_eig = Eigen::VectorXd::Zero(NJoints);
            x_e_dot = Eigen::VectorXd::Zero(6);
            previous_velocity = Eigen::VectorXd::Zero(6);
            xe_ddot = Eigen::VectorXd::Zero(6);
            x_tilde = Eigen::VectorXd::Zero(6); 
            x_tilde_dot = Eigen::VectorXd::Zero(6);
            ex_wrench = Eigen::VectorXd::Zero(6);
            acc_des = Eigen::VectorXd::Zero(6);
            y = Eigen::VectorXd::Zero(6);
            u = Eigen::VectorXd::Zero(NJoints);

            // _______________________________________________________________________
            // _________________________SUBSCRIBERS___________________________________ 
            // Subscriber to joint states
            joint_state_subscriber = create_subscription
                <sensor_msgs::msg::JointState>("/joint_states", 1, 
                bind(&IDControllerClass::jointStateCallback, this, _1));

            // Subscriber to desired joint positions
            desired_trajectory = create_subscription
                <moveit_msgs::msg::CartesianPoint>("/reference_trajectory", 1, 
                bind(&IDControllerClass::referenceTrajectoryCallback, this, _1));
            //___________________________________________________________________________
            // ____________________________PUBLISHER_____________________________________  
            // Publisher for arm pose
            arm_pose = this->create_publisher
                <std_msgs::msg::Float64MultiArray>("/arm_pose", 1);

            // Publisher for ee velocity
            ee_vel_publisher = this->create_publisher
                <geometry_msgs::msg::Twist>("/ee_vel",1);

            // Publisher for ee acceleration
            ee_accel_publisher = this->create_publisher
                <geometry_msgs::msg::Accel>("/ee_accel",1);

            // Publisher to desired pose for visualize in RViz
            TrayFrame_Publisher = this->create_publisher
                <geometry_msgs::msg::PoseStamped>("/TrayPose_Frame", 1);

            // Publisher for arm pose error
            arm_pose_error = this->create_publisher
                <std_msgs::msg::Float64MultiArray>("/arm_pose_error", 1);

            // Publisher for torque commands
            torque_command_publisher = create_publisher
                <std_msgs::msg::Float64MultiArray>("/arm_controller/commands", 1); 
            //_____________________________________________________________________________
            //________________________________Control Loop___________________________________
            auto timercallback = [this]() -> void 
            {
                //get_xe();
                publishPose ();
                ComputeControlTorque();
                show_current_pose();
            };
            timer = this->create_wall_timer(chrono::milliseconds(1), timercallback);
        }

    private:
        int NJoints;

        // Proportional and Derivative Gains
        double dt;
        double Kp; 
        double Kd; 

        // Elements to manage time
        rclcpp::TimerBase::SharedPtr timer;
        std::chrono::steady_clock::time_point last_call_time;

        // Current and desired pose
        moveit_msgs::msg::CartesianPoint x_e;
        moveit_msgs::msg::CartesianPoint x_e_des;    

        // KDL elements
        KDL::Tree kdl_tree_;
        KDL::Chain kdl_chain;
        KDL::JntArray q;
        KDL::JntArray q_dot;
        KDL::Jacobian J_kdl;
        unique_ptr<KDL::ChainJntToJacSolver> jac_solver;
        unique_ptr<KDL::ChainDynParam> dyn_solver;

        // Matries and vectors for dynamic model
        Eigen::MatrixXd I;
        Eigen::MatrixXd M;
        Eigen::VectorXd C;
        Eigen::VectorXd g;
        Eigen::MatrixXd J;
        Eigen::MatrixXd J_transpose;
        Eigen::MatrixXd J_pse;
        Eigen::VectorXd Jdotqdot; 

        Eigen::VectorXd q_dot_eig;
        Eigen::VectorXd x_e_dot;
        Eigen::VectorXd previous_velocity;
        Eigen::VectorXd xe_ddot;

        Eigen::VectorXd x_tilde; 
        Eigen::VectorXd x_tilde_dot;

        Eigen::VectorXd acc_des;
        Eigen::VectorXd ex_wrench;
        Eigen::VectorXd y;
        Eigen::VectorXd u;

        // Publishers and Subscribers
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber;
        rclcpp::Subscription<moveit_msgs::msg::CartesianPoint>::SharedPtr desired_trajectory;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pose_error;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pose;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ee_vel_publisher;
        rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr ee_accel_publisher;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr TrayFrame_Publisher;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_command_publisher;

        // Function to read urdf file
        string loadURDFFromFile(const string& urdf_file_path) 
        {
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

        // Define the skew function inline
        inline Eigen::Matrix3d skew(const Eigen::Vector3d & t)
        {
            Eigen::Matrix3d t_hat;

            t_hat <<     0, -t[2],  t[1],
                    t[2],     0, -t[0],
                    -t[1],  t[0],     0;

            return t_hat;
        }

        // Function to convert kdl joint array in eigen vector
        inline Eigen::VectorXd kdl_to_eigen (const KDL::JntArray& vector_kdl)
        {
            Eigen::VectorXd vector(vector_kdl.data);
            return vector;
        }
        
        // Function to initialize x_des
        void init_x_des() 
        {
            // Declare parameters for initial pose
            declare_parameter("position_x", 0.0);
            declare_parameter("position_y", 0.0);
            declare_parameter("position_z", 0.0);
            declare_parameter("orientation_x", 0.0);
            declare_parameter("orientation_y", 0.0);
            declare_parameter("orientation_z", 0.0);
            declare_parameter("orientation_w", 1.0);

            // Get initial pose parameters
            x_e_des.pose.position.x      = get_parameter("position_x").as_double();
            x_e_des.pose.position.y      = get_parameter("position_y").as_double();
            x_e_des.pose.position.z      = get_parameter("position_z").as_double();
            x_e_des.pose.orientation.x   = get_parameter("orientation_x").as_double();
            x_e_des.pose.orientation.y   = get_parameter("orientation_y").as_double();
            x_e_des.pose.orientation.z   = get_parameter("orientation_z").as_double();
            x_e_des.pose.orientation.w   = get_parameter("orientation_w").as_double();

            cout << "\033[1;32mInitial Position: \033[0m"       
                    << x_e_des.pose.position.x << " " 
                    << x_e_des.pose.position.y << " "
                    << x_e_des.pose.position.z << endl;
            cout << "\033[1;32mInitial Orientation: \033[0m"    
                    << x_e_des.pose.orientation.x << " " 
                    << x_e_des.pose.orientation.y << " "
                    << x_e_des.pose.orientation.z << " "
                    << x_e_des.pose.orientation.w << endl;                                 
        }
        
        // Function to publish arm pose and arm pose frame for RVIZ 
        void publishPose ()
        {
            tf2::Quaternion q(
                x_e.pose.orientation.x,
                x_e.pose.orientation.y,
                x_e.pose.orientation.z,
                x_e.pose.orientation.w
            );

            double roll, pitch, yaw;
            tf2::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);

            // Publish end-effector pose (RPY)
            auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
            msg->data.resize(6);
            msg->data[0] = x_e.pose.position.x;
            msg->data[1] = x_e.pose.position.y;
            msg->data[2] = x_e.pose.position.z;
            msg->data[3] = roll;
            msg->data[4] = pitch;
            msg->data[5] = yaw;
            arm_pose->publish(std::move(msg));

            // Publish end-effector pose (Quat) as 3D Frame for Rviz
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.frame_id = "world";
            pose_msg.header.stamp = this->now();
            pose_msg.pose = x_e.pose;
            TrayFrame_Publisher->publish(pose_msg);
            
            // Publish end-effector velocity
            auto ee_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
            ee_vel_msg->linear.x = x_e_dot[0];
            ee_vel_msg->linear.y = x_e_dot[1];
            ee_vel_msg->linear.z = x_e_dot[2];
            ee_vel_msg->angular.x = x_e_dot[3];
            ee_vel_msg->angular.y = x_e_dot[4];
            ee_vel_msg->angular.z = x_e_dot[5];
            ee_vel_publisher->publish(std::move(ee_vel_msg));

            // Publish end-effector accelleration
            auto ee_accel_msg = std::make_unique<geometry_msgs::msg::Accel>();
            ee_accel_msg->linear.x = xe_ddot[0];
            ee_accel_msg->linear.y = xe_ddot[1];
            ee_accel_msg->linear.z = xe_ddot[2];
            ee_accel_msg->angular.x = xe_ddot[3];
            ee_accel_msg->angular.y = xe_ddot[4];
            ee_accel_msg->angular.z = xe_ddot[5];
            ee_accel_publisher->publish(std::move(ee_accel_msg));
        }

        // Function to get current pose x_e of end-effector
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg) 
        {    
            if (joint_state_msg->position.size() != NJoints ||
                joint_state_msg->velocity.size() != NJoints) 
            {
                RCLCPP_ERROR(get_logger(), "Joint state message size does not match the number of joints in the chain");
                return;
            }

            for (size_t i = 0; i < joint_state_msg->position.size(); ++i) 
            {
                q(i) = joint_state_msg->position[i];
                q_dot(i) = joint_state_msg->velocity[i];
            }
            q_dot_eig = kdl_to_eigen(q_dot);
            
            // Forward Kinematics
            KDL::Frame frame;            
            KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
            fk_solver.JntToCart(q, frame); 

            x_e.pose.position.x = frame.p.x();
            x_e.pose.position.y = frame.p.y();
            x_e.pose.position.z = frame.p.z();

            KDL::Rotation rotation = frame.M;
            // Convert rotation matrix to quaternion
            rotation.GetQuaternion(
                x_e.pose.orientation.x,
                x_e.pose.orientation.y,
                x_e.pose.orientation.z,
                x_e.pose.orientation.w
            );

            M = compute_mass_matrix();
            C = compute_coriolis ();
            g = compute_gravity ();
            J = compute_jacobian ();
            J_transpose = compute_jacobian_transpose ();
            J_pse = compute_jacobian_pseudoinverse ();
            Jdotqdot = compute_Jdot_cross_qdot ();

            // Compute end effector velocity x_e_dot
            x_e_dot = J * q_dot_eig;
                        
            xe_ddot[0] = (x_e_dot[0] - previous_velocity[0]) / dt;
            xe_ddot[1] = (x_e_dot[1] - previous_velocity[1]) / dt;
            xe_ddot[2] = (x_e_dot[2] - previous_velocity[2]) / dt;
            xe_ddot[3] = (x_e_dot[3] - previous_velocity[3]) / dt;
            xe_ddot[4] = (x_e_dot[4] - previous_velocity[4]) / dt;
            xe_ddot[5] = (x_e_dot[5] - previous_velocity[5]) / dt;

            previous_velocity = x_e_dot;
        }

        // Function to get desired pose x_e_des of end-effector
        void referenceTrajectoryCallback (const moveit_msgs::msg::CartesianPoint::SharedPtr traj_msg)
        {
            vector<double> buff (7);
            buff[0] = traj_msg->pose.position.x;
        
            if (!buff.empty()) 
            {
                x_e_des.pose         = traj_msg->pose;
                x_e_des.velocity     = traj_msg->velocity;
                x_e_des.acceleration = traj_msg->acceleration;
            }
        }

        // Function to get wrench measurements from f/t sensor
        void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench_msg) 
        {    
            ex_wrench[0] = wrench_msg->wrench.force.x;
            ex_wrench[1] = wrench_msg->wrench.force.y;
            ex_wrench[2] = wrench_msg->wrench.force.z;
            ex_wrench[3] = wrench_msg->wrench.torque.x;
            ex_wrench[4] = wrench_msg->wrench.torque.y;
            ex_wrench[5] = wrench_msg->wrench.torque.z;
        }

        Eigen::MatrixXd compute_mass_matrix ()
        {
            KDL::JntSpaceInertiaMatrix M_kdl(NJoints);
            dyn_solver->JntToMass(q, M_kdl);
            Eigen::MatrixXd M(M_kdl.data);
            return M;
        }

        Eigen::VectorXd compute_coriolis ()
        {
            KDL::JntArray C_kdl(NJoints);
            dyn_solver->JntToCoriolis(q, q_dot, C_kdl);
            Eigen::VectorXd C(C_kdl.data);
            return C;
        }

        Eigen::VectorXd compute_gravity ()
        {
            // Compute gravity compensation torques
            KDL::JntArray g_kdl(NJoints);
            dyn_solver->JntToGravity(q, g_kdl); 
            Eigen::VectorXd g(g_kdl.data);
            return g;
        }

        Eigen::MatrixXd compute_jacobian ()
        {
            // Compute Jacobian J
            KDL::Jacobian J_kdl(NJoints);
            jac_solver->JntToJac(q, J_kdl);
            Eigen::MatrixXd J(J_kdl.data);
            return J;
        }

        Eigen::MatrixXd compute_jacobian_transpose ()
        {
            // Compute Jacobian transpose J^T
            Eigen::MatrixXd J_transpose = J.transpose();
            return J_transpose;
        }

        Eigen::MatrixXd compute_jacobian_pseudoinverse ()
        {
            Eigen::MatrixXd J_pse = J_transpose * (J * J_transpose).inverse();
            return J_pse;
        }
        
        Eigen::VectorXd compute_Jdot_cross_qdot ()
        {
            // Compute Jdot
            KDL::JntArrayVel joint_vel(q, q_dot);
            KDL::Twist Jdotqdot_kdl;
            KDL::ChainJntToJacDotSolver jac_dot_solver(kdl_chain);
            jac_dot_solver.JntToJacDot(joint_vel,Jdotqdot_kdl);
            
            Eigen::VectorXd Jdotqdot(6);
            Jdotqdot[0] = Jdotqdot_kdl.vel.data[0];
            Jdotqdot[1] = Jdotqdot_kdl.vel.data[1];
            Jdotqdot[2] = Jdotqdot_kdl.vel.data[2];
            Jdotqdot[3] = Jdotqdot_kdl.rot.data[0];
            Jdotqdot[4] = Jdotqdot_kdl.rot.data[1];
            Jdotqdot[5] = Jdotqdot_kdl.rot.data[2];
            return Jdotqdot;
        }

        Eigen::VectorXd compute_external_wrench ()
        {
            Eigen::VectorXd tau_ext = J_transpose * ex_wrench;
            return tau_ext;
        }

        Eigen::VectorXd compute_pose_error ()
        {
            // Convert quaternions to rotation matrices
            tf2::Quaternion quat_e, quat_e_des;
            tf2::fromMsg(x_e.pose.orientation, quat_e);
            tf2::fromMsg(x_e_des.pose.orientation, quat_e_des);

            double w_d = x_e.pose.orientation.w;
            double w_e = x_e_des.pose.orientation.w;
            tf2::Vector3 v_d(x_e_des.pose.orientation.x, x_e_des.pose.orientation.y, x_e_des.pose.orientation.z);
            tf2::Vector3 v_e(x_e.pose.orientation.x, x_e.pose.orientation.y, x_e.pose.orientation.z);

            // Compute skew-symmetric matrix
            tf2::Matrix3x3 skew_d(0, -v_d.z(), v_d.y(),
                                v_d.z(), 0, -v_d.x(),
                                -v_d.y(), v_d.x(), 0);

            // Compute the orientation error
            tf2::Vector3 orientation_error = w_e * v_d - w_d * v_e - skew_d * v_e;

            Eigen::VectorXd pose_error(6);
            pose_error[0] = x_e_des.pose.position.x - x_e.pose.position.x;
            pose_error[1] = x_e_des.pose.position.y - x_e.pose.position.y;
            pose_error[2] = x_e_des.pose.position.z - x_e.pose.position.z;
            pose_error[3] = orientation_error[0];
            pose_error[4] = orientation_error[1];
            pose_error[5] = orientation_error[2];

            // Publish arm pose error
            auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
            msg->data.resize(6);
            for (size_t i = 0; i < 6; ++i)
            {
                msg->data[i] = abs(pose_error[i]);
            }
            arm_pose_error->publish(std::move(msg));

            return pose_error;
        }

        Eigen::VectorXd compute_velocity_error()
        {
            Eigen::VectorXd vel_error(6);
            vel_error[0] = x_e_des.velocity.linear.x - x_e_dot[0];
            vel_error[1] = x_e_des.velocity.linear.y - x_e_dot[1];
            vel_error[2] = x_e_des.velocity.linear.z - x_e_dot[2];
            vel_error[3] = x_e_des.velocity.angular.x - x_e_dot[3];
            vel_error[4] = x_e_des.velocity.angular.y - x_e_dot[4];
            vel_error[5] = x_e_des.velocity.angular.z - x_e_dot[5];
            return vel_error;
        }

        // Function to compute torque commands and publish them
        void ComputeControlTorque() 
        {
            // Position error
            x_tilde = compute_pose_error(); 
            // Velocity error
            x_tilde_dot = compute_velocity_error();
            
            // Desired accelleration
            acc_des[0] = x_e_des.acceleration.linear.x;
            acc_des[1] = x_e_des.acceleration.linear.y;
            acc_des[2] = x_e_des.acceleration.linear.z;
            acc_des[3] = x_e_des.acceleration.angular.x;
            acc_des[4] = x_e_des.acceleration.angular.y;
            acc_des[5] = x_e_des.acceleration.angular.z;

            // Control input y
            y = acc_des + Kd*x_tilde_dot + Kp*x_tilde - Jdotqdot;
            
            // Inverse dynamics contro law u
            u = M*((J_pse*y)+(I-J_pse*J)*q_dot_eig) + C + g;
            
            // Compute and publish control law
            auto torque_command_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
            torque_command_msg->data.resize(NJoints);
            for (int i=0; i<NJoints; i++)
                torque_command_msg->data[i] = u[i];
            torque_command_publisher->publish(*torque_command_msg);
        }

        // Show in terminal current pose of end-effector and pose error (update each 3 seconds)
        void show_current_pose()
        {
            // Get current time
            auto now = std::chrono::steady_clock::now(); 

            if (now - last_call_time > std::chrono::seconds(3))
            {
                // Convert Quaternion to RPY
                double roll, pitch, yaw;
                tf2::Quaternion quaternion;
                quaternion.setX(x_e.pose.orientation.x);
                quaternion.setY(x_e.pose.orientation.y);
                quaternion.setZ(x_e.pose.orientation.z);
                quaternion.setW(x_e.pose.orientation.w);
                tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);  

                // Show only three significant numerical digits
                std::cout << std::fixed << std::setprecision(3); 
                cout << "CURRENT POSE" << endl;
                cout << "\033[34mCurrent Position:    \033[0m" 
                    << x_e.pose.position.x << " " 
                    << x_e.pose.position.y << " "
                    << x_e.pose.position.z << endl;
                cout << "\033[34mCurrent Orientation: \033[0m"  
                    << roll * 180 / M_PI << " " 
                    << pitch * 180 / M_PI << " "
                    << yaw * 180 / M_PI << endl; 
                cout << "POSE ERROR" << endl;
                cout << "\033[38;5;208mPosition Errors:    \033[0m"         
                    << x_tilde[0] << " "
                    << x_tilde[1] << " "
                    << x_tilde[2] << endl;
                cout << "\033[38;5;208mOrientation Errors: \033[0m"      
                    << x_tilde[3] << " "
                    << x_tilde[4] << " "
                    << x_tilde[5] << endl; 
                
                // Update the last call time to the current time
                last_call_time = now;
            }
        }
};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IDControllerClass>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}