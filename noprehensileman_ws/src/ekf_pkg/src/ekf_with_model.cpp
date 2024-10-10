//______________MODEL-BASED EXTENDED KALMAN FILTER______________________ 
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <Eigen/Dense>
#include <cmath> // Per utilizzare le funzioni pow() ed exp()
#include <iostream>

using namespace std;
using namespace placeholders;

class MBExtendedKalmanFilter : public rclcpp::Node 
{
    public:
        MBExtendedKalmanFilter() : Node("ekf_with_model")
        {
            // Declare, save and print the values ​​of the EKF gains
            declare_parameter("Kq", 1.0);
            declare_parameter("Kr", 1.0);
            declare_parameter("Kp", 1.0);
            Kq = get_parameter("Kq").as_double();
            Kr = get_parameter("Kr").as_double();
            Kp = get_parameter("Kp").as_double();
            cout << "EKF GAINS" << endl;
            cout << "Kq: " << Kq << endl;
            cout << "Kr: " << Kr << endl;
            cout << "Kp: " << Kp << endl; 

            // ____________________Model Parameters________________________
            dt = 0.01;
            mass_object = 1.0;
            g = 9.81;
            vt=0.5;
            vsp=2*vt;
            // Coulomb friction amplitude
            Fc_bar = 0.25*mass_object*g;
            // Viscous friction amplitude
            Fv_bar = 0.1;
            //  Stribeck friction amplitude;
            Fshat = 0.7;
            Fs_bar = Fshat-Fc_bar*tanh(vsp/vt)-Fv_bar*vsp;
            // _______________________________________________________________________

            ee_acc      = Eigen::VectorXd::Zero(6);
            x_hat       = Eigen::VectorXd::Zero(2);
            obj_vel     = Eigen::VectorXd::Zero(2);
            objpose_ft  = Eigen::VectorXd::Zero(2);
            objpose_cam = Eigen::VectorXd::Zero(2);
            z_actual    = Eigen::VectorXd::Zero(2);
            z_pred      = Eigen::VectorXd::Zero(2);

            I =      Eigen::MatrixXd::Identity(2, 2);
            Q = Kq * Eigen::MatrixXd::Identity(2, 2);
            R = Kr * Eigen::MatrixXd::Identity(2, 2);
            P = Kp * Eigen::MatrixXd::Identity(2, 2);
            F =      Eigen::MatrixXd::Identity(2, 2);
            H =      Eigen::MatrixXd::Identity(2, 2);
            K =      Eigen::MatrixXd::Identity(2, 2);

            // _______________________________________________________________________
            // _________________________SUBSCRIBERS___________________________________      
            // Subscriber for obtain mass of object computed by force sensor
            mass_subscriber = this->create_subscription
                <std_msgs::msg::Float64>("/mass_object", 1, 
                std::bind(&MBExtendedKalmanFilter::massCallback, this, _1));
            
            /* De-comment if you want mass of object by ROS paramter
            // Subscriber to obtain mass of object through ROS parameter
            this->declare_parameter<double>("m_obj", 1.0);
            parameter_subscriber_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
              "/parameter_events", 10, std::bind(&MBExtendedKalmanFilter::parameterCallback, this, std::placeholders::_1));
            */  

            // Subscriber to obtain end-effector acceleration
            ee_accel_subscriber = this->create_subscription
                <geometry_msgs::msg::Accel>("/ee_accel", 1, 
                std::bind(&MBExtendedKalmanFilter::eeaccelCallback, this, _1));

            // Subscriber to obtain object pose from the force sensor
            ftsensor_subscriber = this->create_subscription
                <geometry_msgs::msg::Point>("/ftsensor_objpose", 1, 
                std::bind(&MBExtendedKalmanFilter::ftsensorCallback, this, _1));
            
            // Subscriber to obtain object pose from the camera
            camera_pose_subscriber = this->create_subscription
                <geometry_msgs::msg::Point>("/camera_objpose", 1, 
                std::bind(&MBExtendedKalmanFilter::cameraPoseCallback, this, _1));
            //___________________________________________________________________________
            // ____________________________PUBLISHER_____________________________________      
            // Publisher to publish estimated pose
            estimated_pose_publisher = this->create_publisher
                <geometry_msgs::msg::Point>("/estimated_objpose", 1);
            //___________________________________________________________________________

            // Call member functions
            auto timercallback = [this]() -> void {
                prediction();
                correction();
                publishStateVector();
            };
            timer = this->create_wall_timer(std::chrono::milliseconds(1), timercallback);
        }

    private:
        // EKF Gains
        double Kq;
        double Kr;
        double Kp;

        // Model Parameters
        double dt;
        double mass_object;
        double g;
        double vt;
        double vsp;
        double Fc_bar;
        double Fv_bar;
        double Fshat;
        double Fs_bar;

        // Tray Acceleration
        Eigen::VectorXd ee_acc;
        // Object Pose
        Eigen::VectorXd x_hat;
        // Object velocity
        Eigen::VectorXd obj_vel; 
        // Object Pose from the force sensor
        Eigen::VectorXd objpose_ft;
        // Object Pose from the camera
        Eigen::VectorXd objpose_cam;
        // Object Pose as a weighted average of sensory data
        Eigen::VectorXd z_actual;
        // Object Pose predicted
        Eigen::VectorXd z_pred;

        // Matries of EKF
        Eigen::MatrixXd I;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;
        Eigen::MatrixXd P;
        Eigen::MatrixXd F;
        Eigen::MatrixXd H;
        Eigen::MatrixXd K;

        rclcpp::TimerBase::SharedPtr timer;
        std::chrono::steady_clock::time_point last_call_time;

        // Subscribers and Publisher
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr mass_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Accel>::SharedPtr ee_accel_subscriber;
        rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ftsensor_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr camera_pose_subscriber;
        
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr estimated_pose_publisher;

        /*
        // Function to obtain object mass as ROS parameters
        void parameterCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
        {
            for (const auto &p : event->changed_parameters) {
                if (p.name == "m_obj") {
                    mass_object = p.value.double_value;
                }
            }
        }
        */
        // Function to obtain tray velocity
        void eeaccelCallback (const geometry_msgs::msg::Accel::SharedPtr msg)
        {
            ee_acc[0] = msg->linear.x;
            ee_acc[1] = msg->linear.y;
            ee_acc[2] = msg->linear.z;
            ee_acc[3] = msg->angular.x;
            ee_acc[4] = msg->angular.y;
            ee_acc[5] = msg->angular.z;
        }

        // Function to obtain mass of object from topic
        void massCallback (const std_msgs::msg::Float64::SharedPtr msg)
        {
            // Verifica se il valore non è NaN e se è maggiore di una soglia piccola
            if (!std::isnan(msg->data) && msg->data > 1e-3)
            {
                mass_object = msg->data;
            }
        }

        // Function to obtain object pose from the force sensor
        void ftsensorCallback(const geometry_msgs::msg::Point::SharedPtr msg) 
        {
            objpose_ft[0] = msg->x;
            objpose_ft[1] = msg->y;
        }

        // Function to obtain object pose from the camera
        void cameraPoseCallback(const geometry_msgs::msg::Point::SharedPtr msg) 
        {
            objpose_cam[0] = msg->x;
            objpose_cam[1] = msg->y;
        }

        // Prediction step of EKF
        void prediction() 
        {
            // Update coulomb and stribeck friction amplitude
            Fc_bar = 0.25*mass_object*g;
            Fs_bar = Fshat-Fc_bar*tanh(vsp/vt)-Fv_bar*vsp;

            double viscous_friction_x = - Fv_bar * obj_vel[0]; 
            double viscous_friction_y = - Fv_bar * obj_vel[1]; 

            double coulomb_friction_x = - Fc_bar * atanh(obj_vel[0]); 
            double coulomb_friction_y = - Fc_bar * atanh(obj_vel[1]); 

            double stribeck_friction_x = - Fs_bar * (obj_vel[0] / vsp) * exp(-pow((obj_vel[0] / (sqrt(2) * vsp)), 2) + 0.5);
            double stribeck_friction_y = - Fs_bar * (obj_vel[1] / vsp) * exp(-pow((obj_vel[1] / (sqrt(2) * vsp)), 2) + 0.5);

            double total_friction_x = viscous_friction_x + coulomb_friction_x + stribeck_friction_x;
            double total_friction_y = viscous_friction_y + coulomb_friction_y + stribeck_friction_y;

            double Fext_x = mass_object * ee_acc[0];
            double Fext_y = mass_object * ee_acc[1];

            double total_force_x = total_friction_x + Fext_x;
            double total_force_y = total_friction_y + Fext_y;

            double acceleration_x = total_force_x / 0.234;
            double acceleration_y = total_force_y / 0.234;

            obj_vel[0] += acceleration_x * dt;
            obj_vel[1] += acceleration_y * dt;

            x_hat[0] += obj_vel[0] * dt;
            x_hat[1] += obj_vel[1] * dt;

            P = F * P * F.transpose() + Q;
        }

        // Correction step of EKF
        void correction() 
        {
            bool ft_sensor_valid = !std::isnan(objpose_ft[0]) || !std::isnan(objpose_ft[1]);
            bool camera_sensor_valid = !std::isnan(objpose_cam[0]) || !std::isnan(objpose_cam[1]);
            
            // If a least one of sensor provide a valid data continue normally
            if (ft_sensor_valid==true || camera_sensor_valid==true)
            {
                // Both sensors have valid data
                if (ft_sensor_valid==true && camera_sensor_valid==true)
                    z_actual = (objpose_ft + objpose_cam) / 2.0;
                
                // Camera with invalid data
                else if (ft_sensor_valid==true && camera_sensor_valid==false) 
                    z_actual = objpose_ft;
                
                // Force sensor with invalid data
                else if (ft_sensor_valid==false && camera_sensor_valid==true) 
                    z_actual = objpose_cam;
                               
                sensors_state(ft_sensor_valid,camera_sensor_valid);

                K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
                z_pred = H * x_hat;
                x_hat = x_hat + K * (z_actual - z_pred);
                P = ( I - K * H ) * P;
            }
            // Else both sensors provide invalid data provide a error message
            else
                sensors_state(ft_sensor_valid,camera_sensor_valid);
            
        }

        // Show in terminal state of sensors and presence of object on the tray
        void sensors_state(bool ft_sensor_valid, bool camera_sensor_valid)
        {
            // Get current time
            auto now = std::chrono::steady_clock::now(); 

            if (now - last_call_time > std::chrono::seconds(1))
            {
                                // Both sensors have valid data
                if (ft_sensor_valid==true && camera_sensor_valid==true)
                {
                    cout << "\033[1;32m- Object on the tray\033[0m" << endl;
                    cout << "\033[1;32mForce/Torque Sensor - Avaiable\033[0m" << endl;
                    cout << "\033[1;32mCamera Sensor - Avaiable\033[0m" << endl;                   
                    cout << "Object Mass " << mass_object << std::endl;
                }
                // Camera with invalid data
                else if (ft_sensor_valid==true && camera_sensor_valid==false) 
                {
                    cout << "\033[1;33m- Object on the tray\033[0m" << std::endl; // Yellow color
                    cout << "\033[1;32mForce/Torque Sensor - Avaiable\033[0m" << endl;
                    cout << "\033[1;31mCamera Sensor - Not Avaiable\033[0m" << endl;  
                    cout << "Object Mass " << mass_object << std::endl;
                }
                // Force sensor with invalid data
                else if (ft_sensor_valid==false && camera_sensor_valid==true) 
                {
                    cout << "\033[1;33m- Object on the tray\033[0m" << std::endl; // Yellow color
                    cout << "\033[1;31mForce/Torque Sensor - Not Avaiable\033[0m" << endl;
                    cout << "\033[1;32mCamera Sensor - Avaiable\033[0m" << endl;  
                    cout << "Object Mass " << mass_object << std::endl;
                }
                else
                {
                    cout << "\033[1;31m- Object fall" << endl;
                    cout << "\033[1;31mForce/Torque Sensor - Not Avaiable\033[0m" << endl;
                    cout << "\033[1;31mCamera Sensor - Not Avaiable\033[0m" << endl;  
                }
                
                // Update the last call time to the current time
                last_call_time = now;
            }
        }

        // Function to publish estimated object pose
        void publishStateVector() 
        {
            geometry_msgs::msg::Point estimated_pose;
            estimated_pose.x = x_hat[0];
            estimated_pose.y = x_hat[1];
            estimated_pose_publisher->publish(estimated_pose);
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MBExtendedKalmanFilter>());
    rclcpp::shutdown();
    return 0;
}