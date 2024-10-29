//______________SENSOR-BASED EXTENDED KALMAN FILTER______________________ 
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <chrono>

using namespace std;
using namespace placeholders;

class SBExtendedKalmanFilter : public rclcpp::Node
{
    public:
        SBExtendedKalmanFilter() : Node("ekf_without_model")
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


            x_hat       = Eigen::VectorXd::Zero(2);
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
            // Subscriber for ft sensor measurement 
            subscriber_ft_pose = this->create_subscription
                <geometry_msgs::msg::Point>("/ftsensor_objpose", 10,
                std::bind(&SBExtendedKalmanFilter::ftsensorCallback, this, _1));

            // Subscriber for camera measurements
            subscriber_camera_pose = this->create_subscription
                <geometry_msgs::msg::Point>("/camera_objpose", 10,
                std::bind(&SBExtendedKalmanFilter::cameraCallback, this, _1));
            // _______________________________________________________________________
            // ____________________________PUBLISHER__________________________________      
            // Publisher for estimated object pose
            publisher_obj_pose = this->create_publisher
                <geometry_msgs::msg::Point>("/estimated_objpose", 10);
            // _______________________________________________________________________

            // Call functions
            auto timercallback = [this]() -> void {
                prediction();
                correction();
                publishStateVector();
            };
            timer = this->create_wall_timer(std::chrono::milliseconds(1), timercallback);
        }

    private:
        // Gains matries EKF
        double Kq;
        double Kr;
        double Kp;        
               
        // Object Pose
        Eigen::VectorXd x_hat;
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
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscriber_ft_pose;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscriber_camera_pose;
        
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_obj_pose;
        
        // Function to obtain object pose from the force sensor
        void ftsensorCallback(const geometry_msgs::msg::Point::SharedPtr ft_sensor_msg)
        {
            objpose_ft[0] = ft_sensor_msg->x;
            objpose_ft[1] = ft_sensor_msg->y;
        }

        // Function to obtain object pose from the camera
        void cameraCallback(const geometry_msgs::msg::Point::SharedPtr camera_sensor_msg)
        {
            objpose_cam[0] = camera_sensor_msg->x;
            objpose_cam[1] = camera_sensor_msg->y;
        }

        // Prediction step of EKF
        void prediction()
        {
            // Previous state
            x_hat(0) = objpose_ft[0];
            x_hat(1) = objpose_ft[1];

            // Prediction next state
            x_hat = F * x_hat;
            // Modify P
            P = F * P * F.transpose() + Q;
        }

        // Correction step of EKF
        void correction()
        {
            // Obtain current time for understand when the object fall
            auto now = std::chrono::system_clock::now();
            auto now_c = std::chrono::system_clock::to_time_t(now);

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

                // Calculate Kalman gain
                K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

                z_pred = H*x_hat;

                x_hat = x_hat + K * (z_actual-z_pred);

                // Update covariance matrix
                P = (I - K * H) * P;
            }
            // Else both sensors provide invalid data provide a error message
            else
            {
                sensors_state(ft_sensor_valid,camera_sensor_valid); 
            }
        }

        // Show in terminal state of sensors and also presence of object on the tray
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
                }
                // Camera with invalid data
                else if (ft_sensor_valid==true && camera_sensor_valid==false) 
                {
                    cout << "\033[1;33m- Object on the tray\033[0m" << std::endl; // Yellow color
                    cout << "\033[1;32mForce/Torque Sensor - Avaiable\033[0m" << endl;
                    cout << "\033[1;31mCamera Sensor - Not Avaiable\033[0m" << endl;  
                }
                // Force sensor with invalid data
                else if (ft_sensor_valid==false && camera_sensor_valid==true) 
                {
                    cout << "\033[1;33m- Object on the tray\033[0m" << std::endl; // Yellow color
                    cout << "\033[1;31mForce/Torque Sensor - Not Avaiable\033[0m" << endl;
                    cout << "\033[1;32mCamera Sensor - Avaiable\033[0m" << endl;  
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
            // Publish estimated state
            geometry_msgs::msg::Point estimated_state_msg;
            estimated_state_msg.x = x_hat(0);
            estimated_state_msg.y = x_hat(1);
            publisher_obj_pose->publish(estimated_state_msg);
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SBExtendedKalmanFilter>());
    rclcpp::shutdown();
    return 0;
}