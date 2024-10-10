#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <fstream>
#include <vector>
#include <string>

class DataCollector : public rclcpp::Node
{
    public:
        DataCollector() : Node("real_data_monitor"), start_time(this->now())
        {
            ee_pose_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("ee_pose", 10, std::bind(&DataCollector::ee_pose_callback, this, std::placeholders::_1));
            ee_vel_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("ee_vel", 10, std::bind(&DataCollector::ee_vel_callback, this, std::placeholders::_1));
            ftsensor_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("ftsensor_data", 10, std::bind(&DataCollector::ftsensor_callback, this, std::placeholders::_1));
            camera_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("camera_data", 10, std::bind(&DataCollector::camera_callback, this, std::placeholders::_1));

            std::string base_path = "/home/" + 
                                    std::string(getenv("USER")) + 
                                    "/noprehensileman_ws/src/data/Experiments/center20/";

            ee_pose_file_.open(base_path + "ee_pose.csv");
            ee_vel_file_.open(base_path + "ee_vel.csv");
            ftsensor_file_.open(base_path + "ftsensor.csv");
            camera_file_.open(base_path + "camera.csv");

            ee_pose_file_ << "Timestamp,x,y,z,R,P,Y\n";
            ee_vel_file_ << "Timestamp,vx,vy,vz,wx,wy,wz\n";
            ftsensor_file_ << "Timestamp,fx,fy,fz,mx,my,mz\n";
            camera_file_ << "Timestamp,x,y,z,q1,q2,q3,q4\n";
        }

    private:

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ee_pose_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ee_vel_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ftsensor_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr camera_subscriber_;

        rclcpp::Time start_time;

        static std::ofstream ee_pose_file_;
        static std::ofstream ee_vel_file_;
        static std::ofstream ftsensor_file_;
        static std::ofstream camera_file_;

        void ee_pose_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            write_to_file(ee_pose_file_, msg);
        }

        void ee_vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            write_to_file(ee_vel_file_, msg);
        }

        void ftsensor_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            write_to_file(ftsensor_file_, msg);
        }

        void camera_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            write_to_file(camera_file_, msg);
        }

        void write_to_file(std::ofstream &file, const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            auto elapsed_time = this->now() - start_time;
            file << elapsed_time.seconds();
            for (const auto &value : msg->data)
            {
                file << "," << value;
            }
            file << "\n";
        }

        static void signalHandler(int signum) 
        {
            if (signum == SIGINT) 
            {
                ee_pose_file_.close();
                ee_vel_file_.close();
                ftsensor_file_.close();
                camera_file_.close();
                
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received SIGINT signal. Closing CSV files and shutting down node.");
                rclcpp::shutdown();
            }
        }
};

std::ofstream DataCollector::ee_pose_file_;
std::ofstream DataCollector::ee_vel_file_;
std::ofstream DataCollector::ftsensor_file_;
std::ofstream DataCollector::camera_file_;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataCollector>());
    rclcpp::shutdown();
    return 0;
}
