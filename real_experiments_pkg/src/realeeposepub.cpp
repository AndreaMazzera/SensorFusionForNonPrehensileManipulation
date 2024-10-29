#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

class EePoseAndVelPublisher : public rclcpp::Node
{
public:
    EePoseAndVelPublisher() : Node("ee_pose_and_vel_publisher"), ee_pose_idx(0), ee_vel_idx(0), finished(false)
    {
        ee_pose_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("ee_pose", 10);
        ee_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("ee_vel", 10);
    
        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5), 
        std::bind(&EePoseAndVelPublisher::timer_callback, this));

        std::string base_path = "/home/" + 
                                std::string(getenv("USER")) + 
                                "/noprehensileman_ws/src/data/Experiments/center20/";
        load_data_from_file(base_path + "ee_pose_raw.txt", ee_pose_data_);
        load_data_from_file(base_path + "ee_vel_raw.txt", ee_vel_data_);
    }

private:
    void load_data_from_file(const std::string &filename, std::vector<std::vector<double>> &data)
    {
        std::ifstream file(filename);
        std::string line;
        while (std::getline(file, line))
        {
            std::vector<double> row;
            std::stringstream ss(line);
            double value;
            while (ss >> value)
            {
                row.push_back(value);
            }
            data.push_back(row);
        }
    }

void timer_callback()
{
    if (ee_pose_idx < ee_pose_data_.size())
    {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = ee_pose_data_[ee_pose_idx++];
        ee_pose_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing ee_pose: [%s]", format_vector(msg.data).c_str());
    }
    else if (!finished)
    {
        RCLCPP_INFO(this->get_logger(), "Finished publishing ee data");
        finished = true;
        rclcpp::shutdown();
    }

    if (ee_vel_idx < ee_vel_data_.size())
    {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = ee_vel_data_[ee_vel_idx++];
        ee_vel_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing ee_vel: [%s]", format_vector(msg.data).c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Finished publishing ee_vel data");
    }
}

std::string format_vector(const std::vector<double> &vec)
{
    std::ostringstream oss;
    for (const auto &val : vec)
    {
        oss << val << " ";
    }
    return oss.str();
}


    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ee_pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ee_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::vector<double>> ee_pose_data_;
    std::vector<std::vector<double>> ee_vel_data_;
    size_t ee_pose_idx;
    size_t ee_vel_idx;
    bool finished;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EePoseAndVelPublisher>());
    rclcpp::shutdown();
    return 0;
}
