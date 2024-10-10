#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

using namespace std;
using namespace placeholders;
using namespace chrono_literals;

class FtSensorPublisher : public rclcpp::Node
{
public:
    FtSensorPublisher() : Node("ftsensor_publisher"), idx(0), finished(false)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("ftsensor_data", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), 
            std::bind(&FtSensorPublisher::timer_callback, this));

        std::string base_path = "/home/" + 
                                std::string(getenv("USER")) + 
                                "/noprehensileman_ws/src/data/Experiments/center20/";
        load_data_from_file(base_path + "ftsensor_data_raw.txt", data_);
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
        if (idx < data_.size())
        {
            auto msg = std_msgs::msg::Float64MultiArray();
            msg.data = data_[idx++];
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Publishing ftsensor data: [%s]", format_vector(msg.data).c_str());
        }
        else if (!finished)
        {
            RCLCPP_INFO(this->get_logger(), "Finished publishing ftsensor data");
            finished = true;
            rclcpp::shutdown();
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

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::vector<double>> data_;
    size_t idx;
    bool finished;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FtSensorPublisher>());
    rclcpp::shutdown();
    return 0;
}


