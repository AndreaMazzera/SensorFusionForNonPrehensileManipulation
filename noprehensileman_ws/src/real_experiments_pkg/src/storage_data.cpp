#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <fstream>
#include <iomanip>
#include <signal.h>
#include <array>
#include <cmath> // Include cmath to use M_PI

using namespace std;
using namespace placeholders;
using namespace chrono_literals;

class PoseSubscriberNode : public rclcpp::Node
{
    public:
        PoseSubscriberNode()
            : Node("storage_data_node"), start_time(this->now()) 
        {
            // Create subscribers for the tag1 and tag2 poses
            tag1_pose_subscriber_ = this->create_subscription
                <geometry_msgs::msg::PoseStamped>("/tag1_pose", 1,
                std::bind(&PoseSubscriberNode::tag1PoseCallback, this, _1));

            tag2_pose_subscriber_ = this->create_subscription
                <geometry_msgs::msg::PoseStamped>("/tag2_pose", 1,
                std::bind(&PoseSubscriberNode::tag2PoseCallback, this, _1));

            // Open the CSV file for writing
            csv_file.open(  "/home/" + 
                            std::string(getenv("USER")) + 
                            "/noprehensileman_ws/src/data/center50/center50.csv");

            if (csv_file.is_open())
            {
                csv_file << "timestamp,x1,y1,z1,R1,P1,Y1,x2,y2,z2,R2,P2,Y2\n";
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file for writing.");
            }

            auto timercallback = [this]() -> void 
            {
                writeToCSV();
            };
            timer = this->create_wall_timer(chrono::milliseconds(33), timercallback);

            signal(SIGINT, signalHandler);
        }

    private:
        static std::ofstream csv_file;
        rclcpp::Time start_time;
        rclcpp::TimerBase::SharedPtr timer;
        std::array<double, 6> tag1_pose_;
        std::array<double, 6> tag2_pose_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tag1_pose_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tag2_pose_subscriber_;

        void tag1PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            // Convert pose to x, y, z, R, P, Y and store in tag1_pose_
            convertPoseToRPY(msg, tag1_pose_);
        }

        void tag2PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            // Convert pose to x, y, z, R, P, Y and store in tag2_pose_
            convertPoseToRPY(msg, tag2_pose_);
        }

        void convertPoseToRPY(const geometry_msgs::msg::PoseStamped::SharedPtr msg, std::array<double, 6>& pose_array)
        {
            // Extract position
            pose_array[0] = msg->pose.position.x;
            pose_array[1] = msg->pose.position.y;
            pose_array[2] = msg->pose.position.z;

            // Convert orientation (quaternion) to roll, pitch, yaw
            tf2::Quaternion q(
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w);
            tf2::Matrix3x3 m(q);

            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            pose_array[3] = roll;
            pose_array[4] = pitch;
            pose_array[5] = yaw;
        }

        void writeToCSV()
        {
            auto elapsed_time = this->now() - start_time;
            csv_file << elapsed_time.seconds() << ",";
            csv_file << tag1_pose_[0] << ",";
            csv_file << tag1_pose_[1] << ",";
            csv_file << tag1_pose_[2] << ",";
            csv_file << tag1_pose_[3] << ",";
            csv_file << tag1_pose_[4] << ",";
            csv_file << tag1_pose_[5] << ",";

            csv_file << tag2_pose_[0] << ",";
            csv_file << tag2_pose_[1] << ",";
            csv_file << tag2_pose_[2] << ",";
            csv_file << tag2_pose_[3] << ",";
            csv_file << tag2_pose_[4] << ",";
            csv_file << tag2_pose_[5] << "\n";
        }

        static void signalHandler(int signum) 
        {
            if (signum == SIGINT) 
            {
                csv_file.close();
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received SIGINT signal. Closing CSV files and shutting down node.");
                rclcpp::shutdown();
            }
        }
};

// Initialize the static member
std::ofstream PoseSubscriberNode::csv_file;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
