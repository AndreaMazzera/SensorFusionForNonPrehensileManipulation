#include <rclcpp/rclcpp.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>
#include <signal.h>
#include <vector>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;
using namespace placeholders;
using namespace chrono_literals;

class Tag2PoseNode : public rclcpp::Node 
{
    public:
        Tag2PoseNode() : Node("tag2pose_node"), start_time(this->now()) 
        {
            subscription_ = this->create_subscription
                            <apriltag_msgs::msg::AprilTagDetectionArray>("/apriltag_detections",
                            10, std::bind(&Tag2PoseNode::topic_callback, this, _1));

            csv_file.open("/home/" + 
                            std::string(getenv("USER")) + 
                            "/noprehensileman_ws/src/data/Experiments/right50/tag2.csv");

            csv_file << "Timestamp,x2,y2,z2,roll2,pitch2,yaw2\n";
 
            // Create PoseStamped publishers for Tag 1 and Tag 2
            tag2_pose_publisher_ = this->create_publisher
            <geometry_msgs::msg::PoseStamped>("/tag2_pose", 10);
            
            // Register signal handler for SIGINT (Ctrl+C)
            signal(SIGINT, signalHandler);
        }

    private:
        static std::ofstream csv_file;
        rclcpp::Time start_time;
        rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr subscription_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tag2_pose_publisher_;

        void topic_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) 
        {
            auto elapsed_time = this->now() - start_time;
            csv_file << elapsed_time.seconds() << ",";

            bool tag1_found = false;
            for (const auto& detection : msg->detections) {
                if (detection.id == 2) {  // Check if the tag ID is 2
                    const auto& tag1_pos = detection.pose.pose.pose.position;
                    const auto& tag1_orient = detection.pose.pose.pose.orientation;

                    double roll1, pitch1, yaw1;
                    tf2::Quaternion q1(tag1_orient.x, tag1_orient.y, tag1_orient.z, tag1_orient.w);
                    tf2::Matrix3x3 m1(q1);
                    m1.getRPY(roll1, pitch1, yaw1);


                    double yaw_offset = 1.57;

                    csv_file << tag1_pos.x << "," << tag1_pos.y << "," << tag1_pos.z << ","
                              << roll1 << "," << pitch1 << "," << yaw1+yaw_offset << "\n";
                    RCLCPP_INFO(this->get_logger(), "Tag2: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f", 
                                tag1_pos.x, tag1_pos.y, tag1_pos.z, roll1, pitch1, yaw1+yaw_offset);
                    tag1_found = true;

                    tf2::Quaternion q;
                    q.setRPY(roll1, pitch1, yaw1+yaw_offset);  // Set the roll, pitch, and yaw

                    // Publish PoseStamped for Tag 1
                    geometry_msgs::msg::PoseStamped tag1_pose;
                    tag1_pose.header.stamp = this->get_clock()->now();
                    tag1_pose.header.frame_id = "camera";
                    tag1_pose.pose.position = tag1_pos;
                    tag1_pose.pose.orientation.x = q.getX();
                    tag1_pose.pose.orientation.y = q.getY();
                    tag1_pose.pose.orientation.z = q.getZ();
                    tag1_pose.pose.orientation.w = q.getW();
                    tag2_pose_publisher_->publish(tag1_pose);

                    break;  // Exit the loop once the tag with ID 1 is found
                }
            }

            
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

std::ofstream Tag2PoseNode::csv_file;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tag2PoseNode>());
    rclcpp::shutdown();
    return 0;
}




/*
## VERSION CON UN DUE TAG


#include <rclcpp/rclcpp.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>
#include <signal.h>
#include <vector>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;
using namespace placeholders;
using namespace chrono_literals;

class Tag2PoseNode : public rclcpp::Node 
{
    public:
        Tag2PoseNode() 
            : Node("apriltag_storage_node"), 
              start_time(this->now()) 
        {
            subscription_ = this->create_subscription
                            <apriltag_msgs::msg::AprilTagDetectionArray>(
                            "/apriltag_detections",
                            10, std::bind(&Tag2PoseNode::topic_callback, this, _1));

            csv_file.open("/home/" + 
                            std::string(getenv("USER")) + 
                            "/noprehensileman_ws/src/data/left30/left30.csv");

            csv_file << "Timestamp,x1,y1,z1,roll1,pitch1,yaw1,x2,y2,z2,roll2,pitch2,yaw2\n";
 
            // Register signal handler for SIGINT (Ctrl+C)
            signal(SIGINT, signalHandler);

            // Create PoseStamped publishers for Tag 1 and Tag 2
            tag2_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/tag2_pose", 10);
            tag2_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/tag2_pose", 10);
        }

    private:
        rclcpp::Time start_time;
        static std::ofstream csv_file;
        rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr subscription_;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tag2_pose_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tag2_pose_publisher_;

        void topic_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
            auto elapsed_time = this->now() - start_time;
            csv_file << elapsed_time.seconds() << ",";

            if (msg->detections.size() > 0) {
                const auto& tag1_pos = msg->detections[0].pose.pose.pose.position;
                const auto& tag1_orient = msg->detections[0].pose.pose.pose.orientation;

                double roll1, pitch1, yaw1;
                tf2::Quaternion q1(tag1_orient.x, tag1_orient.y, tag1_orient.z, tag1_orient.w);
                tf2::Matrix3x3 m1(q1);
                m1.getRPY(roll1, pitch1, yaw1);

                csv_file << tag1_pos.x << "," << tag1_pos.y << "," << tag1_pos.z << ","
                          << roll1 << "," << pitch1 << "," << yaw1 << ",";
                RCLCPP_INFO(this->get_logger(), "Tag1: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f", 
                            tag1_pos.x, tag1_pos.y, tag1_pos.z, roll1, pitch1, yaw1);

                // Publish PoseStamped for Tag 1
                geometry_msgs::msg::PoseStamped tag2_pose;
                tag2_pose.header.stamp = this->get_clock()->now();
                tag2_pose.header.frame_id = "camera";
                tag2_pose.pose.position = tag1_pos;
                tag2_pose.pose.orientation = tag1_orient;
                tag2_pose_publisher_->publish(tag2_pose);

            } else {
                csv_file << "0,0,0,0,0,0,";
                RCLCPP_INFO(this->get_logger(), "Tag1: x=0, y=0, z=0, roll=0, pitch=0, yaw=0");
            }

            if (msg->detections.size() > 1) {
                const auto& tag2_pos = msg->detections[1].pose.pose.pose.position;
                const auto& tag2_orient = msg->detections[1].pose.pose.pose.orientation;

                double roll2, pitch2, yaw2;
                tf2::Quaternion q2(tag2_orient.x, tag2_orient.y, tag2_orient.z, tag2_orient.w);
                tf2::Matrix3x3 m2(q2);
                m2.getRPY(roll2, pitch2, yaw2);

                csv_file << tag2_pos.x << "," << tag2_pos.y << "," << tag2_pos.z << ","
                          << roll2 << "," << pitch2 << "," << yaw2 << "\n";
                RCLCPP_INFO(this->get_logger(), "Tag2: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f", 
                            tag2_pos.x, tag2_pos.y, tag2_pos.z, roll2, pitch2, yaw2);

                // Publish PoseStamped for Tag 2
                geometry_msgs::msg::PoseStamped tag2_pose;
                tag2_pose.header.stamp = this->get_clock()->now();
                tag2_pose.header.frame_id = "camera";
                tag2_pose.pose.position = tag2_pos;
                tag2_pose.pose.orientation = tag2_orient;
                tag2_pose_publisher_->publish(tag2_pose);

            } else {
                csv_file << "0,0,0,0,0,0\n";
                RCLCPP_INFO(this->get_logger(), "Tag2: x=0, y=0, z=0, roll=0, pitch=0, yaw=0");
            }
        }

        static void signalHandler(int signum) {
            if (signum == SIGINT) {
                csv_file.close();
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received SIGINT signal. Closing CSV files and shutting down node.");
                rclcpp::shutdown();
            }
        }
};

std::ofstream Tag2PoseNode::csv_file;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tag2PoseNode>());
    rclcpp::shutdown();
    return 0;
}
*/