#include <rclcpp/rclcpp.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <signal.h>
#include <vector>
#include <string>
#include <fstream>
#include <cmath> // Include cmath to use M_PI

using namespace std;
using namespace placeholders;
using namespace chrono_literals;

class Tag1PoseNode : public rclcpp::Node 
{
    public:
        Tag1PoseNode() : Node("tag1pose_node"), start_time(this->now()) 
        {
            tag1_pose_subscriber = this->create_subscription
                            <apriltag_msgs::msg::AprilTagDetectionArray>("/apriltag_detections", 1, 
                            std::bind(&Tag1PoseNode::topic_callback, this, _1));
            
            tag1_pose_publisher = this->create_publisher
            <geometry_msgs::msg::PoseStamped>("/tag1_pose", 10);

            csv_file.open("/home/" + 
                            std::string(getenv("USER")) + 
                            "/noprehensileman_ws/src/data/Experiments/right50/tag1.csv");

            csv_file << "Timestamp,x1,y1,z1,roll1,pitch1,yaw1\n";
 
            signal(SIGINT, signalHandler);
        }

    private:        
        static std::ofstream csv_file;
        rclcpp::Time start_time;
        rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag1_pose_subscriber;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tag1_pose_publisher;

        void topic_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) 
        {
            bool tag1_found = false;
            apriltag_msgs::msg::AprilTagDetection detected;
            double dim = msg->detections.size();

            for (int i=0; i<dim; ++i) 
            {
                detected = msg->detections[i];
                if (detected.id == 1) 
                {  
                    geometry_msgs::msg::PoseStamped tag1_pose;
                    tag1_pose.header.stamp      = this->get_clock()->now();
                    tag1_pose.header.frame_id   = "camera";
                    tag1_pose.pose.position     = detected.pose.pose.pose.position;
                    tag1_pose.pose.orientation  = detected.pose.pose.pose.orientation;
                    tag1_pose_publisher->publish(tag1_pose);

                    double roll1, pitch1, yaw1;
                    tf2::Quaternion q1( tag1_pose.pose.orientation.x,
                                        tag1_pose.pose.orientation.y,
                                        tag1_pose.pose.orientation.z,
                                        tag1_pose.pose.orientation.w);
                    tf2::Matrix3x3 m1(q1);
                    m1.getRPY(roll1, pitch1, yaw1);
                    
                    // Convert RPY from radians to degrees
                    roll1   = roll1  * (180.0 / M_PI);
                    pitch1  = pitch1 * (180.0 / M_PI);
                    yaw1    = yaw1   * (180.0 / M_PI);
                    
                    auto elapsed_time = this->now() - start_time;
                    csv_file << elapsed_time.seconds() << ",";
                    csv_file    << tag1_pose.pose.position.x << "," 
                                << tag1_pose.pose.position.y << "," 
                                << tag1_pose.pose.position.z << ","
                                << roll1 << "," 
                                << pitch1 << "," 
                                << yaw1 << "\n";
                    
                    RCLCPP_INFO(this->get_logger(), "Tag1: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f", 
                                tag1_pose.pose.position.x, 
                                tag1_pose.pose.position.y, 
                                tag1_pose.pose.position.z, 
                                roll1, 
                                pitch1, 
                                yaw1);

                    tag1_found = true;
                    break;  
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

std::ofstream Tag1PoseNode::csv_file;

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tag1PoseNode>());
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

class Tag1PoseNode : public rclcpp::Node 
{
    public:
        Tag1PoseNode() 
            : Node("apriltag_storage_node"), 
              start_time(this->now()) 
        {
            tag1_pose_subscriber = this->create_subscription
                            <apriltag_msgs::msg::AprilTagDetectionArray>(
                            "/apriltag_detections",
                            10, std::bind(&Tag1PoseNode::topic_callback, this, _1));

            csv_file.open("/home/" + 
                            std::string(getenv("USER")) + 
                            "/noprehensileman_ws/src/data/left30/left30.csv");

            csv_file << "Timestamp,x1,y1,z1,roll1,pitch1,yaw1,x2,y2,z2,roll2,pitch2,yaw2\n";
 
            // Register signal handler for SIGINT (Ctrl+C)
            signal(SIGINT, signalHandler);

            // Create PoseStamped publishers for Tag 1 and Tag 2
            tag1_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/tag1_pose", 10);
            tag2_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/tag2_pose", 10);
        }

    private:
        rclcpp::Time start_time;
        static std::ofstream csv_file;
        rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag1_pose_subscriber;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tag1_pose_publisher;
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
                geometry_msgs::msg::PoseStamped tag1_pose;
                tag1_pose.header.stamp = this->get_clock()->now();
                tag1_pose.header.frame_id = "camera";
                tag1_pose.pose.position = tag1_pos;
                tag1_pose.pose.orientation = tag1_orient;
                tag1_pose_publisher->publish(tag1_pose);

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

std::ofstream Tag1PoseNode::csv_file;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tag1PoseNode>());
    rclcpp::shutdown();
    return 0;
}
*/