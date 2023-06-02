#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <networktables/NetworkTableInstance.h>
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>
#include <networktables/NetworkTable.h>
#include <networktables/StringTopic.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
class NTClientNode : public rclcpp::Node
{
private:
    nt::NetworkTableInstance instance;
    std::shared_ptr<nt::NetworkTable> ros, apriltag;
    nt::StringPublisher status;
    rclcpp::TimerBase::SharedPtr timer;
    std::vector<rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr>
        apriltagNodesDetectionArrays;
    void timerCallback()
    {
    }
public:
    NTClientNode() : Node("ntclient_node")
    {
        this->declare_parameter("robot_width", 0.85);
        this->declare_parameter("robot_length", 0.85);
        this->declare_parameter("drivetrain_model", "differential");
        this->declare_parameter("apriltag_nodes", std::vector<std::string>());
        instance = nt::NetworkTableInstance::GetDefault();
        ros = instance.GetTable("ros");
        apriltag = ros->GetSubTable("apriltag");
        status = instance.GetStringTopic("status").Publish();
        status.Set("online");
        instance.StartClient4("coprocessor");
        instance.SetServerTeam(172);
        for (std::string nodeName : get_parameter("apriltag_nodes").as_string_array())
        {
            auto cameraTable = apriltag->GetSubTable(nodeName);
            apriltagNodesDetectionArrays.push_back(
                create_subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(nodeName, 10,
                    [&](isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray& msg)
                    {
                        std::vector<int> ids;
                        std::vector<double> data;
                        for (size_t i = 0; i < msg.detections.size(); i++)
                        {
                            ids.emplace_back(msg.detections[i].id);
                            data.emplace_back(msg.header.stamp.nanosec);
                            data.emplace_back(msg.detections[i].pose.pose.pose.position.x);
                            data.emplace_back(msg.detections[i].pose.pose.pose.position.y);
                            data.emplace_back(msg.detections[i].pose.pose.pose.position.z);
                            auto quaternion = msg.detections[i].pose.pose.pose.orientation;
                            tf2::Quaternion newQuaternion;
                            tf2::convert(quaternion, newQuaternion);
                            tf2::Matrix3x3 mat(newQuaternion);
                            double roll, pitch, yaw;
                            mat.getRPY(roll, pitch, yaw);
                            data.emplace_back(roll);
                            data.emplace_back(pitch);
                            data.emplace_back(yaw);
                        }
                    }
                )
            );
        }
    }
};
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NTClientNode>());
    rclcpp::shutdown();
    return 0;
}