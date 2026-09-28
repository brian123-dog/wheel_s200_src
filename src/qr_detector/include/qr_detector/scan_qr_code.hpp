#pragma once

#include <string>
#include <mutex>

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>    // 新增：用于发布 /qr_detected
#include <opencv2/opencv.hpp>
#include <zbar.h>

// 前向声明
namespace BT {
class BehaviorTreeFactory;
}

namespace qr_detector {

class ScanQRCode : public BT::StatefulActionNode {
public:
    ScanQRCode(const std::string& name, const BT::NodeConfiguration& cfg);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_pub_;  // 新增：发布 /qr_detected

    std::string save_path_;
    std::string camera_topic_;
    double timeout_sec_;

    rclcpp::Time start_time_;
    bool image_received_;
    int retry_count_;
    static constexpr int MAX_RETRY = 3;

    std::string detected_qr_;
    std::string saved_file_;
    cv::Mat latest_image_;
    std::mutex image_mutex_;

    // ---- ZBar 扫描器 (成员复用，避免每次重构) ----
    zbar::ImageScanner scanner_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    std::string detectQRCode(const cv::Mat& image);
    bool saveJsonToFile();
    std::string generateTimestampFilename() const;

    // ---- base64 + gzip 解码 ----
    static std::string decodeBase64Gzip(const std::string& encoded);
};

/** @brief 向 BT 工厂注册 ScanQRCode 节点类型 */
void RegisterQRDetector(BT::BehaviorTreeFactory& factory);

}  // namespace qr_detector
