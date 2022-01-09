#pragma once

#include <behaviortree_cpp_v3/loggers/abstract_logger.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/int32.hpp>
#include <map>

namespace BT {
class RclcppLogger : public StatusChangeLogger {
    static std::atomic<bool> ref_count;

 public:
    RclcppLogger(
        TreeNode* root_node, rclcpp::Node::SharedPtr rosNode,
        rclcpp::Logger::Level verbosity_level = rclcpp::Logger::Level::Info);

    void setTreeRoot(TreeNode* root_node);

    rclcpp::Logger::Level getLevel() const;

    // Accepts only Info and Debug
    void setLevel(rclcpp::Logger::Level level);

    ~RclcppLogger() override;

    virtual void callback(Duration timestamp, const TreeNode& node,
                          NodeStatus prev_status, NodeStatus status) override;

    virtual void flush() override;

 private:
    rclcpp::Logger::Level _level;
    rclcpp::Node::SharedPtr rosNode;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr nodeIdxPub;
    int activeNodeIdx;
    std::map<uint16_t, uint16_t> uidToIdxMap = {};
};

}  // namespace BT
