#include "auto_bt/rclcpp_logger.hpp"

#include <rclcpp/qos.hpp>
#include <algorithm>

namespace BT {
std::atomic<bool> RclcppLogger::ref_count(false);

RclcppLogger::RclcppLogger(TreeNode* root_node, rclcpp::Node::SharedPtr rosNode,
                           rclcpp::Logger::Level verbosity_level)
    : StatusChangeLogger(root_node), _level(verbosity_level), rosNode(rosNode) {
    nodeIdxPub = rosNode->create_publisher<std_msgs::msg::Int32>(
        "/auto_bt/active_node_idx", rclcpp::SystemDefaultsQoS());

    setTreeRoot(root_node);
}

void RclcppLogger::setTreeRoot(TreeNode* root_node) {
    bool expected = false;
    if (!ref_count.compare_exchange_strong(expected, true)) {
        throw std::logic_error(
            "Only a single instance of RclcppLogger shall be created");
    }

    uidToIdxMap.clear();
    activeNodeIdx = -1;
}

rclcpp::Logger::Level RclcppLogger::getLevel() const { return _level; }

void RclcppLogger::setLevel(rclcpp::Logger::Level level) {
    if (level != rclcpp::Logger::Level::Debug &&
        level != rclcpp::Logger::Level::Info) {
        throw std::invalid_argument(
            "RclcppLogger::setLevel acepts only Debug or Info");
    }
    _level = level;
}

RclcppLogger::~RclcppLogger() {
    ref_count.store(false);
    nodeIdxPub->~Publisher();
}

void RclcppLogger::callback(Duration timestamp, const TreeNode& node,
                            NodeStatus prev_status, NodeStatus status) {
    constexpr const char* whitespaces = "                         ";
    const size_t ws_count = strlen(whitespaces) - 1;

    const auto& node_name = node.name();

    switch (_level) {
        case rclcpp::Logger::Level::Debug:
            RCLCPP_DEBUG(rosNode->get_logger(), "[%s%s]: %s -> %s",
                         node_name.c_str(),
                         &whitespaces[std::min(ws_count, node_name.size())],
                         toStr(prev_status, true), toStr(status, true));
            break;

        case rclcpp::Logger::Level::Info:
            RCLCPP_INFO(rosNode->get_logger(), "[%s%s]: %s -> %s",
                        node_name.c_str(),
                        &whitespaces[std::min(ws_count, node_name.size())],
                        toStr(prev_status, true), toStr(status, true));
            break;
    }

    // if this is true we discovered a unique node. assign a new index
    if (uidToIdxMap.find(node.UID()) != uidToIdxMap.end()) {
        activeNodeIdx++;
        uidToIdxMap.insert(std::pair<uint16_t, uint16_t>(node.UID(), activeNodeIdx));
    }

    // send the active node index
    auto msg = std_msgs::msg::Int32();
    // check for a uid match to pull the tree index
    msg.data = uidToIdxMap.at(node.UID());
    nodeIdxPub->publish(msg);
}

void RclcppLogger::flush() {}

}  // namespace BT