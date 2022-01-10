#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"

namespace ros2_bt {

class NodeThread;

class CreateROS2Node : public BT::SyncActionNode {
 public:
    CreateROS2Node(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>(
                    "node_name", "The name of the ROS2 node to create"),
                BT::InputPort<std::string>(
                    "namespace", "The namespace in which to create the node"),
                BT::OutputPort<std::shared_ptr<rclcpp::Node>>(
                    "node_handle", "The node handle of the created node")};
    }

    BT::NodeStatus tick() override {
        std::string node_name;
        if (!getInput("node_name", node_name)) {
            throw BT::RuntimeError(
                "Missing parameter [node_name] in CreateROS2Node");
        }

        std::string ns;
        if (!getInput("namespace", ns)) {
            throw BT::RuntimeError(
                "Missing parameter [namespace] in CreateROS2Node");
        }

        auto node = std::make_shared<rclcpp::Node>(node_name, ns);

        if (!setOutput("node_handle", node)) {
            throw BT::RuntimeError(
                "Failed to set output port value [node_handle] in "
                "CreateROS2Node");
        }

        node_thread_ = std::make_unique<NodeThread>(node);

        return BT::NodeStatus::SUCCESS;
    }

 private:
    std::shared_ptr<NodeThread> node_thread_;
};

class NodeThread {
 public:
    explicit NodeThread(
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
        : node_(node_base) {
        thread_ = std::make_unique<std::thread>([&]() {
            executor_.add_node(node_);
            executor_.spin();
            executor_.remove_node(node_);
        });
    }

    template <typename NodeT>
    explicit NodeThread(NodeT node)
        : NodeThread(node->get_node_base_interface()) {}

    ~NodeThread() {
        executor_.cancel();
        thread_->join();
    }

 protected:
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
    std::unique_ptr<std::thread> thread_;
    rclcpp::executors::SingleThreadedExecutor executor_;
};
}  // namespace ros2_bt
