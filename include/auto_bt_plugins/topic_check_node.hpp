#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>

#include <rclcpp/rclcpp.hpp>

namespace ros2_bt {

template <class msgT>
class TopicValueAction : public BT::CoroActionNode {
 public:
    TopicCondition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::shared_ptr<rclcpp::Node>>(
                "node_handle", "The ROS2 node to use"),
            BT::InputPort<std::string>("topic_name",
                                       "The topic name to subscribe to"),
            BT::InputPort<std::string>("condition",
                                       "The condtion to use when evaluating"),
            BT::InputPort<std::string>("value", "The value to check against"),
        };
    }

    // The main override required by a BT service
    BT::NodeStatus tick() override {
        if (!getInput("node_handle", rosNode)) {
            throw BT::RuntimeError(
                "Missing parameter [ros2_node] in topic condition check");
        }
        if (!getInput("topic_name", topicName)) {
            throw BT::RuntimeError(
                "Missing parameter [topic_name] in topic condition check");
        }
        if (!getInput("condition", condType)) {
            throw BT::RuntimeError(
                "Missing parameter [condition] in topic condition check");
        }
        if (!getInput("value", service_name_)) {
            throw BT::RuntimeError(
                "Missing parameter [value] in topic condition check");
        }
    }

 private:
    rclcpp::Node::SharedPtr rosNode;
    std::shared_ptr<rclcpp::Subscription<typename msgT>> sub;
    std::string topicName, condType;
    std::shared_ptr<typename msgT> message;
};

}  // namespace ros2_bt