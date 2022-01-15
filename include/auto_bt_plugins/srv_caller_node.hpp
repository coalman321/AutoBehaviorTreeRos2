#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

namespace ros2_bt {

/**
 * @brief 
 * 
 * @tparam T
 * 
 * This is going to be horredously stupid, but essentially the idea is to fake a service call by creating
 * a generic publisher and subscriber with the service QOS. send the input data, and listen for the response
 * on the other end with a generic subscriber? otherwise this gets really ugly 
 */

template <class T>
class SrvCallerAction : public BT::SyncActionNode {
 public:
    SrvCallerAction(const std::string& name,
                    const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::shared_ptr<rclcpp::Node>>(
                "node_handle", "The ROS2 node to use"),
            BT::InputPort<std::string>("topic_name", "The topic name to call"),
            BT::InputPort<std::string>("param1", "service param 1"),
            BT::InputPort<std::string>("param2", "service param 2"),
            BT::InputPort<std::string>("param3", "service param 3"),
            BT::InputPort<std::string>("param4", "service param 4"),
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