#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cstdio>
#include <exception>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "auto_bt/rclcpp_logger.hpp"
#include "auto_bt/srv/run_tree.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class BehaviorTreeExec : public rclcpp::Node {
 private:
    BT::BehaviorTreeFactory factory;
    bool isDebugging = false;
    std::vector<BT::Tree> treeVector;
    int activeTreeIdx = -1;
    std::shared_ptr<BT::RclcppLogger> logger;

    // Tick timer for ticking the tree
    rclcpp::TimerBase::SharedPtr tickTimer;

    // exec tree from index Service
    rclcpp::Service<auto_bt::srv::RunTree>::SharedPtr execStartService;
    void execStartCallback(
        const auto_bt::srv::RunTree::Request::SharedPtr request,
        auto_bt::srv::RunTree::Response::SharedPtr response);

    // Cancel running tree service
    // needs to move the tree directly to the "reset / abort state"

    // debug enable mode service
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr debugService;
    void debugCallback(const std_srvs::srv::SetBool::Request::SharedPtr request,
                       std_srvs::srv::SetBool::Response::SharedPtr response);

    // reset service is a full abort, no attempt to cancel
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetService;
    void resetCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                       std_srvs::srv::Trigger::Response::SharedPtr response);

    // timer callback for ticking the tree until complete
    void tickCallback();

 public:
    BehaviorTreeExec();
    ~BehaviorTreeExec();
    bool reset();
};

BehaviorTreeExec::BehaviorTreeExec() : Node("auto_bt_node") {
    this->declare_parameter<int>("tick_time_ms", 20);
    this->declare_parameter<std::vector<std::string>>(
        "yaml_plugin_array",{""});
    this->declare_parameter<std::vector<std::string>>(
        "yaml_tree_file_path_array", {""});

    std::vector<std::string> yamlPluginArr = {};
    std::vector<std::string> treePaths = {};
    int tickPeriod = 20;

    try {
        RCLCPP_INFO(this->get_logger(), "Getting parameter data");
        // get param values
        yamlPluginArr =
            this->get_parameter("yaml_plugin_array").as_string_array();
        tickPeriod = this->get_parameter("tick_time_ms").as_int();
        treePaths =
            this->get_parameter("yaml_tree_file_path_array").as_string_array();
    } catch (std::exception e) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Error checking params: " << e.what());
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown error checking params");
    }

    // create debug service
    debugService = this->create_service<std_srvs::srv::SetBool>(
        "/sys/debug",
        std::bind(&BehaviorTreeExec::debugCallback, this, _1, _2));

    // create the reset service
    resetService = this->create_service<std_srvs::srv::Trigger>(
        "/sys/reset",
        std::bind(&BehaviorTreeExec::resetCallback, this, _1, _2));

    // behavior tree factory
    factory = BT::BehaviorTreeFactory();

    // plugin loading phase
    for (auto plugin : yamlPluginArr) {
        if(plugin.empty()) continue;
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Loading plugin: " << plugin);
        factory.registerFromPlugin(plugin);
        RCLCPP_INFO_STREAM(this->get_logger(), "Loaded plugin: " << plugin);
    }

    // create the tick timer
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Ticking trees with period: " << tickPeriod << "ms");
    tickTimer = this->create_wall_timer(
        std::chrono::milliseconds(tickPeriod),
        std::bind(&BehaviorTreeExec::tickCallback, this));

    // load the xml trees
    treeVector = std::vector<BT::Tree>();
    for (auto path : treePaths) {
        if(path.empty()) continue;
        treeVector.push_back(factory.createTreeFromFile(path));
        RCLCPP_INFO_STREAM(this->get_logger(), "Loaded tree: " << path);
    }

    if (treeVector.size() < 1) {
        RCLCPP_ERROR(this->get_logger(),
                     "No trees were given to load. Shutting down.");
        throw std::runtime_error("Recieved no valid tree files to load");
    }

    reset();
}

BehaviorTreeExec::~BehaviorTreeExec() {}

bool BehaviorTreeExec::reset() {
    try {
        tickTimer->cancel();
        while (!tickTimer->is_canceled()) {
            RCLCPP_DEBUG(this->get_logger(), "Waiting for tick timer cancel");
        }

        // reset all trees
        for (size_t i = 0; i < treeVector.size(); i++) {
            treeVector.at(i).haltTree();
        }

        // attach an rclcpp logger to the tree
        logger = std::make_shared<BT::RclcppLogger>(treeVector.at(0).rootNode(),
                                                    this->shared_from_this());

        // reset flags
        isDebugging = false;

        // reset active tree to -1
        activeTreeIdx = -1;

        return true;
    } catch (std::exception e) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Error during reset: " << e.what());
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown error during reset");
    }
    return false;
}

void BehaviorTreeExec::tickCallback() {
    if (activeTreeIdx > -1 && activeTreeIdx < treeVector.size())
        treeVector.at(activeTreeIdx).tickRoot();
    else
        RCLCPP_ERROR_STREAM(
            this->get_logger(),
            "Attempting to tick an invalid tree at index " << activeTreeIdx);
}

void BehaviorTreeExec::execStartCallback(
    const auto_bt::srv::RunTree::Request::SharedPtr request,
    auto_bt::srv::RunTree::Response::SharedPtr response) {
    // select the tree
    activeTreeIdx = request->tree_idx;

    try {
        // bind the logger to the new tree
        logger->setTreeRoot(treeVector.at(activeTreeIdx).rootNode());

        // test ticking the root
        treeVector.at(activeTreeIdx).tickRoot();

        // allow the timer to tick
        tickTimer->reset();

        // we were able to start ticking the tree
        response->success = true;
    } catch (std::exception e) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Error starting tree: " << e.what());
        response->success = false;
        response->message = std::string("Error starting tree: ") + e.what();
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown error starting tree");
        response->success = false;
        response->message = std::string("Unknown error starting tree");
    }
}

void BehaviorTreeExec::resetCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
    try {
        reset();
    } catch (std::exception e) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Error starting tree: " << e.what());
        response->success = false;
        response->message = std::string("Error starting tree: ") + e.what();
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown error starting tree");
        response->success = false;
        response->message = std::string("Unknown error starting tree");
    }
}

void BehaviorTreeExec::debugCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response) {}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BehaviorTreeExec>());
    rclcpp::shutdown();
}
