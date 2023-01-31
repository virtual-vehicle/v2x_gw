
#include "ExampleMHandler.h"

ExampleMHandler::ExampleMHandler(rclcpp::Node *gateway_node)
        : V2XMHandler(MsgType::kNone, gateway_node) {

    // configure
    ReadConfig();

    // publisher

    // subscriber
}

ExampleMHandler::~ExampleMHandler() {}

std::queue<std::pair<void *, size_t>> ExampleMHandler::GetMessages() {

    // processing variables
    rclcpp::Time current_timestamp = GetNode()->get_clock()->now();
    std::queue<std::pair<void *, size_t>> examplem_queue; // depending on message type, there may be 0..N messages in the queue

    // static variables
    static rclcpp::Time prev_process_timestamp = current_timestamp;

    // max frequency check -> ExampleM specs <= 1Hz
    if(GetFrequency(current_timestamp, prev_process_timestamp) <= 1.0) {

        auto& clk = *GetNode()->get_clock();
        RCLCPP_WARN_THROTTLE(GetNode()->get_logger(), clk, EXAMPLEM_DEBUG_MSG_THROTTLE_MS,
                             "GetMessages(): not yet implemented for ExampleM -> throwing \"Not Implemented\"");

        throw "NotImplemented";

        prev_process_timestamp = current_timestamp;
    }

    return examplem_queue;
}

void ExampleMHandler::PutMessages(std::queue<std::pair<void *, size_t>> msgs) {
    // take each element from the incoming_queue, convert it to ROS2 format and publish
    RCLCPP_WARN(GetNode()->get_logger(), "PutMessages(): not yet implemented for ExampleM -> throwing \"Not Implemented\"");

    auto& clk = *GetNode()->get_clock();
    RCLCPP_WARN_THROTTLE(GetNode()->get_logger(), clk, EXAMPLEM_DEBUG_MSG_THROTTLE_MS,
                         "PutMessages(): not yet implemented for ExampleM -> throwing \"Not Implemented\"");

    throw "NotImplemented";
}

void ExampleMHandler::ReadConfig() {
    // read configuration and set is_configured_

    // necessary handler processing constants
    GetNode()->declare_parameter("examplem.active", false);
    GetNode()->get_parameter("examplem.active", EXAMPLEM_ACTIVE);
    GetNode()->declare_parameter("examplem.handler_debug_msg_throttle_ms", 5000);
    GetNode()->get_parameter("examplem.handler_debug_msg_throttle_ms", EXAMPLEM_DEBUG_MSG_THROTTLE_MS);

    if(EXAMPLEM_ACTIVE) {
        is_active_ = true;

        // implement configuration!
        RCLCPP_ERROR(GetNode()->get_logger(), "Configuration not yet implemented for ExampleM");
        throw "NotImplemented";
    }

    is_configured_ = true;
}
