//
// Created by Christoph Pilz
//
// Description:
// The V2XMHandler is the main class to derive from for implementation of new messages
// - is an interface class
//
// Author(s): "Christoph Pilz"
// Copyright: "Copyright 2023, vehicleCAPTAIN toolbox"
// Credits: ["Christoph Pilz"]
// License: "BSD-3-clause"
// Version: "1.0.0"
// Maintainer: "Christoph Pilz"
// E-Mail: "christoph.pilz@v2c2.at"
// Status = "Production"
//
// Reference to used code:
// <Description1 what was used> (<Link>)
// <Description2 what was used> (<Link>)
//
// Possible Improvements:
// [] <Bug 1>
// [] <Refactoring Idea 2>
// [] <Feature Idea 3>
//

#ifndef V2X_GW_V2XMHANDLER_H
#define V2X_GW_V2XMHANDLER_H

#include <cstdint>
#include <queue>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

enum class MsgType : int {
    kNone = 0,
    kDENM = 1,
    kCAM = 2,
    kCPM = 14,
    kIVIM = 6
} ;

class V2XMHandler {
public:

    /// Initializes the message handler for a specific V2X message
    /// \param msg_type type of the message
    /// \param gateway_node reference to the ros2 gateway_node
    V2XMHandler(MsgType msg_type, rclcpp::Node *gateway_node);

    /// Destroys message handler for a specific V2X message
    virtual ~V2XMHandler();

    /// Receive handler diagnostics
    /// \return handler diagnostic key value pairs
    virtual std::vector<diagnostic_msgs::msg::KeyValue> GetDiagnostics() = 0;

    /// Get the messages generated by the handler
    /// V2X Messages have been prepared by ros magic in the background; get them here
    /// BE AWARE: you have to control all message parameters here, including the generation frequency!
    /// \return most recent generated V2X messages
    virtual std::queue <std::pair<void *, size_t>> GetMessages() = 0;

    /// Get the type of a V2X message
    /// \param msg as asn1 buffer with size
    /// \return the type of the message
    MsgType GetMessageType(std::pair<void *, size_t> msg);

    /// Put the received V2X messages
    /// Messages will be processed and ros published in the background
    /// \param msgs most recent set of received V2X messages
    virtual void PutMessages(std::queue <std::pair<void *, size_t>> msgs) = 0;

protected:
    // attributes - general
    bool is_active_;
    bool is_configured_;

    // oss asn1 world pointer
    void* world_;

    // attributes - messages
    int message_received_counter_;
    int message_sent_counter_;

    /// Get the V2X timestamp (Generation Delta Time), according to ETSI
    /// \return V2X message timestamp
    uint64_t GetGenerationDeltaTime();

    /// Get the V2X timestamp (Generation Delta Time), according to ETSI
    /// \param milliseconds timestamp since epoch in ms
    /// \return V2X message timestamp
    uint64_t GetGenerationDeltaTime(uint64_t milliseconds);

    /// Get access to the node; masked to prevent accidental tampering
    /// \return pointer to parent ros2 node
    rclcpp::Node *GetNode();

    /// Read the configuration for the message
    virtual void ReadConfig() = 0;

    // Helpers
    /// Get the frequency between two timestamps
    /// \param A first timestamp
    /// \param B second timestamp
    /// \return frequency between two timestamps
    double GetFrequency(rclcpp::Time A, rclcpp::Time B);

private:
    // Attributes
    MsgType msg_type_;
    rclcpp::Node *gateway_node_;
};

#endif //V2X_GW_V2XMHANDLER_H
