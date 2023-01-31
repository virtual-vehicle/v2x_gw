//
// Created by Christoph Pilz
//
// Description:
// The V2XServer is the interface class for the Server implementation
// - inherit this interface class to implement another V2X connection
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

#ifndef V2X_GW_V2XSERVER_H
#define V2X_GW_V2XSERVER_H

#include <queue>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "handler/V2XMHandler.h"

class V2XServer {
public:
    /// Stores parameters, derivatives should setup and connect to server in constructor
    /// \param gateway_node reference to the ros2 gateway_node
    /// \param v2x_m_handler reference to the message handlers
    V2XServer(rclcpp::Node *gateway_node, std::map<MsgType, V2XMHandler *> v2x_m_handler);

    /// Derivates should disconnect and clean up server in destructor
    virtual ~V2XServer();

    /// Checks if the server is still alive
    /// \return true, if alive; false, otherwise
    virtual bool IsAlive() = 0;

    /// Receive server diagnostics
    /// \return server diagnostic key value pairs
    virtual std::vector<diagnostic_msgs::msg::KeyValue> GetDiagnostics() = 0;

    /// Receives V2X messages from the server
    /// \return a queue of received bytestreams
    virtual std::queue<std::pair<void *, size_t>> ReceiveMessages() = 0;

    /// Sends V2X messages to the server
    /// \param msgs a queue of bytestream messages to send
    virtual void SendMessages(std::queue<std::pair<void *, size_t>> msgs) = 0;

protected:
    // attributes - general
    bool is_active_;
    bool is_configured_;
    bool is_connected_;

    std::map<MsgType, V2XMHandler *> v2x_m_handler_;

    // attributes - client
    long client_received_messages_;
    long client_sent_messages_;
    rclcpp::Time client_last_message_received_;
    rclcpp::Time client_last_message_sent_;

    // attributes - server
    long server_heartbeat_counter_;
    long server_received_messages_;
    long server_sent_messages_;
    rclcpp::Time server_last_message_received_;
    rclcpp::Time server_last_message_sent_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    /// Get access to the node; masked to prevent accidental tampering
    /// \return pointer to parent ros2 node
    rclcpp::Node *GetNode();

    /// main server method; processes incoming/outgoing messages
    void Process(void);

    /// Read the configuration for the server
    virtual void ReadConfig() = 0;

    /// Returns an IP address for a given hostname
    /// \param hostname to find the IP address for
    /// \return ip address for the given hostname
    std::string GetIPAddressFromHostname(std::string hostname);

    /// Verifies if a given IPv4 Address is valid
    /// \param ip_address to check
    /// \return true if valid, false if invalid
    bool VerifyIPv4Address(std::string ip_address);

private:
    // attributes
    rclcpp::Node *gateway_node_;

};

#endif //V2X_GW_V2XSERVER_H
