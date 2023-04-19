//
// Created by Christoph Pilz
//
// Description:
// The V2XZMQServer is the ZMQ implementation of the V2XServer that connects to the vehicleCAPTAIN routing core
// - this implementation of the V2XServer connects to the vehicleCAPTAIN routing core
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

#ifndef V2X_GW_V2XZMQSERVER_H
#define V2X_GW_V2XZMQSERVER_H

#include <queue>
#include <string>
#include <utility>

#include "zmq/zmq.hpp"

#include "V2XServer.h"

class V2XZMQServer : public V2XServer {
public:
    /// Sets up and connects to the server
    /// \param gateway_node reference to the ros2 gateway_node
    /// \param v2x_m_handler reference to the message handlers
    V2XZMQServer(rclcpp::Node *gateway_node, std::map<MsgType, V2XMHandler *> v2x_m_handler);

    /// Disconnects from the server
    ~V2XZMQServer();

    /// Checks if the server is still alive
    /// \return true, if alive; false, otherwise
    bool IsAlive() override;

    /// Receive server diagnostics
    /// \return server diagnostic key value pairs
    virtual std::vector<diagnostic_msgs::msg::KeyValue> GetDiagnostics() override;

    /// Receives V2X messages from the server
    /// \return a queue of received bytestreams
    std::queue <std::pair<void *, size_t>> ReceiveMessages() override;

    /// Sends V2X messages to the server
    /// \param msgs a queue of bytestream messages to send
    void SendMessages(std::queue <std::pair<void *, size_t>> msgs) override;

protected:
    /// Read the configuration for the ZMQ server
    void ReadConfig() override;

private:
    // necessary CAM constants
    std::string SERVER_TYPE;
    std::string SERVER_ADDRESS;
    std::string SERVER_TOPIC_FILTER_HB;
    std::string SERVER_TOPIC_FILTER_V2X;
    long SERVER_RECEIVE_PORT;
    long SERVER_SEND_PORT;
    long SERVER_CYCLE_TIME_MS;

    // zmq server attributes
    zmq::context_t *zmq_context_;
    zmq::socket_t *zmq_recv_socket_;
    zmq::socket_t *zmq_snd_socket_;

};

#endif //V2X_GW_V2XZMQSERVER_H
