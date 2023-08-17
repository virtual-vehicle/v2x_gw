//
// Created by Christoph Pilz
//
// Description:
// The DENMHandler handles DENMs
// - it converts ROS2 type messages into vcits type messages and vice versa
// - it takes care of basic input output handling
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

#ifndef V2X_GW_DENMHANDLER_H
#define V2X_GW_DENMHANDLER_H

#include "V2XMHandler.h"

#include <v2x_msgs/msg/denm_list.hpp>

class DENMHandler : public V2XMHandler {
public:
    /// Initializes the DENM handler
    DENMHandler(rclcpp::Node *gateway_node);

    /// Destroys DENM handler
    ~DENMHandler();

    /// Receive DENM diagnostics
    /// \return DENM diagnostic key value pairs
    virtual std::vector<diagnostic_msgs::msg::KeyValue> GetDiagnostics() override;

    /// Get the most recent DENMs
    /// DENMs are produced in the background with ros subscriber magic; this method encodes them before returning.
    /// \return most recent DENMs
    std::queue<std::pair<void *, size_t>> GetMessages() override;

    /// Process the received DENM(s)
    /// DENM(s) will be processed and ros published
    /// \param msgs most recent set of received DENM(s)
    void PutMessages(std::queue<std::pair<void *, size_t>> msgs);

protected:
    /// Read the configuration for the DENM
    void ReadConfig() override;

private:
    // necessary handler processing constants
    bool DENM_ACTIVE;
    long DENM_DEBUG_MSG_THROTTLE_MS;

    // necessary header constants
    long DENM_HEADER_STATION_ID;

    // necessary DENM constants

    // constants that should be set by code

    // DENM attributes
    void  *denm_;

    // Published topics
    rclcpp::Publisher<v2x_msgs::msg::DENMList>::SharedPtr denm_pub_;

    // Subsribed topics

    // Callbacks

    // DENM generation
    void InitDENM();

    v2x_msgs::msg::DENM GetROSDENM(std::pair<void *, size_t> msg);

    // debug
    void PrintDENM();

};

#endif //V2X_GW_DENMHANDLER_H
