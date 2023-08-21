//
// Created by Christoph Pilz
//
// Description:
// The CPMHandler handles CPMs
// - it converts ROS2 type messages into vcits type messages and vice versa
// - it takes care of basic input output handling
//
// Author(s): "Lukas, Kuschnig, Alina Steinberger"
// Copyright: "Copyright 2023, vehicleCAPTAIN toolbox"
// Credits: ["Lukas Kuschnig", "Christoph Pilz", "Alina Steinberger"]
// License: "BSD-3-clause"
// Version: "1.0.0"
// Maintainer: "Alina Steinberger"
// E-Mail: "alina.steinberger@v2c2.at"
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

#ifndef V2X_GW_CPMHANDLER_H
#define V2X_GW_CPMHANDLER_H

#include "V2XMHandler.h"
#include <v2x_msgs/msg/cpm_list.hpp>

class CPMHandler : public V2XMHandler {
public:
    /// Initializes the CPM handler
    CPMHandler(rclcpp::Node *gateway_node);

    /// Destroys CPM handler
    ~CPMHandler();

    /// Receive CPM diagnostics
    /// \return CPM diagnostic key value pairs
    virtual std::vector<diagnostic_msgs::msg::KeyValue> GetDiagnostics() override;

    /// Get the most recent CPM
    /// CPM is produced in the background with ros subscriber magic; this method encodes it before returning.
    /// \return most recent CPM
    std::queue<std::pair<void *, size_t>> GetMessages() override;

    /// Process the received CPM(s)
    /// CPM(s) will be processed and ros published
    /// \param msgs most recent set of received CPM(s)
    void PutMessages(std::queue<std::pair<void *, size_t>> msgs);

protected:
    /// Read the configuration for the CPM
    void ReadConfig() override;

private:
    // necessary handler processing constants
    bool CPM_ACTIVE;
    long CPM_DEBUG_MSG_THROTTLE_MS;
    bool CPM_WRITE_TRACE_FILE;

    bool new_data_received_;

    // CPM attributes
    void  *cpm_;

    // Published topics
    rclcpp::Publisher<v2x_msgs::msg::CPMList>::SharedPtr cpm_pub_;

    // Subsribed topics
    rclcpp::Subscription<v2x_msgs::msg::CPMList>::SharedPtr cpm_sub_;

    // Callbacks
    void rosCPMCallback(const v2x_msgs::msg::CPMList::SharedPtr cpm);

    // CPM generation
    void InitCPM();

    v2x_msgs::msg::CPM GetROSCPM(std::pair<void *, size_t> msg);

    // debug
    void PrintCPM();
};

#endif //V2X_GW_CPMHANDLER_H
