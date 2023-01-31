//
// Created by Christoph Pilz
//
// Description:
// The ExampleMHandler handles ExampleMs
// - it is a sample implementation
// - it may serve as placeholder
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

#ifndef V2X_GW_EXAMPLEMHANDLER_H
#define V2X_GW_EXAMPLEMHANDLER_H

#include "V2XMHandler.h"

class ExampleMHandler : public V2XMHandler {
public:
    /// Initializes the ExampleM handler
    ExampleMHandler(rclcpp::Node *gateway_node);

    /// Destroys ExampleM handler
    ~ExampleMHandler();

    /// Get the most recent ExampleM
    /// ExampleM is produced in the background with ros subscriber magic; this method encodes it before returning.
    /// \return most recent ExampleM
    std::queue<std::pair<void *, size_t>> GetMessages() override;

    /// Process the received ExampleM(s)
    /// ExampleM(s) will be processed and ros published
    /// \param msgs most recent set of received ExampleM(s)
    void PutMessages(std::queue<std::pair<void *, size_t>> msgs);

protected:
    /// Read the configuration for the ExampleM
    void ReadConfig() override;

private:
    // necessary handler processing constants
    bool EXAMPLEM_ACTIVE;
    long EXAMPLEM_DEBUG_MSG_THROTTLE_MS;

};

#endif //V2X_GW_EXAMPLEMHANDLER_H
