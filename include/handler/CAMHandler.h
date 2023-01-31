//
// Created by Christoph Pilz
//
// Description:
// The CAMHandler handles CAMs
// - it converts ROS2 type messages into vcits type messages and vice versa
// - it takes care of basic input output handling
//
// Author(s): "Christoph Pilz, Alina Steinberger"
// Copyright: "Copyright 2023, vehicleCAPTAIN toolbox"
// Credits: ["Christoph Pilz", "Alina Steinberger"]
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

#ifndef V2X_GW_CAMHANDLER_H
#define V2X_GW_CAMHANDLER_H

#include "V2XMHandler.h"

#include <nav_msgs/msg/odometry.hpp>

#include <v2x_msgs/msg/cam_list.hpp>

extern "C" {
#include "vcits/cam/CAM.h"
}


class CAMHandler : public V2XMHandler {
public:
    /// Initializes the CAM handler
    CAMHandler(rclcpp::Node *gateway_node);

    /// Destroys CAM handler
    ~CAMHandler();

    /// Get the most recent CAM
    /// CAM is produced in the background with ros subscriber magic; this method encodes it before returning.
    /// \return most recent CAM
    std::queue<std::pair<void *, size_t>> GetMessages() override;

    /// Process the received CAM(s)
    /// CAM(s) will be processed and ros published
    /// \param msgs most recent set of received CAM(s)
    void PutMessages(std::queue<std::pair<void *, size_t>> msgs);

protected:
    /// Read the configuration for the CAM
    void ReadConfig() override;

private:
    // necessary handler processing constants
    bool CAM_ACTIVE;
    long CAM_DEBUG_MSG_THROTTLE_MS;
    
    bool new_data_received_;

    // CAM attributes
    std::vector <CAM_t*> cam_list_;
    std::mutex cam_list_lock_;

    // Published topics
    rclcpp::Publisher<v2x_msgs::msg::CAMList>::SharedPtr cam_pub_;

    // Subsribed topics
    rclcpp::Subscription<v2x_msgs::msg::CAMList>::SharedPtr ros_cam_sub_;

    // Callbacks
    void RosCAMCallback(const v2x_msgs::msg::CAMList::SharedPtr odom);

    // CAM generation
    void InitCAM();
    void fillCAM(v2x_msgs::msg::CAM ros_cam, CAM_t* cam);


    v2x_msgs::msg::CAM GetROSCAM(std::pair<void *, size_t> msg);

    // debug
    void PrintCAM();

};

#endif //V2X_GW_CAMHANDLER_H
