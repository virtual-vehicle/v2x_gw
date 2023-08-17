//
// Created by Christoph Pilz
//
// Description:
// This is the main file for the ROS2 node
// - it handles the server connection for rx/tx of V2X messages
// - it handles the execution of Handler methods for the single V2X messages
//
// Author(s): "Christoph Pilz, Markus Schratter"
// Copyright: "Copyright 2023, vehicleCAPTAIN toolbox"
// Credits: ["Christoph Pilz", "Markus Schratter"]
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

#ifndef _V2XGATEWAY_H_
#define _V2XGATEWAY_H_

#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <list>
#include <map>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <stdlib.h>


#include "rclcpp/rclcpp.hpp"
#include <sys/time.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2/visibility_control.h>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "V2XMHandler.h"
#include "server/V2XServer.h"

using namespace std::chrono_literals;

class V2XGateway : public rclcpp::Node {
public:
    V2XGateway();

    ~V2XGateway();

protected:

private:

    // Attributes
    bool is_node_initialized_;

    diagnostic_msgs::msg::DiagnosticArray diagnostics_;

    std::map<MsgType, V2XMHandler *> v2x_m_handler_;

    V2XServer *v2x_server_;

    // Parameters
    std::string ZMQ_IP_ADDRESS;
    int ZMQ_SEND_PORT;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Published topics
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

    // Methods
    bool readConfig();

    void process(void);

    // Helpers
    double getTimeDifferenceSeconds(rclcpp::Time A, rclcpp::Time B);

};

#endif // _V2XGATEWAY_H_
