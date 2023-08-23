//
// Created by <Nives Križanec> on <Date as 16.05.2023>.
//
// Description:
// The file contains methods of theIVIMHandler which handles the translation of IVI messages
// There is a IVIMUtils.h file containing additional translation functions
//
// Author(s): "Nives Križanec"
// Copyright: "Copyright 2007, The Cogent Project"
// Credits: ["Christoph Pilz"]
// License: "GPL"
// Version: TBD
// Maintainer: TBD
// E-Mail: "Nives.Krizanec@v2c2.at"
// Status = "Production"
//
// Possible Improvements:
// [] <Bug 1>
// [] <Refactoring Idea 2>
// [] <Feature Idea 3>
//

#ifndef V2X_GW_IVIMHANDLER_H
#define V2X_GW_IVIMHANDLER_H


#include "V2XMHandler.h"
#include "../utils/IVIMUtils.h"

#include <nav_msgs/msg/odometry.hpp>

#include <v2x_msgs/msg/ivim_list.hpp>





class IVIMHandler : public V2XMHandler {
public:
  /// Initializes the IVIM handler
  IVIMHandler(rclcpp::Node *gateway_node);

  /// Destroys IVIM handler
  ~IVIMHandler();

  std::vector<diagnostic_msgs::msg::KeyValue> GetDiagnostics() override;

  /// Get the most recent IVIM
  /// IVIM is produced in the background with ros subscriber magic; this method encodes it before returning.
  /// \return most recent IVIM
  std::queue<std::pair<void *, size_t>> GetMessages() override;

  /// Process the received IVIM(s)
  /// IVIM(s) will be processed and ros published
  /// \param messages most recent set of received IVIM(s)
  void PutMessages(std::queue<std::pair<void *, size_t>> messages) override;

protected:
  /// Read the configuration for the IVIM
  void ReadConfig() override;

private:
  // necessary handler processing constants
  bool IVIM_ACTIVE;
  long IVIM_DEBUG_MSG_THROTTLE_MS;

  bool new_data_received_;

  // IVIM attributes
  std::vector <void*> ivim_list_;

  // Published topics
  rclcpp::Publisher<v2x_msgs::msg::IVIMList>::SharedPtr ivim_pub_;

  // Subsribed topics
  rclcpp::Subscription<v2x_msgs::msg::IVIMList>::SharedPtr ros_ivim_sub_;

  // List maniputaliton functions
  void addIVIM();
  void removeIVIM();

  // Callbacks
  void RosIVIMCallback(const v2x_msgs::msg::IVIMList::SharedPtr ros_ivim_list);

  // IVIM generation
  void InitIVIM();
  void fillIVIM(v2x_msgs::msg::IVIM ros_ivim, void* asn_ivim_void_ptr);

  v2x_msgs::msg::IVIM GetROSIVIM(std::pair<void *, size_t> message);

  // debug
  void PrintIVIM();

};

#endif // V2X_GW_IVIMHANDLER_H
