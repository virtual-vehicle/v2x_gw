// -*- lsst-c++ -*-

/*
* <Name of Software>
* See COPYRIGHT file at the top of the source tree.
*
* (Optional)
* This product includes software developed by the
* <Project Name> (<http://projectlink.org/>).
*
* (Licensing)
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a cop&=IVIM_Ty of the LSST License Statement and
* the GNU General Public License along with this program. If not,
* see <http://www.lsstcorp.org/LegalNotices/>.
*/

#include "handler/IVIMHandler.h"
#include "handler/ossits/specifics/IVIMUtils.h"

extern "C" {
#include "ossits/include/ossits/ossits.h"
}

IVIMHandler::IVIMHandler(rclcpp::Node *gateway_node)
        : V2XMHandler(MsgType::kIVIM, gateway_node) {

  // configure
  ReadConfig();
  new_data_received_ = false;

  //init ossits world
  world_ = malloc(sizeof(OssGlobal));
  int retcode;
  if (retcode = ossinit((OssGlobal*) world_, ITS_Container)) {
    RCLCPP_ERROR(GetNode()->get_logger(), "ossinit error: %d", retcode);
  }

  ossSetEncodingFlags((OssGlobal*)world_, DEBUGPDU);
  ossSetDecodingFlags((OssGlobal*)world_, DEBUGPDU);

  // publisher
  ivim_pub_ = GetNode()->create_publisher<v2x_msgs::msg::IVIMList>("ivim/received", 1);

  // subscriber
  ros_ivim_sub_ = GetNode()->create_subscription<v2x_msgs::msg::IVIMList>
          ("ivim/transmitted", 10, std::bind(&IVIMHandler::RosIVIMCallback, this, std::placeholders::_1));


  IVIMUtils::setNode(GetNode());
}

IVIMHandler::~IVIMHandler() {
  // free the ivim structure
  while(ivim_list_.size() > 0){
    //frees and removes last element of ivim_list
    int ret_code;
    if ((ret_code = ossFreePDU((ossGlobal*)world_, IVIM_PDU, ivim_list_.back())) != 0) {
        RCLCPP_ERROR(GetNode()->get_logger(), "Free decoded error: %d", ret_code);
    }
    ivim_list_.pop_back();
  }

  ossterm((ossGlobal*)world_);
  free(world_);
}

std::vector<diagnostic_msgs::msg::KeyValue> IVIMHandler::GetDiagnostics() {
  std::vector<diagnostic_msgs::msg::KeyValue> values;
  diagnostic_msgs::msg::KeyValue key_value;

  // status - general
  key_value.key = "v2x_handler.ivim.is_active_";
  key_value.value = std::to_string(is_active_);
  values.push_back(key_value);
  key_value.key = "v2x_handler.ivim.is_configured_";
  key_value.value = std::to_string(is_configured_);
  values.push_back(key_value);

  // status - messages
  key_value.key = "v2x_handler.ivim.message_received_counter_";
  key_value.value = std::to_string(message_received_counter_);
  values.push_back(key_value);
  key_value.key = "v2x_handler.ivim.message_sent_counter_";
  key_value.value = std::to_string(message_sent_counter_);
  values.push_back(key_value);

  return values;
}

std::queue <std::pair<void *, size_t>> IVIMHandler::GetMessages() {

  // processing variables
  std::queue <std::pair<void *, size_t>> ivim_queue;

  if(!new_data_received_){
    return ivim_queue;
  }

  // IVIM creation
  OssBuf final_ivim_buffer;
  int ret_code;

  for(void* ivim_void_ptr : ivim_list_){
    IVIM* ivim = (IVIM*) ivim_void_ptr;

    final_ivim_buffer.value = NULL;  
    final_ivim_buffer.length = 0;

    if ((ret_code = ossEncode((ossGlobal*)world_, IVIM_PDU, (IVIM*)ivim, &final_ivim_buffer)) != 0) {
        RCLCPP_ERROR(GetNode()->get_logger(), "IVIM creation error: %d with message %s", ret_code, ossGetErrMsg((OssGlobal*) world_));
    }else{
        ivim_queue.push(std::make_pair(final_ivim_buffer.value, final_ivim_buffer.length));
        auto& clk = *GetNode()->get_clock();
        RCLCPP_INFO_THROTTLE(GetNode()->get_logger(), clk, IVIM_DEBUG_MSG_THROTTLE_MS,
                                    "IVIM created successfully with size %ld", final_ivim_buffer.length);
    }
  }

  new_data_received_ = false;
  message_sent_counter_ += ivim_queue.size();

  return ivim_queue;
}

void IVIMHandler::PutMessages(std::queue <std::pair<void *, size_t>> messages) {
  // variables
  v2x_msgs::msg::IVIMList ivim_list;
  ivim_list.header.stamp = GetNode()->get_clock()->now();

  // take each element from the incoming_queue, convert it to ROS2 format and publish
  while (!messages.empty()) {
    ivim_list.ivims.push_back(GetROSIVIM(messages.front()));
    messages.pop();
  }

  message_received_counter_ += ivim_list.ivims.size();

  ivim_pub_->publish(ivim_list);
}
//
void IVIMHandler::ReadConfig() {
  // read configuration and set is_configured_

  // necessary handler processing constants
  GetNode()->declare_parameter("ivim.active", false);
  GetNode()->get_parameter("ivim.active", IVIM_ACTIVE);
  GetNode()->declare_parameter("ivim.handler_debug_msg_throttle_ms", 5000);
  GetNode()->get_parameter("ivim.handler_debug_msg_throttle_ms", IVIM_DEBUG_MSG_THROTTLE_MS);

  if (IVIM_ACTIVE) {
    is_active_ = true;
  }

  is_configured_ = true;
}

void IVIMHandler::addIVIM() {
  IVIM* new_ivim = (IVIM*)malloc(sizeof(IVIM));
  memset(new_ivim, 0, sizeof(IVIM));
  ivim_list_.push_back((void*)new_ivim);
}

void IVIMHandler::removeIVIM() {
  int ret_code;
  if ((ret_code = ossFreePDU((ossGlobal*)world_, IVIM_PDU, ivim_list_.back())) != 0) {
      RCLCPP_ERROR(GetNode()->get_logger(), "Free decoded error: %d", ret_code);
  }
  ivim_list_.pop_back();
}

// Callbacks
void IVIMHandler::RosIVIMCallback(const v2x_msgs::msg::IVIMList::SharedPtr ros_ivim_list) {
  while(ivim_list_.size() < ros_ivim_list->ivims.size()) {
    addIVIM();
  }

  while(ivim_list_.size() > ros_ivim_list->ivims.size()) {
    removeIVIM();
  }

  if(ivim_list_.size() != ros_ivim_list->ivims.size()) {
    std::cout << "[ERROR] wrong size of ivim_list!" << std::endl;
    exit(-1);
  }

  // TODO: if the old structures are not freed then the pointers will be overwritten, resulting in leaks
  for (int ivim = 0; ivim < ros_ivim_list->ivims.size(); ivim++) {
    // allocate attributes
    fillIVIM(ros_ivim_list->ivims[ivim], ivim_list_[ivim]);
  }

  new_data_received_ = true;
}

void IVIMHandler::fillIVIM(v2x_msgs::msg::IVIM ros_ivim, void* asn_ivim_void_ptr) {
  // reset data structure - TODO: free the pointers before memsetting to 0?
  IVIM* asn_ivim = (IVIM*) asn_ivim_void_ptr;
  memset((void *) asn_ivim_void_ptr, 0, sizeof(IVIM));

  // header
  asn_ivim->header.protocolVersion = ros_ivim.header.protocol_version; // V2 is most recent CDD header Q1 2022
  asn_ivim->header.messageID = ros_ivim.header.message_id;
  asn_ivim->header.stationID = ros_ivim.header.station_id.station_id;

  // ivi
  /// mandatory
  ////serviceProviderId
  //TODO: convert int country code to string https://eur-lex.europa.eu/resource.html?uri=cellar:9a2fe08f-4580-11e9-a8ed-01aa75ed71a1.0014.02/DOC_3&format=PDF page 29
  // maybe use int64_t_to_bit2
  _bit2 tmpCountryCode = IVIMUtils::int64_t_to_bit2(ros_ivim.ivi.mandatory.service_provider_id.country_code.country_code);
  asn_ivim->ivi.mandatory.serviceProviderId.countryCode.length = tmpCountryCode.length;
  asn_ivim->ivi.mandatory.serviceProviderId.countryCode.value = tmpCountryCode.value;
  asn_ivim->ivi.mandatory.serviceProviderId.providerIdentifier = ros_ivim.ivi.mandatory.service_provider_id.provider_identifier.issuer_identifier;

  ////iviIdentificationNumber
  asn_ivim->ivi.mandatory.iviIdentificationNumber = ros_ivim.ivi.mandatory.ivi_identification_number.ivi_identification_number;

  ////OPTIONAL: timeStamp
  if (ros_ivim.ivi.mandatory.time_stamp_present) {
    asn_ivim->ivi.mandatory.bit_mask |= IviManagementContainer_timeStamp_present;
    asn_ivim->ivi.mandatory.timeStamp = ros_ivim.ivi.mandatory.time_stamp.timestamp_its;
  }

  ////OPTIONAL: validFrom
  ////OPTIONAL: validTo
  ////OPTIONAL: connectedIviStructures

  ////iviStatus
  asn_ivim->ivi.mandatory.iviStatus = ros_ivim.ivi.mandatory.ivi_status.ivi_status;
  ////OPTIONAL: connectedDenms

  ///OPTIONAL: optional
  if (ros_ivim.ivi.optional_present) {
    asn_ivim->ivi.bit_mask |= optional_present;
    asn_ivim->ivi.optional = (IviContainers_ *) IVIMUtils::AllocateClearedMemory(sizeof(IviContainers_));

    auto asn_ivi_containers = asn_ivim->ivi.optional;
    auto &ros_ivi_containers = ros_ivim.ivi.optional.containers;
    for (int i = 0; i < ros_ivi_containers.size(); ++i) {
      auto &ros_ivi_container = ros_ivi_containers.at(i);

      if (i != 0){
        asn_ivi_containers->next = (IviContainers_ *) IVIMUtils::AllocateClearedMemory(sizeof(IviContainers_));
        asn_ivi_containers = asn_ivi_containers->next;
      }

      IVIMUtils::fillASNIviContainer(ros_ivi_container, &(asn_ivi_containers->value));
    }
  }

  if(ossPrintPDU((OssGlobal*)world_, IVIM_PDU, asn_ivim)){
    RCLCPP_INFO(GetNode()->get_logger(), "IVIM could not be printed");
  }
}

v2x_msgs::msg::IVIM IVIMHandler::GetROSIVIM(std::pair<void *, size_t> message) {

  v2x_msgs::msg::IVIM ros_ivim;
  IVIM *asn_ivim = nullptr;

  int ret_code;

  int pdu_num=IVIM_PDU;
  OssBuf buffer;
  buffer.length = message.second;
  buffer.value = (unsigned char*) message.first;

  // decode
  if ((ret_code = ossDecode((ossGlobal*)world_, &pdu_num, &buffer, (void**) &asn_ivim)) != 0) {
      RCLCPP_ERROR(GetNode()->get_logger(), "Decode error: %d with message %s", ret_code, ossGetErrMsg((OssGlobal*) world_));
  }

  auto &clk = *GetNode()->get_clock();
  RCLCPP_WARN_THROTTLE(GetNode()->get_logger(), clk, IVIM_DEBUG_MSG_THROTTLE_MS,
                       "Not all parts implemented for translation to ROS IVIM!");

  // header
  ros_ivim.header.protocol_version = asn_ivim->header.protocolVersion;
  ros_ivim.header.message_id = asn_ivim->header.messageID;
  ros_ivim.header.station_id.station_id = asn_ivim->header.stationID;

  // ivi
  /// mandatory

  ros_ivim.ivi.mandatory.service_provider_id.country_code.country_code = IVIMUtils::bitstream_to_int64(asn_ivim->ivi.mandatory.serviceProviderId.countryCode.length, asn_ivim->ivi.mandatory.serviceProviderId.countryCode.value); //TODO: check
  ros_ivim.ivi.mandatory.service_provider_id.provider_identifier.issuer_identifier = asn_ivim->ivi.mandatory.serviceProviderId.providerIdentifier;

  ros_ivim.ivi.mandatory.ivi_identification_number.ivi_identification_number = asn_ivim->ivi.mandatory.iviIdentificationNumber;

  if (asn_ivim->ivi.mandatory.bit_mask & IviManagementContainer_timeStamp_present) {
    ros_ivim.ivi.mandatory.time_stamp_present = true;
    ros_ivim.ivi.mandatory.time_stamp.timestamp_its = asn_ivim->ivi.mandatory.timeStamp;
  }

  ros_ivim.ivi.mandatory.ivi_status.ivi_status = asn_ivim->ivi.mandatory.iviStatus;

  /// optional
  if (asn_ivim->ivi.optional) {
    ros_ivim.ivi.optional_present = true;

    IviContainers_* asn_ivi_containers = asn_ivim->ivi.optional;
    auto &ros_ivi_containers = ros_ivim.ivi.optional.containers;

    while(asn_ivi_containers != nullptr){
      auto  ros_ivi_container = IVIMUtils::GetROSIviContainer(&asn_ivi_containers->value);
      ros_ivi_containers.push_back(ros_ivi_container);
      asn_ivi_containers = asn_ivi_containers->next;
    }
  }

  return ros_ivim;
}