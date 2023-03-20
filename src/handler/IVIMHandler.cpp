
#include "IVIMHandler.h"

extern "C" {
#include "vcits/exceptions/V2XLibExceptions.h"

#include "vcits/parser/Decoder.h"
#include "vcits/parser/Encoder.h"
}

IVIMHandler::IVIMHandler(rclcpp::Node *gateway_node)
        : V2XMHandler(MsgType::kIVIM, gateway_node) {

  // configure
  ReadConfig();
  new_data_received_ = false;

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
    ASN_STRUCT_FREE(asn_DEF_IVIM ,ivim_list_.back());
    ivim_list_.pop_back();
  }
}

std::queue <std::pair<void *, size_t>> IVIMHandler::GetMessages() {

  // processing variables
  rclcpp::Time current_timestamp = GetNode()->get_clock()->now(); // TODO: this is never used, why is it here? (also in CAMHandler)
  std::queue <std::pair<void *, size_t>> ivim_queue; // there will only be one CAM in the queue, despite the queue

  if(!new_data_received_){
    return ivim_queue;
  }

  // IVIM creation
  size_t final_ivim_size;
  void *final_ivim_buffer;

  for(IVIM_t* ivim : ivim_list_){

    try {
      Encoder::validate_constraints(&asn_DEF_IVIM, ivim);
    } catch (ValidateConstraintsException e) {
      RCLCPP_ERROR(GetNode()->get_logger(), e.what());
      continue;
    }

    try {
      final_ivim_size = Encoder::encode(&asn_DEF_IVIM, nullptr, ivim, &final_ivim_buffer);
    } catch (EncodingException e) {
      RCLCPP_ERROR(GetNode()->get_logger(), e.what());
      continue;
    }

    if (final_ivim_size <= 0) {
      RCLCPP_ERROR(GetNode()->get_logger(), "IVIM creation failed - you should probably stop the program");
      continue;
    }
    // TODO: comment from CAMHandler: put multiple cams in queue if receiving from carla. Is this relevant for IVIM?
    ivim_queue.push(std::make_pair(final_ivim_buffer, final_ivim_size));

    auto& clk = *GetNode()->get_clock();
    RCLCPP_INFO_THROTTLE(GetNode()->get_logger(), clk, IVIM_DEBUG_MSG_THROTTLE_MS, "IVIM created successfully with size %ld", final_ivim_size);
  }

  new_data_received_ = false;

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
  IVIM_t* new_ivim = (IVIM_t*)malloc(sizeof(IVIM_t));
  memset(new_ivim, 0, sizeof(IVIM_t));
  ivim_list_.push_back(new_ivim);
}

void IVIMHandler::removeIVIM() {
  ASN_STRUCT_FREE(asn_DEF_IVIM, ivim_list_.back());
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

void IVIMHandler::fillIVIM(v2x_msgs::msg::IVIM ros_ivim, IVIM_t* asn_ivim) {
  // reset data structure - TODO: free the pointers before memsetting to 0?
  memset((void *) asn_ivim, 0, sizeof(IVIM_t));

  // header
  asn_ivim->header.protocolVersion = ros_ivim.header.protocol_version; // V2 is most recent CDD header Q1 2022
  asn_ivim->header.messageID = ros_ivim.header.message_id;
  asn_ivim->header.stationID = ros_ivim.header.station_id.station_id;

  // ivi
  /// mandatory
//  asn_ivim->ivi.mandatory.serviceProviderId.countryCode = ros_ivim.ivi.mandatory.service_provider_id.country_code; // TODO: bitstream
  asn_ivim->ivi.mandatory.serviceProviderId.providerIdentifier = ros_ivim.ivi.mandatory.service_provider_id.provider_identifier.issuer_identifier;

  asn_ivim->ivi.mandatory.iviIdentificationNumber = ros_ivim.ivi.mandatory.ivi_identification_number.ivi_identification_number;

  if (ros_ivim.ivi.mandatory.time_stamp_present) {
    if (asn_imax2INTEGER(asn_ivim->ivi.mandatory.timeStamp, ros_ivim.ivi.mandatory.time_stamp.timestamp_its) == -1) {
      // TODO: handle error
    }
  }

  asn_ivim->ivi.mandatory.iviStatus = ros_ivim.ivi.mandatory.ivi_status.ivi_status;

  /// optional
  if (ros_ivim.ivi.optional_present) {
    asn_ivim->ivi.optional = (IviContainers_t *) IVIMUtils::AllocateClearedMemory(sizeof(IviContainers_t));

    auto asn_ivi_containers = &asn_ivim->ivi.optional->list;
    auto &ros_ivi_containers = ros_ivim.ivi.optional.containers;

    for (int i = 0; i < ros_ivi_containers.size(); ++i) {
      auto asn_ivi_container = (IviContainer_t *) IVIMUtils::AllocateClearedMemory(sizeof(IviContainer_t));
      auto &ros_ivi_container = ros_ivi_containers.at(i);

      IVIMUtils::fillASNIviContainer(ros_ivi_container, asn_ivi_container);

      if (ASN_SEQUENCE_ADD(asn_ivi_containers, asn_ivi_container)) {
        RCLCPP_ERROR(GetNode()->get_logger(), "ASN_SEQUENCE_ADD failed for asn_ivi_containers");
        exit(EXIT_FAILURE);
      }
    }
  }
}

v2x_msgs::msg::IVIM IVIMHandler::GetROSIVIM(std::pair<void *, size_t> message) {

  v2x_msgs::msg::IVIM ros_ivim;
  IVIM_t *asn_ivim = nullptr;

  // decode
  try {
    asn_ivim = (IVIM_t *) Decoder::decode(&asn_DEF_IVIM, message.first, message.second);
  } catch (DecodingException e) {
    RCLCPP_ERROR(GetNode()->get_logger(), e.what());
    RCLCPP_INFO(GetNode()->get_logger(),
                "If decoding fails, we throw away everything, as we would have to check how far we were able to decode");
    exit(-1); // TODO: should i exit?
  }

  auto &clk = *GetNode()->get_clock();
  RCLCPP_WARN_THROTTLE(GetNode()->get_logger(), clk, IVIM_DEBUG_MSG_THROTTLE_MS,
                       "Not all parts implemented for translation to ROS IVIM! -> search for TODO");

  // header
  ros_ivim.header.protocol_version = asn_ivim->header.protocolVersion;
  ros_ivim.header.message_id = asn_ivim->header.messageID;
  ros_ivim.header.station_id.station_id = asn_ivim->header.stationID;

  // ivi
  /// mandatory
//  ros_ivim.ivi.mandatory.service_provider_id.country_code = asn_ivim->ivi.mandatory.serviceProviderId.countryCode; // TODO: bitstring in IVIM_t
  ros_ivim.ivi.mandatory.service_provider_id.provider_identifier.issuer_identifier = asn_ivim->ivi.mandatory.serviceProviderId.providerIdentifier;

  ros_ivim.ivi.mandatory.ivi_identification_number.ivi_identification_number = asn_ivim->ivi.mandatory.iviIdentificationNumber;

  if (asn_ivim->ivi.mandatory.timeStamp) {
    ros_ivim.ivi.mandatory.time_stamp_present = true;
    if (asn_INTEGER2imax(asn_ivim->ivi.mandatory.timeStamp, &ros_ivim.ivi.mandatory.time_stamp.timestamp_its) == -1) {
      // TODO: handle error
    }
  }

  ros_ivim.ivi.mandatory.ivi_status.ivi_status = asn_ivim->ivi.mandatory.iviStatus;

  /// optional

  if (asn_ivim->ivi.optional) {
    ros_ivim.ivi.optional_present = true;

    auto &asn_ivi_containers = asn_ivim->ivi.optional->list;
    auto &ros_ivi_containers = ros_ivim.ivi.optional.containers;

    for (int i = 0; i < asn_ivi_containers.count; ++i) {
      auto asn_ivi_container = asn_ivi_containers.array[i];
      auto  ros_ivi_container = IVIMUtils::GetROSIviContainer(asn_ivi_container);

      ros_ivi_containers.push_back(ros_ivi_container);
    }
  }

  return ros_ivim;
}