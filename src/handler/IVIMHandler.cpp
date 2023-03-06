
#include "IVIMHandler.h"

#include <sstream>

extern "C" {

#include "vcits/ivim/IviContainers.h"
#include "vcits/ivim/IviContainer.h"
#include "vcits/ivim/GlcParts.h"
#include "vcits/ivim/GlcPart.h"
#include "vcits/ivim/Zone.h"
#include "vcits/ivim/DeltaPositions.h"
#include "vcits/ivim/DeltaPosition.h"
#include "vcits/ivim/TextContainer.h"
#include "vcits/ivim/TcPart.h"
#include "vcits/ivim/ZoneIds.h"
#include "vcits/ivim/Zid.h"
#include "vcits/ivim/LanePositions.h"
#include "vcits/ivim/LanePosition.h"
#include "vcits/ivim/TextLines.h"
#include "vcits/ivim/Text.h"

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
  // reset data structure (free the pointers?)
  memset((void *) asn_ivim, 0, sizeof(IVIM_t));

  // header
  asn_ivim->header.protocolVersion = 2; // V2 is most recent CDD header Q1 2022
  asn_ivim->header.messageID = 6;
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
    // TODO: missing asn definitions
  }
}

v2x_msgs::msg::IVIM IVIMHandler::GetROSIVIM(std::pair<void *, size_t> message) {
  // variables
  v2x_msgs::msg::IVIM ros_ivim;
  IVIM_t *asn_ivim = nullptr;

  // decode
  try {
    asn_ivim = (IVIM_t *) Decoder::decode(&asn_DEF_IVIM, message.first, message.second);
  } catch (DecodingException e) {
    RCLCPP_ERROR(GetNode()->get_logger(), e.what());
    RCLCPP_INFO(GetNode()->get_logger(),
                "If decoding fails, we throw away everything, as we would have to check how far we were able to decode");
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

    auto& asn_ivi_containers = asn_ivim->ivi.optional->list;
    auto& ros_ivi_containers = ros_ivim.ivi.optional.containers;

    for (int i = 0; i < asn_ivi_containers.count; ++i) {
      auto asn_ivi_container = asn_ivi_containers.array[i];
      auto  ros_ivi_container = GetROSIviContainer(asn_ivi_container);

      ros_ivi_containers.push_back(ros_ivi_container);
    }
  }

  return ros_ivim;
}

v2x_msgs::msg::IviContainer IVIMHandler::GetROSIviContainer(IviContainer_t* asn_ivi_container) {

  v2x_msgs::msg::IviContainer ros_ivi_container;

  switch (asn_ivi_container->present) {

    case IviContainer_PR_NOTHING: {
      ros_ivi_container.ivi_container_container_select = ros_ivi_container.IVI_CONTAINER_NOTHING;
    }
    break;

    case 	IviContainer_PR_glc: {
      ros_ivi_container.ivi_container_container_select = ros_ivi_container.IVI_CONTAINER_GLC;

      auto &asn_glc = asn_ivi_container->choice.glc;
      auto &ros_glc = ros_ivi_container.glc;

      ros_glc.reference_position.latitude.latitude = asn_glc.referencePosition.latitude;
      ros_glc.reference_position.longitude.longitude = asn_glc.referencePosition.longitude;

      ros_glc.reference_position.position_confidence_ellipse.semi_major_confidence.semi_axis_length = asn_glc.referencePosition.positionConfidenceEllipse.semiMajorConfidence;
      ros_glc.reference_position.position_confidence_ellipse.semi_minor_confidence.semi_axis_length = asn_glc.referencePosition.positionConfidenceEllipse.semiMinorConfidence;
      ros_glc.reference_position.position_confidence_ellipse.semi_major_orientation.heading_value = asn_glc.referencePosition.positionConfidenceEllipse.semiMajorOrientation;

      ros_glc.reference_position.altitude.altitude_value.altitude_value = asn_glc.referencePosition.altitude.altitudeValue;
      ros_glc.reference_position.altitude.altitude_confidence.altitude_confidence = asn_glc.referencePosition.altitude.altitudeConfidence;

      if (asn_glc.referencePositionTime) {
        ros_glc.reference_position_time_present = true;
        if (asn_INTEGER2imax(asn_glc.referencePositionTime, &ros_glc.reference_position_time.timestamp_its) == -1) {
          // TODO: handle error
        }
      }

      if (asn_glc.referencePositionHeading) {
        ros_glc.reference_position_heading_present = false;
        // not implemented
      }

      if (asn_glc.referencePositionSpeed) {
        ros_glc.reference_position_speed_present = false;
        // not implemented
      }

      auto &asn_glc_parts = asn_glc.parts.list;
      auto &ros_glc_parts = ros_glc.parts.parts;

      for (int i = 0; i < asn_glc_parts.count; ++i) {
        auto asn_glc_part = asn_glc_parts.array[i];
        auto ros_glc_part = GetROSGlcPart(asn_glc_part);

        ros_glc_parts.push_back(ros_glc_part);
      }
    }
    break;

    case IviContainer_PR_tc: {
      ros_ivi_container.ivi_container_container_select = ros_ivi_container.IVI_CONTAINER_TC;

      auto &asn_tc = asn_ivi_container->choice.tc;
      auto &ros_tc = ros_ivi_container.tc;

      auto &asn_tc_parts = asn_tc.list;
      auto &ros_tc_parts = ros_tc.container;

      for (int i = 0; i < asn_tc_parts.count; ++i) {
        auto asn_tc_part = asn_tc_parts.array[i];
        auto ros_tc_part = GetROSTcPart(asn_tc_part);

        ros_tc_parts.push_back(ros_tc_part);
      }

    }
    break;

    default: {
      RCLCPP_INFO(GetNode()->get_logger(),
                  "This ivi container is not implemented: %u", asn_ivi_container->present);
    }
    break;
  }

  return ros_ivi_container;
}


v2x_msgs::msg::GlcPart IVIMHandler::GetROSGlcPart(GlcPart_t* asn_glc_part) {

  v2x_msgs::msg::GlcPart ros_glc_part;

  ros_glc_part.zone_id.zid = asn_glc_part->zoneId;

  if (asn_glc_part->laneNumber) {
    ros_glc_part.lane_number_present = true;
    ros_glc_part.lane_number.lane_position = *asn_glc_part->laneNumber;
  }

  if (asn_glc_part->zoneExtension) {
    ros_glc_part.zone_extension_present = true;
    ros_glc_part.zone_extension = *asn_glc_part->zoneExtension;
  }

  if (asn_glc_part->zoneHeading) {
    ros_glc_part.zone_heading_present = true;
    ros_glc_part.zone_heading.heading_value = *asn_glc_part->zoneHeading;
  }

  if (asn_glc_part->zone) {
    ros_glc_part.zone_present = true;
    ros_glc_part.zone = GetROSZone(asn_glc_part->zone);
  }

  return ros_glc_part;
}

v2x_msgs::msg::Zone IVIMHandler::GetROSZone(Zone_t* asn_zone) {

  v2x_msgs::msg::Zone ros_zone;

  switch (asn_zone->present) {

    case Zone_PR_NOTHING: {
      ros_zone.zone_container_select = ros_zone.ZONE_NOTHING;
    }
    break;

    case Zone_PR_segment: {
      ros_zone.zone_container_select = ros_zone.ZONE_SEGMENT;

      auto &asn_segment_zone = asn_zone->choice.segment;
      auto &ros_segment_zone = ros_zone.segment;

      ros_segment_zone.line = GetROSPolygonalLine(&asn_segment_zone.line);

      if (asn_segment_zone.laneWidth) {
        ros_segment_zone.lane_width_present = true;
        ros_segment_zone.lane_width.ivi_lane_width = *asn_segment_zone.laneWidth;
      }
    }
    break;

    default: {
      RCLCPP_INFO(GetNode()->get_logger(),
                  "This zone is not implemented: %u", asn_zone->present);
    }
    break;
  }

  return ros_zone;
}

v2x_msgs::msg::PolygonalLine IVIMHandler::GetROSPolygonalLine(PolygonalLine_t* asn_polygonal_line)
{
  v2x_msgs::msg::PolygonalLine ros_polygonal_line;

  switch(asn_polygonal_line->present) {

    case PolygonalLine_PR_NOTHING: {
      ros_polygonal_line.polygonal_line_container_select = ros_polygonal_line.POLYGONAL_LINE_NOTHING;
    }
    break;

    case PolygonalLine_PR_deltaPositions: {
      ros_polygonal_line.polygonal_line_container_select = ros_polygonal_line.POLYGONAL_LINE_DELTA_POSITIONS;

      auto &asn_polygonal_line_delta_positions = asn_polygonal_line->choice.deltaPositions;
      auto &ros_polygonal_line_delta_positions = ros_polygonal_line.delta_positions;

      auto &asn_delta_positions = asn_polygonal_line_delta_positions.list;
      auto &ros_delta_positions = ros_polygonal_line_delta_positions.positions;

      for (int i = 0; i < asn_delta_positions.count; ++i) {
        auto asn_delta_position = asn_delta_positions.array[i];
        auto ros_delta_position = GetROSDeltaPosition(asn_delta_position);

        ros_delta_positions.push_back(ros_delta_position);
      }
    }
    break;

    default: {
      RCLCPP_INFO(GetNode()->get_logger(),
                  "This polygonal line is not implemented: %u", asn_polygonal_line->present);
    }
    break;
  }

  return ros_polygonal_line;
}

v2x_msgs::msg::DeltaPosition IVIMHandler::GetROSDeltaPosition(DeltaPosition_t* asn_delta_position) {

  v2x_msgs::msg::DeltaPosition ros_delta_position;

  ros_delta_position.delta_latitude.delta_latitude = asn_delta_position->deltaLatitude;
  ros_delta_position.delta_longitude.delta_longitude = asn_delta_position->deltaLongitude;

  return ros_delta_position;
}

v2x_msgs::msg::TcPart IVIMHandler::GetROSTcPart(TcPart_t* asn_tc_part) {

  v2x_msgs::msg::TcPart ros_tc_part;

  if (asn_tc_part->detectionZoneIds) {
    ros_tc_part.detection_zone_ids_present = true;

    auto &asn_direction_zone_ids = asn_tc_part->detectionZoneIds->list;
    auto &ros_direction_zone_ids = ros_tc_part.detection_zone_ids.ids;

    for (int i = 0; i < asn_direction_zone_ids.count; ++i) {
      auto asn_direction_zone_id = asn_direction_zone_ids.array[i];
      auto ros_direction_zone_id = GetROSZoneId(asn_direction_zone_id);

      ros_direction_zone_ids.push_back(ros_direction_zone_id);
    }
  }

  auto &asn_relevance_zone_ids = asn_tc_part->relevanceZoneIds.list;
  auto &ros_relevance_zone_ids = ros_tc_part.relevance_zone_ids.ids;

  for (int i = 0; i < asn_relevance_zone_ids.count; ++i) {
    auto asn_relevance_zone_id = asn_relevance_zone_ids.array[i];
    auto ros_relevance_zone_id = GetROSZoneId(asn_relevance_zone_id);

    ros_relevance_zone_ids.push_back(ros_relevance_zone_id);
  }

  if (asn_tc_part->direction) {
    ros_tc_part.direction_present = true;
    ros_tc_part.direction.direction = *asn_tc_part->direction;
  }

  if (asn_tc_part->driverAwarenessZoneIds) {
    ros_tc_part.driver_awareness_zone_ids_present = true;

    auto &asn_driver_awareness_zone_ids = asn_tc_part->driverAwarenessZoneIds->list;
    auto &ros_driver_awareness_zone_ids = ros_tc_part.driver_awareness_zone_ids.ids;

    for (int i = 0; i < asn_driver_awareness_zone_ids.count; ++i) {
      auto asn_driver_awareness_zone_id = asn_driver_awareness_zone_ids.array[i];
      auto ros_driver_awareness_zone_id = GetROSZoneId(asn_driver_awareness_zone_id);

      ros_driver_awareness_zone_ids.push_back(ros_driver_awareness_zone_id);
    }
  }

  if (asn_tc_part->minimumAwarenessTime) {
    ros_tc_part.minimum_awareness_time_present = true;
    ros_tc_part.minimum_awareness_time = *asn_tc_part->minimumAwarenessTime;
  }

  if (asn_tc_part->applicableLanes) {
    ros_tc_part.applicable_lanes_present = true;

    auto &asn_applicable_lanes = asn_tc_part->applicableLanes->list;
    auto &ros_applicable_lanes = ros_tc_part.applicable_lanes.positions;

    for (int i = 0; i < asn_applicable_lanes.count; ++i) {
      auto asn_applicable_lane = asn_applicable_lanes.array[i];
      auto ros_applicable_lane = GetROSLanePosition(asn_applicable_lane);

      ros_applicable_lanes.push_back(ros_applicable_lane);
    }
  }

  if (asn_tc_part->layoutId) {
    ros_tc_part.layout_id_present = true;
    ros_tc_part.layout_id = *asn_tc_part->layoutId;
  }

  if (asn_tc_part->preStoredlayoutId) {
    ros_tc_part.pre_storedlayout_id_present = true;
    ros_tc_part.pre_storedlayout_id = *asn_tc_part->preStoredlayoutId;
  }

  if (asn_tc_part->text) {
    ros_tc_part.text_present = true;

    auto &asn_text_lines = asn_tc_part->text->list;
    auto &ros_text_lines = ros_tc_part.text.lines;

    for (int i = 0; i < asn_text_lines.count; ++i) {
      auto asn_text = asn_text_lines.array[i];
      auto ros_text = GetROSText(asn_text);

      ros_text_lines.push_back(ros_text);
    }
  }

// TODO: asn_tc_part.data - how do i translate an OCTET_STRING ?

  if (asn_tc_part->ext1) {
    ros_tc_part.ivi_type.ivi_type = asn_tc_part->ext1->iviType;

    if (asn_tc_part->ext1->laneStatus) {
      ros_tc_part.lane_status_present = true;
      ros_tc_part.lane_status.lane_status =  *asn_tc_part->ext1->laneStatus;
    }

    if (asn_tc_part->ext1->vehicleCharacteristics) {
      ros_tc_part.vehicle_characteristics_present = false;
      // not implemented
    }
  }

  return ros_tc_part;
}

v2x_msgs::msg::Zid IVIMHandler::GetROSZoneId(Zid_t* asn_zone_id) {
  v2x_msgs::msg::Zid ros_zone_id;
  ros_zone_id.zid = *asn_zone_id;
  return ros_zone_id;
}

v2x_msgs::msg::LanePosition IVIMHandler::GetROSLanePosition(LanePosition_t* asn_lane_position) {
  v2x_msgs::msg::LanePosition ros_lane_position;
  ros_lane_position.lane_position = *asn_lane_position;
  return ros_lane_position;
}

v2x_msgs::msg::Text IVIMHandler::GetROSText(Text_t* asn_text) {

  v2x_msgs::msg::Text ros_text;

  if (asn_text->layoutComponentId) {
    ros_text.layout_component_id_present = true;
    ros_text.layout_component_id = *asn_text->layoutComponentId;
  }

//  TODO: ros_text.language - issue : how do i translate the asn BIT_STRING ?

//  TODO: ros_text.text_content - issue : how do i translate the asn UTF8String ?

  return ros_text;
}