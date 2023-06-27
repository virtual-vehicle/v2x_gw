
#include "DENMHandler.h"

#include <sstream>

#include <v2x_msgs/msg/default_validity.hpp>

extern "C" {
#include "vcits/denm/SituationContainer.h"
#include "vcits/denm/EventHistory.h"
#include "vcits/denm/EventPoint.h"

#include "vcits/exceptions/V2XLibExceptions.h"

#include "vcits/parser/Decoder.h"
#include "vcits/parser/Encoder.h"
}

DENMHandler::DENMHandler(rclcpp::Node *gateway_node)
        : V2XMHandler(MsgType::kDENM, gateway_node) {

    // allocate attributes
    denm_ = (DENM_t *) malloc(
            sizeof(DENM_t));  // we could check on nullptr ... however, if this fails, we have other problems (out of memory)

    // configure
    ReadConfig();

    // init denm structure
    InitDENM();

    // publisher
    denm_pub_ = GetNode()->create_publisher<v2x_msgs::msg::DENMList>("denm/received", 1);

    // subscriber
}

DENMHandler::~DENMHandler() {
    // free the denm structure
    ASN_STRUCT_FREE(asn_DEF_DENM, denm_);
    denm_ = nullptr;
}

std::vector<diagnostic_msgs::msg::KeyValue> DENMHandler::GetDiagnostics() {
    std::vector<diagnostic_msgs::msg::KeyValue> values;
    diagnostic_msgs::msg::KeyValue key_value;

    // status - general
    key_value.key = "v2x_handler.denm.is_active_";
    key_value.value = std::to_string(is_active_);
    values.push_back(key_value);
    key_value.key = "v2x_handler.denm.is_configured_";
    key_value.value = std::to_string(is_configured_);
    values.push_back(key_value);

    // status - messages
    key_value.key = "v2x_handler.denm.message_received_counter_";
    key_value.value = std::to_string(message_received_counter_);
    values.push_back(key_value);
    key_value.key = "v2x_handler.denm.message_sent_counter_";
    key_value.value = std::to_string(message_sent_counter_);
    values.push_back(key_value);

    return values;
}

std::queue <std::pair<void *, size_t>> DENMHandler::GetMessages() {

    // processing variables
    rclcpp::Time current_timestamp = GetNode()->get_clock()->now();
    std::queue <std::pair<void *, size_t>> denm_queue; // there may be 0..N DENMs in the queue

    // static variables
    static rclcpp::Time prev_process_timestamp = current_timestamp;

    // max frequency check -> DENM specs <= 10Hz
    if (GetFrequency(current_timestamp, prev_process_timestamp) <= 10.0) {

        auto &clk = *GetNode()->get_clock();
        //RCLCPP_INFO_THROTTLE(GetNode()->get_logger(), clk, DENM_DEBUG_MSG_THROTTLE_MS,
        //                     "GetMessages(): not yet implemented for DENM");

        // TODO implement DENM relaying, do it similar to CAM, when its changed to the Autoware interface
        // TODO also think about: (i) denms being sent out multiple times with specific frequency, (ii) denms being sent out first time, (iii) multiple, different denms
        // TODO -> who is the trigger, who is in control of sending ... i.e. similar with CAM sending

        prev_process_timestamp = current_timestamp;
    }

    // diagnostics
    message_sent_counter_ += denm_queue.size();

    return denm_queue;
}

void DENMHandler::PutMessages(std::queue <std::pair<void *, size_t>> msgs) {
    // variables
    v2x_msgs::msg::DENMList denm_list;
    denm_list.header.stamp = GetNode()->get_clock()->now();

    // take each element from the incoming_queue, convert it to ROS2 format and publish
    while (!msgs.empty()) {
        denm_list.denms.push_back(GetROSDENM(msgs.front()));
        msgs.pop();
    }

    // diagnostics
    message_received_counter_ += denm_list.denms.size();

    denm_pub_->publish(denm_list);
}

void DENMHandler::ReadConfig() {
    // read configuration and set is_configured_

    // necessary handler processing constants
    GetNode()->declare_parameter("denm.active", false);
    GetNode()->get_parameter("denm.active", DENM_ACTIVE);
    GetNode()->declare_parameter("denm.handler_debug_msg_throttle_ms", 5000);
    GetNode()->get_parameter("denm.handler_debug_msg_throttle_ms", DENM_DEBUG_MSG_THROTTLE_MS);

    if (DENM_ACTIVE) {
        is_active_ = true;

        // necessary header constants
        GetNode()->declare_parameter("header.station_id", 0);
        GetNode()->get_parameter("header.station_id", DENM_HEADER_STATION_ID);

        // necessary DENM constants

        // constants that should be set by code

    }

    is_configured_ = true;
}

// Callbacks


// DENM generation
void DENMHandler::InitDENM() {
    // reset data structure
    memset((void *) denm_, 0, sizeof(DENM_t));

    // set header
    denm_->header.protocolVersion = 2; // V2 is most recent CDD header Q1 2022
    denm_->header.messageID = 1; // DENM
    denm_->header.stationID = DENM_HEADER_STATION_ID;

    //ManagementContainer
    denm_->denm.management; //TODO WARNING: undefined parameters may lead to validation/encoding issues

    //SituationContainer
    denm_->denm.situation = nullptr; // NOT SET;

    //LocationContainer
    denm_->denm.location = nullptr; // NOT SET;

    //AlacarteContainer
    denm_->denm.alacarte = nullptr; // NOT SET;
}

v2x_msgs::msg::DENM DENMHandler::GetROSDENM(std::pair<void *, size_t> msg) {
    // variables
    v2x_msgs::msg::DENM ros_denm;
    DENM_t *asn_denm = nullptr;
    unsigned long tmp_timestamp; // needed for shoving ulong timestamp into int64

    // decode
    try {
        asn_denm = (DENM_t *) Decoder::decode(&asn_DEF_DENM, msg.first, msg.second);
    } catch (DecodingException e) {
        RCLCPP_ERROR(GetNode()->get_logger(), e.what());
        RCLCPP_INFO(GetNode()->get_logger(),
                    "If decoding fails, we throw away everything, as we would have to check how far we were able to decode");
    }

    auto &clk = *GetNode()->get_clock();
    RCLCPP_WARN_THROTTLE(GetNode()->get_logger(), clk, DENM_DEBUG_MSG_THROTTLE_MS,
                         "Not all parts implemented for translation to ROS DENM! -> search for TODO");

    // convert from asn_denm to ros_denm
    //Header
    ros_denm.header.protocol_version = asn_denm->header.protocolVersion;
    ros_denm.header.message_id = asn_denm->header.messageID;
    ros_denm.header.station_id.station_id = asn_denm->header.stationID;

    //ManagementContainer
    ros_denm.denm.management.action_id.originating_station_id.station_id = asn_denm->denm.management.actionID.originatingStationID;
    ros_denm.denm.management.action_id.sequence_number.sequence_number = asn_denm->denm.management.actionID.sequenceNumber;
    asn_INTEGER2ulong(&(asn_denm->denm.management.detectionTime), &tmp_timestamp);
    ros_denm.denm.management.detection_time.timestamp_its = tmp_timestamp;
    asn_INTEGER2ulong(&(asn_denm->denm.management.referenceTime), &tmp_timestamp);
    ros_denm.denm.management.detection_time.timestamp_its = tmp_timestamp;

    if (asn_denm->denm.management.termination != nullptr) {
        ros_denm.denm.management.termination_present = true;
        ros_denm.denm.management.termination.termination = *(asn_denm->denm.management.termination);
    }
    ros_denm.denm.management.event_position.latitude.latitude = asn_denm->denm.management.eventPosition.latitude;
    ros_denm.denm.management.event_position.longitude.longitude = asn_denm->denm.management.eventPosition.longitude;
    ros_denm.denm.management.event_position.position_confidence_ellipse.semi_major_confidence.semi_axis_length = asn_denm->denm.management.eventPosition.positionConfidenceEllipse.semiMajorConfidence;
    ros_denm.denm.management.event_position.position_confidence_ellipse.semi_minor_confidence.semi_axis_length = asn_denm->denm.management.eventPosition.positionConfidenceEllipse.semiMinorConfidence;
    ros_denm.denm.management.event_position.position_confidence_ellipse.semi_major_orientation.heading_value = asn_denm->denm.management.eventPosition.positionConfidenceEllipse.semiMajorOrientation;
    ros_denm.denm.management.event_position.altitude.altitude_value.altitude_value = asn_denm->denm.management.eventPosition.altitude.altitudeValue;
    ros_denm.denm.management.event_position.altitude.altitude_confidence.altitude_confidence = asn_denm->denm.management.eventPosition.altitude.altitudeConfidence;
    if (asn_denm->denm.management.relevanceDistance != nullptr) {
        ros_denm.denm.management.relevance_distance_present = true;
        ros_denm.denm.management.relevance_distance.relevance_distance = *(asn_denm->denm.management.relevanceDistance);
    }
    if (asn_denm->denm.management.relevanceTrafficDirection != nullptr) {
        ros_denm.denm.management.relevance_traffic_direction_present = true;
        ros_denm.denm.management.relevance_traffic_direction.relevance_traffic_direction = *(asn_denm->denm.management.relevanceTrafficDirection);
    }
    if (asn_denm->denm.management.validityDuration != nullptr) {
        ros_denm.denm.management.validity_duration.validity_duration = *(asn_denm->denm.management.validityDuration);
    }
    if (asn_denm->denm.management.transmissionInterval != nullptr) {
        ros_denm.denm.management.transmission_interval_present = true;
        ros_denm.denm.management.transmission_interval.transmission_interval = *(asn_denm->denm.management.transmissionInterval);
    }
    ros_denm.denm.management.station_type.station_type = asn_denm->denm.management.stationType;

    //SituationContainer
    if (asn_denm->denm.situation != nullptr) {
        ros_denm.denm.situation_present = true;
        ros_denm.denm.situation.information_quality.information_quality = asn_denm->denm.situation->informationQuality;
        ros_denm.denm.situation.event_type.cause_code.cause_code_type = asn_denm->denm.situation->eventType.causeCode;
        ros_denm.denm.situation.event_type.sub_cause_code.sub_cause_code_type = asn_denm->denm.situation->eventType.subCauseCode;
        if (asn_denm->denm.situation->linkedCause != nullptr) {
            ros_denm.denm.situation.linked_cause_present = true;
            ros_denm.denm.situation.linked_cause.cause_code.cause_code_type = asn_denm->denm.situation->linkedCause->causeCode;
            ros_denm.denm.situation.linked_cause.sub_cause_code.sub_cause_code_type = asn_denm->denm.situation->linkedCause->subCauseCode;
        }
        if (asn_denm->denm.situation->eventHistory != nullptr) {
            ros_denm.denm.situation.event_history_present = true;
            for (int i = 0; i < asn_denm->denm.situation->eventHistory->list.count; i++) {
                v2x_msgs::msg::EventPoint ep;
                ep.event_position.delta_latitude.delta_latitude;
                ep.event_position.delta_longitude.delta_longitude;
                ep.event_position.delta_altitude.delta_altitude;
                if (asn_denm->denm.situation->eventHistory->list.array[i]->eventDeltaTime != nullptr) {
                    ep.event_delta_time_present = true;
                    ep.event_delta_time.path_delta_time = *(asn_denm->denm.situation->eventHistory->list.array[i]->eventDeltaTime);
                }
                ep.information_quality.information_quality = asn_denm->denm.situation->eventHistory->list.array[i]->informationQuality;
                ros_denm.denm.situation.event_history.history.push_back(ep);
            }
        }
    }

    //TODO implementation of translation missing from here
    //RCLCPP_INFO(GetNode()->get_logger(), "GetROSDENM not yet fully implemented -> only important parts");

    //LocationContainer
    if (false) {
        ros_denm.denm.location_present = true;
    }

    //AlacarteContainer
    if (false) {
        ros_denm.denm.alacarte_present = true;
    }


    // free memory
    ASN_STRUCT_FREE(asn_DEF_DENM, asn_denm);
    asn_denm = nullptr;

    // return converted denm
    return ros_denm;
}

// for debug
void DENMHandler::PrintDENM() {
    std::ostringstream oss;

    oss << "--- --- --- DENM --- --- ---" << "\n";

    oss << "DENM" << "\n";
    oss << "|-header:" << "\n";
    oss << " |-protocolVersion: " << denm_->header.protocolVersion << "\n";
    oss << " |-messageID: " << denm_->header.messageID << "\n";
    oss << " |-stationID: " << denm_->header.stationID << "\n";

    RCLCPP_INFO(GetNode()->get_logger(), oss.str().c_str());
}
