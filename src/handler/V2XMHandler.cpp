
#include "V2XMHandler.h"

extern "C" {
#include "vcits/cam/ItsPduHeader.h"

#include "vcits/exceptions/V2XLibExceptions.h"

#include "vcits/parser/Decoder.h"
}

V2XMHandler::V2XMHandler(MsgType msg_type = MsgType::kNone, rclcpp::Node *gateway_node = nullptr) {
    // params
    msg_type_ = msg_type;
    gateway_node_ = gateway_node;

    // init
    is_active_ = false;
    is_configured_ = false;
}

V2XMHandler::~V2XMHandler() {}

// public methods
MsgType V2XMHandler::GetMessageType(std::pair<void *, size_t> msg){
    MsgType msg_type = MsgType::kNone;
    ItsPduHeader_t *its_pdu_header = nullptr;

    try {
        its_pdu_header = (ItsPduHeader_t *) Decoder::decode(&asn_DEF_ItsPduHeader, msg.first, msg.second);

        msg_type = MsgType(its_pdu_header->messageID);

        ASN_STRUCT_FREE(asn_DEF_ItsPduHeader, its_pdu_header);
        its_pdu_header = nullptr;
    } catch (DecodingException e) {
        RCLCPP_INFO(GetNode()->get_logger(), e.what());
    }
    return msg_type;
}

// protected methods
uint64_t V2XMHandler::GetGenerationDeltaTime() {
    rclcpp::Time current_time = gateway_node_->get_clock()->now();
    uint64_t current_epoch_ms = (current_time.nanoseconds() / 1000) / 1000;

    return GetGenerationDeltaTime(current_epoch_ms);
}

uint64_t V2XMHandler::GetGenerationDeltaTime(uint64_t milliseconds) {
//    ETSI EN 302 637-2 V1.3.1 (Annex B3)
//      - Time corresponding to the time of the reference position in the CAM, considered as time of the CAM generation.
//          -> meaning the timestamp of the situation, i.e., the exact time matching the GPS position, when the CAM was generated
//          -> in other words: the timestamp matching the GPS position
//      - calculation: generationDeltaTime = TimestampIts mod 65 536
//          -> TimestampIts represents an integer value in milliseconds since 2004-01-01T00:00:00:000Z as defined in ETSI TS 102 894-2 [2].
//              ... TimestampIts = zulu_time_now - 2004-01-01T00:00:00:000Z
    rclcpp::Time current_time = gateway_node_->get_clock()->now();
    uint64_t current_epoch_ms = (current_time.nanoseconds() / 1000) / 1000;
    uint64_t its_time_ms = 1072915200; // time to ITS time since epoch in MS https://www.epochconverter.com/

    if (current_epoch_ms < its_time_ms)
        std::cout << "ros epoch time should not be smaller than 2004! bad things will happen now!" << std::endl;

    uint64_t timestamp_its = current_epoch_ms - its_time_ms;

    return timestamp_its % 65536;
}

rclcpp::Node *V2XMHandler::GetNode() {
    return gateway_node_;
}

double V2XMHandler::GetFrequency(rclcpp::Time A, rclcpp::Time B) {
    return (1.0/abs((A - B).seconds()));
}