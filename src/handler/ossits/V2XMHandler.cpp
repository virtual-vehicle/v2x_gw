
#include "V2XMHandler.h"

extern "C" {
#include "ossits/include/ossits/ossits.h"
}

V2XMHandler::V2XMHandler(MsgType msg_type = MsgType::kNone, rclcpp::Node *gateway_node = nullptr) {
    // params
    msg_type_ = msg_type;
    gateway_node_ = gateway_node;

    // init
    is_active_ = false;
    is_configured_ = false;

    // diagnostics
    message_received_counter_ = 0;
    message_sent_counter_ = 0;
    world_ = malloc(sizeof(OssGlobal));
}

V2XMHandler::~V2XMHandler() {
    ossterm((OssGlobal*) world_);
    free(world_);
}

// public methods
MsgType V2XMHandler::GetMessageType(std::pair<void *, size_t> msg){
    MsgType msg_type = MsgType::kNone;
    ItsPduHeader *its_pdu_header = NULL;
    int ret_code;

    int pdu_num=ItsPduHeader_PDU;
    
    if ((ret_code = ossDecode((ossGlobal*)world_, &pdu_num, (OssBuf*) msg.first, (void**) &its_pdu_header)) != 0) {
        RCLCPP_INFO(GetNode()->get_logger(), "Decode error: " + ret_code);
    }
    
    msg_type = MsgType(its_pdu_header->messageID);

    /* resets the memory*/
    ossFreePDU((ossGlobal*)world_, pdu_num, its_pdu_header);

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