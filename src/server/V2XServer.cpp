#include<netdb.h>	//hostent
#include<arpa/inet.h>

#include "V2XServer.h"

V2XServer::V2XServer(rclcpp::Node *gateway_node, std::map<MsgType, V2XMHandler *> v2x_m_handler) {
    // params
    gateway_node_ = gateway_node;
    v2x_m_handler_ = v2x_m_handler;

    // init
    is_active_ = false;
    is_configured_ = false;
    is_connected_ = false;

    // init client status
    client_received_messages_ = 0;
    client_sent_messages_ = 0;
    client_last_message_received_ = rclcpp::Time(0);
    client_last_message_sent_ = rclcpp::Time(0);

    // init server status
    server_heartbeat_counter_ = 0;
    server_received_messages_ = 0;
    server_sent_messages_ = 0;
    server_connection_data_ = "not yet set";
    server_heartbeat_message_ = "not yet set";
    client_last_message_received_ = rclcpp::Time(0);
    client_last_message_sent_ = rclcpp::Time(0);
}

V2XServer::~V2XServer() {}


rclcpp::Node *V2XServer::GetNode() {
    return gateway_node_;
}

void V2XServer::Process(void) {
    // first receive, then send

    // receive all messages
    std::queue <std::pair<void *, size_t>> incoming = ReceiveMessages();
    if (v2x_m_handler_.size() == 0) {
        RCLCPP_ERROR(GetNode()->get_logger(), "Configuration error! There are no handlers for messages. This will crash!");
    }
    if (v2x_m_handler_.count(MsgType::kNone) != 1) {
        RCLCPP_ERROR(GetNode()->get_logger(), "Configuration error! Please add the MsgType::kNone handler to be used for decoding");
    }

    // handle each incoming message
    std::map<MsgType, std::queue<std::pair<void *, size_t>>> msg_mapping;
    MsgType msg_type = MsgType::kNone;
    while (!incoming.empty()) {
        std::pair<void *, size_t> msg =  incoming.front();
        msg_type = v2x_m_handler_[MsgType::kNone]->GetMessageType(msg);
        msg_mapping[msg_type].push(msg);
        incoming.pop();
    }
    for (auto const& elem : msg_mapping) {
        if (v2x_m_handler_.count(elem.first) > 0) // since C++20: if (v2x_m_handler_.contains(elem.first))
            v2x_m_handler_[elem.first]->PutMessages(elem.second);
        else
            RCLCPP_WARN(GetNode()->get_logger(), ("Configuration Warning! Please add the MsgType::int(" + std::to_string(static_cast<int>(elem.first)) + ") handler to be used for decoding.").c_str());
    }

    // collect outgoing messages
    std::queue <std::pair<void *, size_t>> outgoing;
    for (auto const& handler : v2x_m_handler_) {
        if(handler.first != MsgType::kNone) {
            std::queue <std::pair<void *, size_t>> msgs_to_send = handler.second->GetMessages();
            while (!msgs_to_send.empty()) {
                outgoing.push(msgs_to_send.front());
                msgs_to_send.pop();
            }
        }
    }
    SendMessages(outgoing);
}

std::string V2XServer::GetIPAddressFromHostname(std::string hostname)
{
    struct hostent *hostent_struct;
    struct in_addr **addr_list;

    std::string ip_address = "";

    if( (hostent_struct = gethostbyname( hostname.c_str()) ) != NULL) {
        addr_list = (struct in_addr **) hostent_struct->h_addr_list;
        if( addr_list[0] != NULL) {
            ip_address = std::string( inet_ntoa(*addr_list[0]) );
        }
    }

    return ip_address;
}

bool V2XServer::VerifyIPv4Address(std::string ip_address) {
    struct sockaddr_in socket_address;

    return inet_pton(AF_INET, ip_address.c_str(), &(socket_address)) == 1;
}