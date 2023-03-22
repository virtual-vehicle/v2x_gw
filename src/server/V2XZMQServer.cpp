
#include "V2XZMQServer.h"

V2XZMQServer::V2XZMQServer(rclcpp::Node *gateway_node, std::map<MsgType, V2XMHandler *> v2x_m_handler)
        : V2XServer(gateway_node, v2x_m_handler) {

    // attributes
    std::string server_ip_address;

    // configure
    ReadConfig();

    // get/verify server ip address
    if(VerifyIPv4Address(SERVER_ADDRESS)) {
        server_ip_address = SERVER_ADDRESS;
        RCLCPP_INFO(GetNode()->get_logger(), "Got %s as server address. Verified as IPv4 address", SERVER_ADDRESS.c_str());
    }
    else {
        server_ip_address = GetIPAddressFromHostname(SERVER_ADDRESS);
        if(VerifyIPv4Address(server_ip_address))
            RCLCPP_INFO(GetNode()->get_logger(), "Got %s as server address. Using %s as IPv4 address.", SERVER_ADDRESS.c_str(), server_ip_address.c_str());
        else
            RCLCPP_WARN(GetNode()->get_logger(), "Got %s as server address. Cannot be translated to IPv4 address. Might not connect ...", SERVER_ADDRESS.c_str());
    }

    // create ZMQ context
    zmq_context_ = new zmq::context_t(1);

    // create ZMQ sockets
    zmq_recv_socket_ = new zmq::socket_t(*zmq_context_, ZMQ_SUB);
    zmq_snd_socket_ = new zmq::socket_t(*zmq_context_, ZMQ_PUB);

    // connect ZMQ sockets
    std::string zmq_recv_connect_string;
    std::string zmq_snd_connect_string;
    zmq_recv_connect_string.append("tcp://");
    zmq_snd_connect_string.append("tcp://");
    zmq_recv_connect_string.append(server_ip_address);
    zmq_snd_connect_string.append(server_ip_address);
    zmq_recv_connect_string.append(":");
    zmq_snd_connect_string.append(":");
    zmq_recv_connect_string.append(std::to_string(SERVER_RECEIVE_PORT));
    zmq_snd_connect_string.append(std::to_string(SERVER_SEND_PORT));

    RCLCPP_INFO(GetNode()->get_logger(), "V2XZMQServer receiver connecting to %s", zmq_recv_connect_string.c_str());
    zmq_recv_socket_->connect(zmq_recv_connect_string);
    RCLCPP_INFO(GetNode()->get_logger(), "V2XZMQServer sender connecting to %s", zmq_snd_connect_string.c_str());
    zmq_snd_socket_->connect(zmq_snd_connect_string);
    is_connected_ = true;

    // set ZMQ socket options
//    zmq_recv_socket_->set(zmq::sockopt::subscribe, SERVER_TOPIC_FILTER);

    // create timers
    timer_ = rclcpp::create_timer(GetNode(), GetNode()->get_clock(),
                                  rclcpp::Duration::from_nanoseconds(RCL_MS_TO_NS(SERVER_CYCLE_TIME_MS)),
                                  std::bind(&V2XZMQServer::Process, this));
}

V2XZMQServer::~V2XZMQServer() {

    // delete ZMQ sockets
    delete zmq_snd_socket_;
    delete zmq_recv_socket_;

    // delete ZMQ context
    delete zmq_context_;
}

bool V2XZMQServer::IsAlive() {
    RCLCPP_ERROR(GetNode()->get_logger(), "V2XZMQServer::IsAlive() not yet implemented");
    throw "NotImplemented";

    return false;
}

std::vector<diagnostic_msgs::msg::KeyValue> V2XZMQServer::GetDiagnostics() {
    std::vector<diagnostic_msgs::msg::KeyValue> values;
    diagnostic_msgs::msg::KeyValue key_value;

    // status - general
    key_value.key = "v2x_server.is_active";
    key_value.value = std::to_string(is_active_);
    values.push_back(key_value);
    key_value.key = "v2x_server.is_configured_";
    key_value.value = std::to_string(is_configured_);
    values.push_back(key_value);
    key_value.key = "v2x_server.is_connected_";
    key_value.value = std::to_string(is_connected_);
    values.push_back(key_value);

    // status - client
    key_value.key = "v2x_server.client_received_messages_";
    key_value.value = std::to_string(client_received_messages_);
    values.push_back(key_value);
    key_value.key = "v2x_server.client_sent_messages_";
    key_value.value = std::to_string(client_sent_messages_);
    values.push_back(key_value);
    key_value.key = "v2x_server.client_last_message_received_";
    key_value.value = std::to_string(client_last_message_received_.nanoseconds());
    values.push_back(key_value);
    key_value.key = "v2x_server.client_last_message_sent_";
    key_value.value = std::to_string(client_last_message_sent_.nanoseconds());
    values.push_back(key_value);

    // status - server
    key_value.key = "v2x_server.server_heartbeat_counter_";
    key_value.value = std::to_string(server_heartbeat_counter_);
    key_value.key = "v2x_server.server_heartbeat_message_";
    key_value.value = server_heartbeat_message_;
    values.push_back(key_value);
    key_value.key = "v2x_server.server_received_messages_";
    key_value.value = std::to_string(server_received_messages_);
    values.push_back(key_value);
    key_value.key = "v2x_server.server_sent_messages_";
    key_value.value = std::to_string(server_sent_messages_);
    values.push_back(key_value);
    key_value.key = "v2x_server.server_last_message_received_";
    key_value.value = std::to_string(server_last_message_received_.nanoseconds());
    values.push_back(key_value);
    key_value.key = "v2x_server.server_last_message_sent_";
    key_value.value = std::to_string(server_last_message_sent_.nanoseconds());
    values.push_back(key_value);

    return values;
}

std::queue <std::pair<void *, size_t>> V2XZMQServer::ReceiveMessages() {

    // variables
    bool isReceivingMessages = true;
    std::queue <std::pair<void *, size_t>> msgs;
    zmq::message_t topic;
    zmq::message_t msg;

    // receive topic and msg
    while (isReceivingMessages) {
        if (zmq_recv_socket_->recv(topic, zmq::recv_flags::dontwait)) {
            // we are receiving a topic (or at least some message)
            server_heartbeat_counter_++;

            if (zmq_recv_socket_->recv(msg, zmq::recv_flags::dontwait)) {

                // we got a topic and a message
                std::string topic_string = std::string(static_cast<char *>(topic.data()), topic.size());
                if(topic_string.rfind(SERVER_TOPIC_FILTER_V2X, 0) == 0) {
                    // we got a v2x message
                    isReceivingMessages = true;

                    // get memory for msg
                    unsigned char *msg_cpy = nullptr;
                    msg_cpy = (unsigned char *) malloc(sizeof(unsigned char) * msg.size());
                    if (msg_cpy == nullptr) {
                        // if we are out of memory here, there are bigger problems ... just return and crash ... or so ...
                        return msgs;
                    }

                    // copy message
                    memcpy(msg_cpy, msg.data(), msg.size());

                    // store message
                    msgs.push(std::make_pair(msg_cpy, msg.size()));

                    // update counter
                    client_received_messages_++;
                    client_last_message_received_ = GetNode()->get_clock()->now();
                } else if(topic_string.rfind(SERVER_TOPIC_FILTER_HB, 0) == 0){
                    // we got a heartbeat message
                    server_heartbeat_message_ = std::string(static_cast<char*>(msg.data()), msg.size());
                } else {
                    // we got a message from another topic ... we will just ignore it for now
                }
            } else {
                //something is wrong ... zmq broken ... will most likely crash at some point
            }
        } else {
            isReceivingMessages = false;
        }
    }

    return msgs;
}

void V2XZMQServer::SendMessages(std::queue <std::pair<void *, size_t>> msgs) {
    //variables
    zmq::message_t topic(SERVER_TOPIC_FILTER_V2X);
    zmq::message_t msg;

    while (!msgs.empty()) {
        // create v2x_msg
        msg = zmq::message_t(msgs.front().first, msgs.front().second);

        // free memory of msg
        free(msgs.front().first);

        // pop msg from queue
        msgs.pop();

        // topic may be extended with message type: currently "v2x.", better: "v2x.cam.", "v2x.denm.", etc.
        topic = zmq::message_t(SERVER_TOPIC_FILTER_V2X);


        // send msg
        zmq_snd_socket_->send(topic, zmq::send_flags::sndmore);
        zmq_snd_socket_->send(msg, zmq::send_flags::none);

        // update counter
        client_sent_messages_++;
        client_last_message_sent_ = GetNode()->get_clock()->now();
    }

}

// --- --- --- protected methods --- --- ---

void V2XZMQServer::ReadConfig() {
    // read configuration and set is_configured_
    GetNode()->declare_parameter("server.type", "none");
    GetNode()->get_parameter("server.type", SERVER_TYPE);

    if (SERVER_TYPE.compare("zmq") == 0) {
        is_active_ = true;

        GetNode()->declare_parameter("server.address", "not set");
        GetNode()->get_parameter("server.address", SERVER_ADDRESS);
        GetNode()->declare_parameter("server.receive_port", 0);
        GetNode()->get_parameter("server.receive_port", SERVER_RECEIVE_PORT);
        GetNode()->declare_parameter("server.send_port", 0);
        GetNode()->get_parameter("server.send_port", SERVER_SEND_PORT);
        GetNode()->declare_parameter("server.topic_filter_hb", "hb.");
        GetNode()->get_parameter("server.topic_filter", SERVER_TOPIC_FILTER_HB);
        GetNode()->declare_parameter("server.topic_filter_v2x", "v2x.");
        GetNode()->get_parameter("server.topic_filter", SERVER_TOPIC_FILTER_V2X);
        GetNode()->declare_parameter("server.cycle_time_ms", 100);
        GetNode()->get_parameter("server.cycle_time_ms", SERVER_CYCLE_TIME_MS);
    }

    is_configured_ = true;
}