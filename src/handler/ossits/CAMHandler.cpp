
#include "handler/CAMHandler.h"

#include <sstream>

extern "C" {
#include "ossits/include/ossits/ossits.h"
}

CAMHandler::CAMHandler(rclcpp::Node *gateway_node)
        : V2XMHandler(MsgType::kCAM, gateway_node) {

    // configure
    ReadConfig();
    new_data_received_ = false;

    world_ = malloc(sizeof(OssGlobal));

    int retcode;
    if (retcode = ossinit((OssGlobal*) world_, ITS_Container)) {
	    RCLCPP_ERROR(GetNode()->get_logger(), "ossinit error: %d with message %s", retcode, ossGetErrMsg((OssGlobal*) world_));
    }

    ossSetEncodingFlags((OssGlobal*)world_, DEBUGPDU);
    ossSetDecodingFlags((OssGlobal*)world_, DEBUGPDU);

    if(CAM_WRITE_TRACE_FILE){
        std::string trace_path_str="trace.out";
        char* trace_path = new char[trace_path_str.length() + 1];
        strcpy(trace_path, trace_path_str.c_str());
        ossOpenTraceFile((ossGlobal*)world_, trace_path);
    }
     

    // publisher
    cam_pub_ = GetNode()->create_publisher<v2x_msgs::msg::CAMList>("cam/received", 1);

    // subscriber
    ros_cam_sub_ = GetNode()->create_subscription<v2x_msgs::msg::CAMList>
            ("cam/transmitted", 10, std::bind(&CAMHandler::RosCAMCallback, this, std::placeholders::_1));
}

CAMHandler::~CAMHandler() {
    // free the cam structure
    while(cam_list_.size() > 0){
        //frees and removes last element of cam_list
        int ret_code;
        if ((ret_code = ossFreePDU((ossGlobal*)world_, CAM_PDU, cam_list_.back())) != 0) {
            RCLCPP_ERROR(GetNode()->get_logger(), "Free decoded error: %d", ret_code);
        }
        cam_list_.pop_back();
    }
    ossterm((ossGlobal*)world_);
    free(world_);
}

std::vector<diagnostic_msgs::msg::KeyValue> CAMHandler::GetDiagnostics() {
    std::vector<diagnostic_msgs::msg::KeyValue> values;
    diagnostic_msgs::msg::KeyValue key_value;

    // status - general
    key_value.key = "v2x_handler.cam.is_active_";
    key_value.value = std::to_string(is_active_);
    values.push_back(key_value);
    key_value.key = "v2x_handler.cam.is_configured_";
    key_value.value = std::to_string(is_configured_);
    values.push_back(key_value);

    // status - messages
    key_value.key = "v2x_handler.cam.message_received_counter_";
    key_value.value = std::to_string(message_received_counter_);
    values.push_back(key_value);
    key_value.key = "v2x_handler.cam.message_sent_counter_";
    key_value.value = std::to_string(message_sent_counter_);
    values.push_back(key_value);

    return values;
}

std::queue <std::pair<void *, size_t>> CAMHandler::GetMessages() {

    // processing variables
    rclcpp::Time current_timestamp = GetNode()->get_clock()->now();
    std::queue <std::pair<void *, size_t>> cam_queue; // there will only be one CAM in the queue, despite the queue

    if(!new_data_received_){
        return cam_queue;
    }

    
    // CAM creation
    OssBuf final_cam_buffer;
    int ret_code;

    cam_list_lock_.lock();
    for(void* cam_void_ptr : cam_list_){
        CAM* cam = (CAM*) cam_void_ptr;
        final_cam_buffer.value = NULL;  
        final_cam_buffer.length = 0;
        if ((ret_code = ossEncode((ossGlobal*)world_, CAM_PDU, cam, &final_cam_buffer)) != 0) {
            RCLCPP_ERROR(GetNode()->get_logger(), "CAM creation error: %d with message %s", ret_code, ossGetErrMsg((OssGlobal*) world_));
        }else{
            //TODO: put multiple cams in queue if receiving from carla
            cam_queue.push(std::make_pair(final_cam_buffer.value, final_cam_buffer.length));
            auto& clk = *GetNode()->get_clock();
            RCLCPP_INFO_THROTTLE(GetNode()->get_logger(), clk, CAM_DEBUG_MSG_THROTTLE_MS,
                                        "CAM created successfully with size %ld", final_cam_buffer.length);
            
        }
    }  
    new_data_received_ = false;
    cam_list_lock_.unlock();

    // diagnostics
    message_sent_counter_ += cam_queue.size();
    
    return cam_queue;
}

void CAMHandler::PutMessages(std::queue <std::pair<void *, size_t>> msgs) {
    // variables
    v2x_msgs::msg::CAMList cam_list;
    cam_list.header.stamp = GetNode()->get_clock()->now();

    // take each element from the incoming_queue, convert it to ROS2 format and publish
    while (!msgs.empty()) {
        cam_list.cams.push_back(GetROSCAM(msgs.front()));
        msgs.pop();
    }

    // diagnostics
    message_received_counter_ += cam_list.cams.size();

    cam_pub_->publish(cam_list);
}

void CAMHandler::ReadConfig() {
    // read configuration and set is_configured_

    // necessary handler processing constants
    GetNode()->declare_parameter("cam.active", false);
    GetNode()->get_parameter("cam.active", CAM_ACTIVE);
    GetNode()->declare_parameter("cam.handler_debug_msg_throttle_ms", 5000);
    GetNode()->get_parameter("cam.handler_debug_msg_throttle_ms", CAM_DEBUG_MSG_THROTTLE_MS);
    
    GetNode()->declare_parameter("cam.trace_file", false);
    GetNode()->get_parameter("cam.trace_file", CAM_WRITE_TRACE_FILE);

    
    if (CAM_ACTIVE) {
        is_active_ = true;
    }

    is_configured_ = true;
}

// Callbacks
void CAMHandler::RosCAMCallback(const v2x_msgs::msg::CAMList::SharedPtr ros_camlist) {
    cam_list_lock_.lock();
    while(cam_list_.size() < ros_camlist->cams.size()){
        RCLCPP_ERROR(GetNode()->get_logger(), "Malloced CAM with size %ld", sizeof(CAM));
        CAM* next_cam = (CAM *) malloc(sizeof(CAM));
        cam_list_.push_back(next_cam);
    }

    while(cam_list_.size() > ros_camlist->cams.size()){
        //frees and removes last element of cam_list TODO test
        int ret_code;
        if ((ret_code = ossFreePDU((ossGlobal*)world_, CAM_PDU, cam_list_.back())) != 0) {
            RCLCPP_ERROR(GetNode()->get_logger(), "Free decoded error: %d", ret_code);
        }
        cam_list_.pop_back();
    }

    if(cam_list_.size() != ros_camlist->cams.size()){
        std::cout<< "[ERROR] wrong size of cam_list!"<< std::endl;
        exit(-1);
    }

    for (int i = 0; i < ros_camlist->cams.size(); i++) {
        // allocate attributes
        fillCAM(ros_camlist->cams[i],cam_list_[i]);
    }

    new_data_received_ = true;
    
    cam_list_lock_.unlock();
}

// CAM generation
void CAMHandler::fillCAM(v2x_msgs::msg::CAM ros_cam, void* cam_void_ptr) {
    CAM* cam = (CAM*) cam_void_ptr;
    // reset data structure
    memset((void *) cam, 0, sizeof(CAM));

    // set header
    cam->header.protocolVersion = 2; // V2 is most recent CDD header Q1 2022
    cam->header.messageID = 2; // CAM
    cam->header.stationID = ros_cam.header.station_id.station_id;

    // Update the CAM with new data
    // BasicContainer
    cam->cam.generationDeltaTime = ros_cam.cam.generation_delta_time.generation_delta_time;
    cam->cam.camParameters.basicContainer.stationType = ros_cam.cam.cam_parameters.basic_container.station_type.station_type;
    cam->cam.camParameters.basicContainer.referencePosition.latitude = ros_cam.cam.cam_parameters.basic_container.reference_position.latitude.latitude; // position callback
    cam->cam.camParameters.basicContainer.referencePosition.longitude = ros_cam.cam.cam_parameters.basic_container.reference_position.longitude.longitude; // position callback
    cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence = ros_cam.cam.cam_parameters.basic_container.reference_position.position_confidence_ellipse.semi_major_confidence.semi_axis_length;
    cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence = ros_cam.cam.cam_parameters.basic_container.reference_position.position_confidence_ellipse.semi_minor_confidence.semi_axis_length;
    cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation = ros_cam.cam.cam_parameters.basic_container.reference_position.position_confidence_ellipse.semi_major_orientation.heading_value; // NOT SET
    cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue = ros_cam.cam.cam_parameters.basic_container.reference_position.altitude.altitude_value.altitude_value; // position callback
    cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence = AltitudeConfidence(ros_cam.cam.cam_parameters.basic_container.reference_position.altitude.altitude_confidence.altitude_confidence);
    
    // HighFrequencyContainer - needed for vehicles
    if(ros_cam.cam.cam_parameters.high_frequency_container.high_frequency_container_container_select = v2x_msgs::msg::HighFrequencyContainer::HIGH_FREQUENCY_CONTAINER_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY){
        cam->cam.camParameters.highFrequencyContainer.choice = basicVehicleContainerHighFrequency_chosen;

        cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.heading.headingValue = 
                ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.heading.heading_value.heading_value;
        cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.heading.headingConfidence = 
                ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.heading.heading_confidence.heading_confidence;
        cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.speed.speedValue = 
                ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.speed.speed_value.speed_value;
        cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.speed.speedConfidence = 
                ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.speed.speed_confidence.speed_confidence;
        cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.driveDirection = DriveDirection(ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.drive_direction.drive_direction);
        
        //set vehicles mesurements
        cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue =
                ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.vehicle_length.vehicle_length_value.vehicle_length_value;
        cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication(ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.vehicle_length.vehicle_length_confidence_indication.vehicle_length_confidence_indication);
        
        cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.vehicleWidth =
                ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.vehicle_width.vehicle_width;

        cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue = 
                ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.longitudinal_acceleration.longitudinal_acceleration_value.longitudinal_acceleration_value;
        cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationConfidence = 
                ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.longitudinal_acceleration.longitudinal_acceleration_confidence.acceleration_confidence;

        cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.curvature.curvatureValue = 
                ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.curvature.curvature_value.curvature_value;
        cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.curvature.curvatureConfidence = CurvatureConfidence(ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.curvature.curvature_confidence.curvature_confidence);
        
        cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.curvatureCalculationMode = CurvatureCalculationMode(ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.curvature_calculation_mode.curvature_calculation_mode);
        
        cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.yawRate.yawRateValue = 
                ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.yaw_rate.yaw_rate_value.yaw_rate_value;
        cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.yawRate.yawRateConfidence = YawRateConfidence(ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.yaw_rate.yaw_rate_confidence.yaw_rate_confidence);
        
        cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.bit_mask = 0;
        //TODO not yet fully implemented -> only important parts, up until here ...
        //cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.accelerationControl = nullptr; // NOT SET
        // cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.lanePosition = nullptr; // NOT SET
        // cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.steeringWheelAngle = nullptr; // NOT SET
        // cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.lateralAcceleration = nullptr; // NOT SET
        // cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.verticalAcceleration = nullptr; // NOT SET
        // cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.performanceClass = nullptr; // NOT SET
        // cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.cenDsrcTollingZone = nullptr; // NOT SET
    }
    cam->cam.camParameters.bit_mask = 0;
    // LowFrequencyContainer - needed for special vehicles //NOT SET
    //cam->cam.camParameters.bit_mask |= lowFrequencyContainer_present; 
    //cam->cam.camParameters.lowFrequencyContainer = nullptr; // NOT SET
    // LowFrequencyContainer - needed for specific special vehicles //NOT SET
    //cam->cam.camParameters.bit_mask |= specialVehicleContainer_present;
    //cam->cam.camParameters.specialVehicleContainer = nullptr; // NOT SET
}


v2x_msgs::msg::CAM CAMHandler::GetROSCAM(std::pair<void *, size_t> msg) {
    // variables
    v2x_msgs::msg::CAM ros_cam;
    CAM *asn_cam = nullptr;
    int ret_code;

    int pdu_num=CAM_PDU;

    // decode
    if ((ret_code = ossDecode((ossGlobal*)world_, &pdu_num, (OssBuf*) msg.first, (void**) &asn_cam)) != 0) {
        RCLCPP_ERROR(GetNode()->get_logger(), "Decode error: %d with message %s", ret_code, ossGetErrMsg((OssGlobal*) world_));
    }

    // convert from asn_cam to ros_cam
    //Header
    ros_cam.header.protocol_version = asn_cam->header.protocolVersion;
    ros_cam.header.message_id = asn_cam->header.messageID;
    ros_cam.header.station_id.station_id = asn_cam->header.stationID;

    ros_cam.cam.generation_delta_time.generation_delta_time = asn_cam->cam.generationDeltaTime;

    //BasicContainer
    ros_cam.cam.cam_parameters.basic_container.station_type.station_type = asn_cam->cam.camParameters.basicContainer.stationType;
    ros_cam.cam.cam_parameters.basic_container.reference_position.latitude.latitude = asn_cam->cam.camParameters.basicContainer.referencePosition.latitude;
    ros_cam.cam.cam_parameters.basic_container.reference_position.longitude.longitude = asn_cam->cam.camParameters.basicContainer.referencePosition.longitude;
    ros_cam.cam.cam_parameters.basic_container.reference_position.position_confidence_ellipse.semi_major_confidence.semi_axis_length = asn_cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence;
    ros_cam.cam.cam_parameters.basic_container.reference_position.position_confidence_ellipse.semi_minor_confidence.semi_axis_length = asn_cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence;
    ros_cam.cam.cam_parameters.basic_container.reference_position.position_confidence_ellipse.semi_major_orientation.heading_value = asn_cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation;
    ros_cam.cam.cam_parameters.basic_container.reference_position.altitude.altitude_value.altitude_value = asn_cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue;
    ros_cam.cam.cam_parameters.basic_container.reference_position.altitude.altitude_confidence.altitude_confidence = asn_cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence;

    //HighFrequencyContainer
    ros_cam.cam.cam_parameters.high_frequency_container.high_frequency_container_container_select = asn_cam->cam.camParameters.highFrequencyContainer.choice;
    if (ros_cam.cam.cam_parameters.high_frequency_container.high_frequency_container_container_select ==
        ros_cam.cam.cam_parameters.high_frequency_container.HIGH_FREQUENCY_CONTAINER_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY) {
        //VehicleContainer
        ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.heading.heading_value.heading_value = asn_cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.heading.headingValue;
        ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.heading.heading_confidence.heading_confidence = asn_cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.heading.headingConfidence;
        ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.speed.speed_value.speed_value = asn_cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.speed.speedValue;
        ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.speed.speed_confidence.speed_confidence = asn_cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.speed.speedConfidence;
        ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.drive_direction.drive_direction = asn_cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.driveDirection;
        ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.vehicle_length.vehicle_length_value.vehicle_length_value = asn_cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue;
        ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.vehicle_length.vehicle_length_confidence_indication.vehicle_length_confidence_indication = asn_cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication;
        ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.vehicle_width.vehicle_width = asn_cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.vehicleWidth;
        ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.longitudinal_acceleration.longitudinal_acceleration_value.longitudinal_acceleration_value = asn_cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue;
        ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.longitudinal_acceleration.longitudinal_acceleration_confidence.acceleration_confidence = asn_cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationConfidence;
        ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.curvature.curvature_value.curvature_value = asn_cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.curvature.curvatureValue;
        ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.curvature.curvature_confidence.curvature_confidence = asn_cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.curvature.curvatureConfidence;
        ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.curvature_calculation_mode.curvature_calculation_mode = asn_cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.curvatureCalculationMode;
        ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.yaw_rate.yaw_rate_value.yaw_rate_value = asn_cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.yawRate.yawRateValue;
        ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.yaw_rate.yaw_rate_confidence.yaw_rate_confidence = asn_cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.yawRate.yawRateConfidence;

        //TODO not yet fully implemented -> only important parts, up until here ...
        if (false) {
//            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.acceleration_control_present = true;
//            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.acceleration_control.brake_pedal_engaged;
//            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.acceleration_control.gas_pedal_engaged;
//            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.acceleration_control.emergency_brake_engaged;
//            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.acceleration_control.collision_warning_engaged;
//            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.acceleration_control.acc_engaged;
//            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.acceleration_control.cruise_control_engaged;
//            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.acceleration_control.speed_limiter_engaged;
        }
        if (false) {
            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.lane_position_present = true;
            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.lane_position.lane_position;
        }
        if (false) {
            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.steering_wheel_angle_present = true;
            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.steering_wheel_angle.steering_wheel_angle_value.steering_wheel_angle_value;
            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.steering_wheel_angle.steering_wheel_angle_confidence.steering_wheel_angle_confidence;
        }
        if (false) {
            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.lateral_acceleration_present = true;
            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.lateral_acceleration.lateral_acceleration_value.lateral_acceleration_value;
            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.lateral_acceleration.lateral_acceleration_confidence.acceleration_confidence;
        }
        if (false) {
            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.vertical_acceleration_present = true;
            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.vertical_acceleration.vertical_acceleration_value.vertical_acceleration_value;
            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.vertical_acceleration.vertical_acceleration_confidence.acceleration_confidence;
        }
        if (false) {
            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.performance_class_present = true;
            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.performance_class.performance_class;
        }
        if (false) {
            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.cen_dsrc_tolling_zone_present = true;
            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.cen_dsrc_tolling_zone.protected_zone_latitude.latitude;
            ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.cen_dsrc_tolling_zone.protected_zone_longitude.longitude;
            if (false) {
                ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.cen_dsrc_tolling_zone.cen_dsrc_tolling_zone_id_present = true;
                ros_cam.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.cen_dsrc_tolling_zone.cen_dsrc_tolling_zone_id.cen_dsrc_tolling_zone_id;
            }
        }
    } else if (ros_cam.cam.cam_parameters.high_frequency_container.high_frequency_container_container_select ==
               ros_cam.cam.cam_parameters.high_frequency_container.HIGH_FREQUENCY_CONTAINER_RSU_CONTAINER_HIGH_FREQUENCY) {
        //RSUContainer
        if (false) {
            ros_cam.cam.cam_parameters.high_frequency_container.rsu_container_high_frequency.protected_communication_zones_rsu_present = true;
            ros_cam.cam.cam_parameters.high_frequency_container.rsu_container_high_frequency.protected_communication_zones_rsu.zonesrsu; // TODO copy zones
        }
    }

    //LowFrequencyContainer
    if (false) {
        ros_cam.cam.cam_parameters.low_frequency_container_present = true;
        ros_cam.cam.cam_parameters.low_frequency_container.low_frequency_container_container_select;
        if (ros_cam.cam.cam_parameters.low_frequency_container.low_frequency_container_container_select ==
            ros_cam.cam.cam_parameters.low_frequency_container.LOW_FREQUENCY_CONTAINER_BASIC_VEHICLE_CONTAINER_LOW_FREQUENCY) {
            ros_cam.cam.cam_parameters.low_frequency_container.basic_vehicle_container_low_frequency.vehicle_role.vehicle_role;
//            ros_cam.cam.cam_parameters.low_frequency_container.basic_vehicle_container_low_frequency.exterior_lights.low_beam_headlights_on;
//            ros_cam.cam.cam_parameters.low_frequency_container.basic_vehicle_container_low_frequency.exterior_lights.high_beam_headlights_on;
//            ros_cam.cam.cam_parameters.low_frequency_container.basic_vehicle_container_low_frequency.exterior_lights.left_turn_signal_on;
//            ros_cam.cam.cam_parameters.low_frequency_container.basic_vehicle_container_low_frequency.exterior_lights.right_turn_signal_on;
//            ros_cam.cam.cam_parameters.low_frequency_container.basic_vehicle_container_low_frequency.exterior_lights.daytime_running_lights_on;
//            ros_cam.cam.cam_parameters.low_frequency_container.basic_vehicle_container_low_frequency.exterior_lights.reverse_light_on;
//            ros_cam.cam.cam_parameters.low_frequency_container.basic_vehicle_container_low_frequency.exterior_lights.fog_light_on;
//            ros_cam.cam.cam_parameters.low_frequency_container.basic_vehicle_container_low_frequency.exterior_lights.parking_lights_on;
            ros_cam.cam.cam_parameters.low_frequency_container.basic_vehicle_container_low_frequency.path_history.history; //TODO copy
        }
    }

    //SpecialVehicleContainer
    if (false) {
        ros_cam.cam.cam_parameters.special_vehicle_container_present = true;
        ros_cam.cam.cam_parameters.special_vehicle_container.special_vehicle_container_container_select;
        if (ros_cam.cam.cam_parameters.special_vehicle_container.special_vehicle_container_container_select ==
            ros_cam.cam.cam_parameters.special_vehicle_container.SPECIAL_VEHICLE_CONTAINER_PUBLIC_TRANSPORT_CONTAINER) {
            ros_cam.cam.cam_parameters.special_vehicle_container.public_transport_container.embarkation_status.embarkation_status;
            if (false) {
                ros_cam.cam.cam_parameters.special_vehicle_container.public_transport_container.pt_activation_present = true;
                ros_cam.cam.cam_parameters.special_vehicle_container.public_transport_container.pt_activation.pt_activation_type.pt_activation_type;
                ros_cam.cam.cam_parameters.special_vehicle_container.public_transport_container.pt_activation.pt_activation_data; //TODO cpy
            }
        }
        if (ros_cam.cam.cam_parameters.special_vehicle_container.special_vehicle_container_container_select ==
            ros_cam.cam.cam_parameters.special_vehicle_container.SPECIAL_VEHICLE_CONTAINER_SPECIAL_TRANSPORT_CONTAINER) {
//            ros_cam.cam.cam_parameters.special_vehicle_container.special_transport_container.special_transport_type.heavy_load;
//            ros_cam.cam.cam_parameters.special_vehicle_container.special_transport_container.special_transport_type.excess_width;
//            ros_cam.cam.cam_parameters.special_vehicle_container.special_transport_container.special_transport_type.excess_length;
//            ros_cam.cam.cam_parameters.special_vehicle_container.special_transport_container.special_transport_type.excess_height;
//            ros_cam.cam.cam_parameters.special_vehicle_container.special_transport_container.light_bar_siren_in_use.light_bar_activated;
//            ros_cam.cam.cam_parameters.special_vehicle_container.special_transport_container.light_bar_siren_in_use.siren_activated;
        }
        if (ros_cam.cam.cam_parameters.special_vehicle_container.special_vehicle_container_container_select ==
            ros_cam.cam.cam_parameters.special_vehicle_container.SPECIAL_VEHICLE_CONTAINER_DANGEROUS_GOODS_CONTAINER) {
            ros_cam.cam.cam_parameters.special_vehicle_container.dangerous_goods_container.dangerous_goods_basic.dangerous_goods_basic;
        }
        if (ros_cam.cam.cam_parameters.special_vehicle_container.special_vehicle_container_container_select ==
            ros_cam.cam.cam_parameters.special_vehicle_container.SPECIAL_VEHICLE_CONTAINER_ROAD_WORKS_CONTAINER_BASIC) {
            if (false) {
                ros_cam.cam.cam_parameters.special_vehicle_container.road_works_container_basic.roadworks_sub_cause_code_present = true;
                ros_cam.cam.cam_parameters.special_vehicle_container.road_works_container_basic.roadworks_sub_cause_code.roadworks_sub_cause_code;
            }
//            ros_cam.cam.cam_parameters.special_vehicle_container.road_works_container_basic.light_bar_siren_in_use.light_bar_activated;
//            ros_cam.cam.cam_parameters.special_vehicle_container.road_works_container_basic.light_bar_siren_in_use.siren_activated;
            if (false) {
                ros_cam.cam.cam_parameters.special_vehicle_container.road_works_container_basic.closed_lanes_present = true;
                if (false) {
                    ros_cam.cam.cam_parameters.special_vehicle_container.road_works_container_basic.closed_lanes.innerhard_shoulder_status_present = true;
                    ros_cam.cam.cam_parameters.special_vehicle_container.road_works_container_basic.closed_lanes.innerhard_shoulder_status.hard_shoulder_status;
                }
                if (false) {
                    ros_cam.cam.cam_parameters.special_vehicle_container.road_works_container_basic.closed_lanes.outerhard_shoulder_status_present = true;
                    ros_cam.cam.cam_parameters.special_vehicle_container.road_works_container_basic.closed_lanes.outerhard_shoulder_status.hard_shoulder_status;
                }
                if (false) {
                    ros_cam.cam.cam_parameters.special_vehicle_container.road_works_container_basic.closed_lanes.driving_lane_status_present = true;
                    ros_cam.cam.cam_parameters.special_vehicle_container.road_works_container_basic.closed_lanes.driving_lane_status.driving_lane_status; //TODO cpy
                }
            }
        }
        if (ros_cam.cam.cam_parameters.special_vehicle_container.special_vehicle_container_container_select ==
            ros_cam.cam.cam_parameters.special_vehicle_container.SPECIAL_VEHICLE_CONTAINER_RESCUE_CONTAINER) {
//            ros_cam.cam.cam_parameters.special_vehicle_container.rescue_container.light_bar_siren_in_use.light_bar_activated;
//            ros_cam.cam.cam_parameters.special_vehicle_container.rescue_container.light_bar_siren_in_use.siren_activated;
        }
        if (ros_cam.cam.cam_parameters.special_vehicle_container.special_vehicle_container_container_select ==
            ros_cam.cam.cam_parameters.special_vehicle_container.SPECIAL_VEHICLE_CONTAINER_EMERGENCY_CONTAINER) {
//            ros_cam.cam.cam_parameters.special_vehicle_container.emergency_container.light_bar_siren_in_use.light_bar_activated;
//            ros_cam.cam.cam_parameters.special_vehicle_container.emergency_container.light_bar_siren_in_use.siren_activated;
            if (false) {
                ros_cam.cam.cam_parameters.special_vehicle_container.emergency_container.incident_indication_present = true;
                ros_cam.cam.cam_parameters.special_vehicle_container.emergency_container.incident_indication.cause_code.cause_code_type;
                ros_cam.cam.cam_parameters.special_vehicle_container.emergency_container.incident_indication.sub_cause_code.sub_cause_code_type;
            }
            if (false) {
                ros_cam.cam.cam_parameters.special_vehicle_container.emergency_container.emergency_priority_present = true;
//                ros_cam.cam.cam_parameters.special_vehicle_container.emergency_container.emergency_priority.request_for_right_of_way;
//                ros_cam.cam.cam_parameters.special_vehicle_container.emergency_container.emergency_priority.request_for_free_crossing_at_a_traffic_light;
            }
        }
        if (ros_cam.cam.cam_parameters.special_vehicle_container.special_vehicle_container_container_select ==
            ros_cam.cam.cam_parameters.special_vehicle_container.SPECIAL_VEHICLE_CONTAINER_SAFETY_CAR_CONTAINER) {
//            ros_cam.cam.cam_parameters.special_vehicle_container.safety_car_container.light_bar_siren_in_use.light_bar_activated;
//            ros_cam.cam.cam_parameters.special_vehicle_container.safety_car_container.light_bar_siren_in_use.siren_activated;
            if (false) {
                ros_cam.cam.cam_parameters.special_vehicle_container.safety_car_container.incident_indication_present = true;
                ros_cam.cam.cam_parameters.special_vehicle_container.safety_car_container.incident_indication.cause_code.cause_code_type;
                ros_cam.cam.cam_parameters.special_vehicle_container.safety_car_container.incident_indication.sub_cause_code.sub_cause_code_type;
            }
            if (false) {
                ros_cam.cam.cam_parameters.special_vehicle_container.safety_car_container.traffic_rule_present = true;
                ros_cam.cam.cam_parameters.special_vehicle_container.safety_car_container.traffic_rule.traffic_rule;
            }
            if (false) {
                ros_cam.cam.cam_parameters.special_vehicle_container.safety_car_container.speed_limit_present = true;
                ros_cam.cam.cam_parameters.special_vehicle_container.safety_car_container.speed_limit.speed_limit;
            }
        }
    }

    /* resets the memory*/
    ossFreePDU((ossGlobal*)world_, pdu_num, asn_cam);
    asn_cam = nullptr;
    
    // return converted cam
    return ros_cam;
}

// for debug
void CAMHandler::PrintCAM() {
    for(void* cam_void_ptr : cam_list_){
        CAM* cam = (CAM*) cam_void_ptr;
        std::ostringstream oss;

        oss << "--- --- --- CAM --- --- ---" << "\n";

        oss << "CAM" << "\n";
        oss << "|-header:" << "\n";
        oss << " |-protocolVersion: " << cam->header.protocolVersion << "\n";
        oss << " |-messageID: " << cam->header.messageID << "\n";
        oss << " |-stationID: " << cam->header.stationID << "\n";
        oss << "|-cam:" << "\n";
        oss << " |-generationDeltaTime: " << cam->cam.generationDeltaTime << "\n";
        oss << " |-camParameters: " << "\n";
        oss << "  |-basicContainer: " << "\n";
        oss << "   |-stationType: " << cam->cam.camParameters.basicContainer.stationType << "\n";
        oss << "   |-referencePosition: " << "\n";
        oss << "    |-latitude: " << cam->cam.camParameters.basicContainer.referencePosition.latitude << "\n";
        oss << "    |-longitude: " << cam->cam.camParameters.basicContainer.referencePosition.longitude << "\n";
        oss << "    |-positionConfidenceEllipse: " << "\n";
        oss << "     |-semiMajorConfidence: "
            << cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence
            << "\n";
        oss << "     |-semiMinorConfidence: "
            << cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence
            << "\n";
        oss << "     |-semiMajorOrientation: "
            << cam->cam.camParameters.basicContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation
            << "\n";
        oss << "    |-altitude: " << "\n";
        oss << "     |-altitudeValue: " << cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue
            << "\n";
        oss << "     |-altitudeConfidence: "
            << cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence << "\n";
        oss << "  |-highFrequencyContainer: " << "\n";
        oss << "   |-present: " << cam->cam.camParameters.highFrequencyContainer.choice << "\n";
        if(cam->cam.camParameters.highFrequencyContainer.choice == basicVehicleContainerHighFrequency_chosen) {
            oss << "   |-choice: " << "HighFrequencyContainer_PR_basicVehicleContainerHighFrequency" << "\n";
            oss << "    |-basicVehicleContainerHighFrequency: " << "\n";
            oss << "     |-heading: " << "\n";
            oss << "      |-headingValue: "
                << cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.heading.headingValue
                << "\n";
            oss << "      |-headingConfidence: "
                << cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.heading.headingConfidence
                << "\n";
            oss << "     |-speed: " << "\n";
            oss << "      |-speedValue: "
                << cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.speed.speedValue
                << "\n";
            oss << "      |-speedConfidence: "
                << cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.speed.speedConfidence
                << "\n";
            oss << "     |-driveDirection: "
                << cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.driveDirection
                << "\n";
            oss << "     |-vehicleLength: " << "\n";
            oss << "      |-vehicleLengthValue: "
                << cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue
                << "\n";
            oss << "      |-vehicleLengthConfidenceIndication: "
                << cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication
                << "\n";
            oss << "     |-vehicleWidth: "
                << cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.vehicleWidth
                << "\n";
            oss << "     |-longitudinalAcceleration: " << "\n";
            oss << "      |-longitudinalAccelerationValue: "
                << cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue
                << "\n";
            oss << "      |-longitudinalAccelerationConfidence: "
                << cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationConfidence
                << "\n";
            oss << "     |-curvature: " << "\n";
            oss << "      |-curvatureValue: "
                << cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.curvature.curvatureValue
                << "\n";
            oss << "      |-curvatureConfidence: "
                << cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.curvature.curvatureConfidence
                << "\n";
            oss << "     |-curvatureCalculationMode: "
                << cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.curvatureCalculationMode
                << "\n";
            oss << "     |-yawRate: " << "\n";
            oss << "      |-yawRateValue: "
                << cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.yawRate.yawRateValue
                << "\n";
            oss << "      |-yawRateConfidence: "
                << cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.yawRate.yawRateConfidence
                << "\n";

            if (cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.bit_mask & accelerationControl_present)
                oss << "     |-accelerationControl(OPTIONAL): present - parsing not yet implemented" << "\n";
            else
                oss << "     |-accelerationControl(OPTIONAL): not present" << "\n";

            if (cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.bit_mask & BasicVehicleContainerHighFrequency_lanePosition_present)
                oss << "     |-lanePosition(OPTIONAL): present - parsing not yet implemented" << "\n";
            else
                oss << "     |-lanePosition(OPTIONAL): not present" << "\n";

            if (cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.bit_mask & steeringWheelAngle_present)
                oss << "     |-steeringWheelAngle(OPTIONAL): present - parsing not yet implemented" << "\n";
            else
                oss << "     |-steeringWheelAngle(OPTIONAL): not present" << "\n";

            if (cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.bit_mask & BasicVehicleContainerHighFrequency_lateralAcceleration_present)
                oss << "     |-lateralAcceleration(OPTIONAL): present - parsing not yet implemented" << "\n";
            else
                oss << "     |-lateralAcceleration(OPTIONAL): not present" << "\n";

            if (cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.bit_mask & BasicVehicleContainerHighFrequency_verticalAcceleration_present)
                oss << "     |-verticalAcceleration(OPTIONAL): present - parsing not yet implemented" << "\n";
            else
                oss << "     |-verticalAcceleration(OPTIONAL): not present" << "\n";

            if (cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.bit_mask & performanceClass_present)
                oss << "     |-performanceClass(OPTIONAL): present - parsing not yet implemented" << "\n";
            else
                oss << "     |-performanceClass(OPTIONAL): not present" << "\n";

            if (cam->cam.camParameters.highFrequencyContainer.u.basicVehicleContainerHighFrequency.bit_mask & cenDsrcTollingZone_present)
                oss << "     |-cenDsrcTollingZone(OPTIONAL): present - parsing not yet implemented" << "\n";
            else
                oss << "     |-cenDsrcTollingZone(OPTIONAL): not present" << "\n";

        } else if(cam->cam.camParameters.highFrequencyContainer.choice == rsuContainerHighFrequency_chosen) {
            oss << "   |-choice: " << "HighFrequencyContainer_PR_rsuContainerHighFrequency" << "\n";
            oss << "    |-rsuContainerHighFrequency: " << "\n";

            if (cam->cam.camParameters.highFrequencyContainer.u.rsuContainerHighFrequency.protectedCommunicationZonesRSU)
                oss << "     |-protectedCommunicationZonesRSU(OPTIONAL): present - parsing not yet implemented" << "\n";
            else
                oss << "     |-protectedCommunicationZonesRSU(OPTIONAL): not present" << "\n";
            break;
        } else {
            oss << "Non standard present value" << "\n";
        }

        if (cam->cam.camParameters.bit_mask & lowFrequencyContainer_present) {
            oss << "  |-lowFrequencyContainer(OPTIONAL): present" << "\n";
            if (cam->cam.camParameters.lowFrequencyContainer.choice == basicVehicleContainerLowFrequency_chosen){
                oss << "   |-choice: " << "LowFrequencyContainer_PR_basicVehicleContainerLowFrequency" << "\n";
                oss << "    |-basicVehicleContainerLowFrequency: " << "\n";
                oss << "     |-vehicleRole: "
                    << cam->cam.camParameters.lowFrequencyContainer.u.basicVehicleContainerLowFrequency.vehicleRole
                    << "\n";
                oss << "     |-exteriorLights: BITSTRING - not implemented for displaying in demo" << "\n";
                oss << "     |-pathHistory: A_SEQUENCE_OF - not implemented for displaying in demo" << "\n";
                break;
            } else {
                oss << "Non standard present value" << "\n";
            }
        } else
            oss << "  |-lowFrequencyContainer(OPTIONAL): not present" << "\n";

        if (cam->cam.camParameters.bit_mask & specialVehicleContainer_present) {
            oss << "  |-specialVehicleContainer(OPTIONAL): present" << "\n";
            oss << "   |-present: " << cam->cam.camParameters.specialVehicleContainer.choice << "\n";
            if (cam->cam.camParameters.specialVehicleContainer.choice == publicTransportContainer_chosen){
                oss << "   |-choice: " << "publicTransportContainer_chosen" << "\n";
                oss << "    |-publicTransportContainer: content parsing not further implemented in demo" << "\n";
            } else if (cam->cam.camParameters.specialVehicleContainer.choice == specialTransportContainer_chosen){
                oss << "   |-choice: " << "specialTransportContainer_chosen" << "\n";
                oss << "    |-specialTransportContainer: content parsing not further implemented in demo" << "\n";
            } else if (cam->cam.camParameters.specialVehicleContainer.choice == dangerousGoodsContainer_chosen){
                oss << "   |-choice: " << "dangerousGoodsContainer_chosen" << "\n";
                oss << "    |-dangerousGoodsContainer: content parsing not further implemented in demo" << "\n";
            } else if (cam->cam.camParameters.specialVehicleContainer.choice == roadWorksContainerBasic_chosen){
                oss << "   |-choice: " << "roadWorksContainerBasic_chosen" << "\n";
                oss << "    |-roadWorksContainerBasic: content parsing not further implemented in demo" << "\n";
            } else if (cam->cam.camParameters.specialVehicleContainer.choice == rescueContainer_chosen){
                oss << "   |-choice: " << "rescueContainer_chosen" << "\n";
                oss << "    |-rescueContainer: content parsing not further implemented in demo" << "\n";
            } else if (cam->cam.camParameters.specialVehicleContainer.choice == emergencyContainer_chosen){
                oss << "   |-choice: " << "emergencyContainer_chosen" << "\n";
                oss << "    |-emergencyContainer: content parsing not further implemented in demo" << "\n";
            } else if (cam->cam.camParameters.specialVehicleContainer.choice == safetyCarContainer_chosen){
                oss << "   |-choice: " << "safetyCarContainer_chosen" << "\n";
                oss << "    |-safetyCarContainer: content parsing not further implemented in demo" << "\n";
            } else {
                oss << "   |-choice: " << "nothing chosen" << "\n";
            }
        } else
            oss << "  |-specialVehicleContainer(OPTIONAL): not present" << "\n";

        RCLCPP_INFO(GetNode()->get_logger(), oss.str().c_str());
    }
    
}
