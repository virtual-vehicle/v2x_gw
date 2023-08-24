#include "CPMHandler.h"
#include "handler/ossits/specifics/utils.h"
extern "C" {
    #include "ossits/include/ossits/ossits.h"
}


struct PerceivedObject;

CPMHandler::CPMHandler(rclcpp::Node *gateway_node)
        : V2XMHandler(MsgType::kCPM, gateway_node) {

    cpm_ =  malloc(sizeof(CPM));  
    new_data_received_ = false;

    // configure
    ReadConfig();

    //init ossits world
    world_ = malloc(sizeof(OssGlobal));
    int retcode;
    if (retcode = ossinit((OssGlobal*) world_, ITS_Container)) {
	    RCLCPP_ERROR(GetNode()->get_logger(), "ossinit error: %d", retcode);
    }

    ossSetEncodingFlags((OssGlobal*)world_, DEBUGPDU);
    ossSetDecodingFlags((OssGlobal*)world_, DEBUGPDU);

    if(CPM_WRITE_TRACE_FILE){
        std::string trace_path_str="trace_cpm.out";
        char* trace_path = new char[trace_path_str.length() + 1];
        strcpy(trace_path, trace_path_str.c_str());
        ossOpenTraceFile((ossGlobal*)world_, trace_path);
    }

    // init CPM
    InitCPM();

    // publisher
    cpm_pub_ = GetNode()->create_publisher<v2x_msgs::msg::CPMList>("cpm/received", 1);;

    // subscriber
    cpm_sub_ = GetNode()->create_subscription<v2x_msgs::msg::CPMList>
            ("cpm/transmitted", 10, std::bind(&CPMHandler::rosCPMCallback, this, std::placeholders::_1));
}

CPMHandler::~CPMHandler() {
    // TODO: CHECK how deep this mighty free functions works, does it free all encapsulated pointers?
    if(cpm_!= nullptr){
        //frees and removes last element of cpm_list
        int ret_code;
        if ((ret_code = ossFreePDU((ossGlobal*)world_, CPM_PDU, cpm_)) != 0) {
            RCLCPP_ERROR(GetNode()->get_logger(), "Free decoded error: %d", ret_code);
        }
        cpm_ = nullptr;
    }
    if(CPM_WRITE_TRACE_FILE){
        ossCloseTraceFile((ossGlobal*)world_);
    }
    ossterm((ossGlobal*)world_);
    free(world_);
}

std::vector<diagnostic_msgs::msg::KeyValue> CPMHandler::GetDiagnostics() {
    std::vector<diagnostic_msgs::msg::KeyValue> values;
    diagnostic_msgs::msg::KeyValue key_value;

    // status - general
    key_value.key = "v2x_handler.cpm.is_active_";
    key_value.value = std::to_string(is_active_);
    values.push_back(key_value);
    key_value.key = "v2x_handler.cpm.is_configured_";
    key_value.value = std::to_string(is_configured_);
    values.push_back(key_value);

    // status - messages
    key_value.key = "v2x_handler.cpm.message_received_counter_";
    key_value.value = std::to_string(message_received_counter_);
    values.push_back(key_value);
    key_value.key = "v2x_handler.cpm.message_sent_counter_";
    key_value.value = std::to_string(message_sent_counter_);
    values.push_back(key_value);

    return values;
}

std::queue<std::pair<void *, size_t>> CPMHandler::GetMessages() {

    // processing variables
    rclcpp::Time current_timestamp = GetNode()->get_clock()->now();
    std::queue<std::pair<void *, size_t>> cpm_queue; // there may be 0..1 CPMs in the queue

    if(!new_data_received_){
        return cpm_queue;
    }

    OssBuf final_cpm_buffer;
    int ret_code;

    // static variables
    static rclcpp::Time prev_process_timestamp = current_timestamp;

    // max frequency check -> CPM specs <= 10Hz
    if(GetFrequency(current_timestamp, prev_process_timestamp) <= 10.0) {

        auto& clk = *GetNode()->get_clock();
        // CPM creation
        final_cpm_buffer.value = NULL;  
        final_cpm_buffer.length = 0;
        if ((ret_code = ossEncode((ossGlobal*)world_, CPM_PDU, (CPM*)cpm_, &final_cpm_buffer)) != 0) {
            RCLCPP_ERROR(GetNode()->get_logger(), "CPM creation error: %d", ret_code);
        }else{
            cpm_queue.push(std::make_pair(final_cpm_buffer.value, final_cpm_buffer.length));
            auto& clk = *GetNode()->get_clock();
            RCLCPP_INFO_THROTTLE(GetNode()->get_logger(), clk, CPM_DEBUG_MSG_THROTTLE_MS,
                                        "CPM created successfully with size %ld", final_cpm_buffer.length);
        }

        prev_process_timestamp = current_timestamp;
    }
    new_data_received_ = false;

    // diagnostics
    message_sent_counter_ += cpm_queue.size();

    return cpm_queue;
}

void CPMHandler::PutMessages(std::queue<std::pair<void *, size_t>> msgs) {
    // take each element from the incoming_queue, convert it to ROS2 format and publish

    auto& clk = *GetNode()->get_clock();

    // variables
    v2x_msgs::msg::CPMList cpm_list;
    cpm_list.header.stamp = GetNode()->get_clock()->now();

    // take each element from the incoming_queue, convert it to ROS2 format and publish
    while (!msgs.empty()) {
        cpm_list.cpms.push_back(GetROSCPM(msgs.front()));
        msgs.pop();
    }

    // diagnostics
    message_received_counter_ += cpm_list.cpms.size();

    cpm_pub_->publish(cpm_list);
}

void CPMHandler::ReadConfig() {
    // read configuration and set is_configured_

    // necessary handler processing constants
    GetNode()->declare_parameter("cpm.active", false);
    GetNode()->get_parameter("cpm.active", CPM_ACTIVE);
    GetNode()->declare_parameter("cpm.handler_debug_msg_throttle_ms", 5000);
    GetNode()->declare_parameter("cpm.write_trace_file", false);
    GetNode()->get_parameter("cpm.write_trace_file", CPM_WRITE_TRACE_FILE);

    if(CPM_ACTIVE) {
        is_active_ = true;
    }

    is_configured_ = true;
}

void CPMHandler::rosCPMCallback(const v2x_msgs::msg::CPMList::SharedPtr ros_cpm_list){
    if(ros_cpm_list->cpms.size() > 1){
        RCLCPP_WARN(GetNode()->get_logger(), "CPM creation only implemented for one CPM in CPMList");
    }

    //TODO: make CPMList when there are more elements in ros_cpm_list

    if(ros_cpm_list->cpms.size() != 0){
        //free optional Containers before pointer is lost
        if(((CPM*)cpm_)->cpm.cpmParameters.bit_mask & perceivedObjectContainer_present){
            int ret_code;
            PerceivedObjectContainer_* poc = ((CPM*)cpm_)->cpm.cpmParameters.perceivedObjectContainer;
            PerceivedObjectContainer_* poc_next;
            while(poc != nullptr){
                poc_next = poc->next;
                free(poc);
                // if ((ret_code = ossFreePDU((ossGlobal*)world_, PerceivedObject_PDU, (void*)&(poc->value))) != 0) {
                //     RCLCPP_ERROR(GetNode()->get_logger(), "Free decoded error: %d", ret_code);
                // }
                poc = poc_next;
            }
            
        }
        // reset data structure
        memset(cpm_, 0, sizeof(CPM));

        new_data_received_ = true;
        for(auto ros_cpm : ros_cpm_list->cpms){
            // header
            ((CPM*)cpm_)->header.protocolVersion = ros_cpm.header.protocol_version;
            ((CPM*)cpm_)->header.messageID = ros_cpm.header.message_id;
            ((CPM*)cpm_)->header.stationID = ros_cpm.header.station_id.station_id;   

            // Generation delta time
            ((CPM*)cpm_)->cpm.generationDeltaTime = ros_cpm.cpm.generation_delta_time.generation_delta_time;
            
            // Management container
            ((CPM*)cpm_)->cpm.cpmParameters.managementContainer.stationType = 
                ros_cpm.cpm.cpm_parameters.management_container.station_type.station_type;

            //TODO: Implement perceived_object_container_segment_info (not needed yet)
            
            ((CPM*)cpm_)->cpm.cpmParameters.managementContainer.referencePosition.latitude = 
                ros_cpm.cpm.cpm_parameters.management_container.reference_position.latitude.latitude;
            ((CPM*)cpm_)->cpm.cpmParameters.managementContainer.referencePosition.longitude = 
                ros_cpm.cpm.cpm_parameters.management_container.reference_position.longitude.longitude;

            ((CPM*)cpm_)->cpm.cpmParameters.managementContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence = 
                ros_cpm.cpm.cpm_parameters.management_container.reference_position.position_confidence_ellipse.semi_major_confidence.semi_axis_length;
            ((CPM*)cpm_)->cpm.cpmParameters.managementContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence = 
                ros_cpm.cpm.cpm_parameters.management_container.reference_position.position_confidence_ellipse.semi_minor_confidence.semi_axis_length;
            ((CPM*)cpm_)->cpm.cpmParameters.managementContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation = 
                ros_cpm.cpm.cpm_parameters.management_container.reference_position.position_confidence_ellipse.semi_major_orientation.heading_value;

            ((CPM*)cpm_)->cpm.cpmParameters.managementContainer.referencePosition.altitude.altitudeValue = 
                ros_cpm.cpm.cpm_parameters.management_container.reference_position.altitude.altitude_value.altitude_value;
            ((CPM*)cpm_)->cpm.cpmParameters.managementContainer.referencePosition.altitude.altitudeConfidence =
                AltitudeConfidence(ros_cpm.cpm.cpm_parameters.management_container.reference_position.altitude.altitude_confidence.altitude_confidence);

            // Station data container
            if (ros_cpm.cpm.cpm_parameters.station_data_container_present){
                ((CPM*)cpm_)->cpm.cpmParameters.bit_mask |= stationDataContainer_present;

                //originatingVehicleContainer
                if(ros_cpm.cpm.cpm_parameters.station_data_container.station_data_container_container_select == v2x_msgs::msg::StationDataContainer::STATION_DATA_CONTAINER_ORIGINATING_VEHICLE_CONTAINER){
                    ((CPM*)cpm_)->cpm.cpmParameters.stationDataContainer.choice = originatingVehicleContainer_chosen;
                    //Heading
                    ((CPM*)cpm_)->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.heading.headingValue = 
                        ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.heading.heading_value.heading_value;
                    ((CPM*)cpm_)->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.heading.headingConfidence = 
                        ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.heading.heading_confidence.heading_confidence;

                    //Speed
                    ((CPM*)cpm_)->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.speed.speedValue = 
                        ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.speed.speed_value.speed_value;
                    ((CPM*)cpm_)->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.speed.speedConfidence = 
                        ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.speed.speed_confidence.speed_confidence;

                    ((CPM*)cpm_)->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.driveDirection = 
                        DriveDirection(ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.drive_direction.drive_direction);
                    
                    // TODO: inpmelent optional Vehicle orientation Angle, Acceleration, Yaw Rate (Not needed Yet)

                    //Optional: Pitch Angle
                    if(ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.pitch_angle_present){
                        ((CPM*)cpm_)->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.bit_mask |= pitchAngle_present;
                        ((CPM*)cpm_)->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.pitchAngle.value = 
                            ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.pitch_angle.value.cartesian_angle_value;
                        ((CPM*)cpm_)->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.pitchAngle.confidence =
                            ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.pitch_angle.confidence.angle_confidence;
                    }
                    
                    //Optional: Roll Angle
                    if(ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.roll_angle_present){
                        ((CPM*)cpm_)->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.bit_mask |= rollAngle_present;
                        
                        ((CPM*)cpm_)->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.rollAngle.value = 
                            ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.roll_angle.value.cartesian_angle_value;
                        ((CPM*)cpm_)->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.rollAngle.confidence =
                            ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.roll_angle.confidence.angle_confidence;
                    }

                    //TODO: Implement Vehicle Length, width, height and trailer data container (not needed yet)
                } else {
                    //((CPM*)cpm_)->cpm.cpmParameters.stationDataContainer.choice = originatingRSUContainer_chosen;
                    //TODO: Implement originating rsu container (Not needed yet)
                } 
            }  

            //TODO: Implement Optional: sensor_information_container (not needed yet)

            //Optional: PerceivedObjectContainer 
            if(ros_cpm.cpm.cpm_parameters.perceived_object_container_present){
                ((CPM*)cpm_)->cpm.cpmParameters.bit_mask |= perceivedObjectContainer_present;
                
                bool first = true;
                PerceivedObjectContainer_* poc =((CPM*)cpm_)->cpm.cpmParameters.perceivedObjectContainer;
                for(auto ros_object : ros_cpm.cpm.cpm_parameters.perceived_object_container.container){
                    PerceivedObjectContainer_ *next_poc = (PerceivedObjectContainer_ *) malloc(sizeof(PerceivedObjectContainer_));
                    memset((void *) next_poc, 0, sizeof(PerceivedObjectContainer_));
                    PerceivedObject *myObj = &next_poc->value;
                    // ID
                    myObj->objectID = ros_object.object_id.identifier;

                    //OPTIONAL: SensorIDList
                    // TODO::   SensorIDList (not needed yet)

                    //time of measurement
                    myObj->timeOfMeasurement = ros_object.time_of_measurement.time_of_measurement;

                    //OPTIONAL: ObjectAge
                    // TODO:: Implement Age (not needed yet)

                    //ObjectConfidence
                    myObj->objectConfidence = ros_object.object_confidence.object_confidence;

                    // Distance
                    myObj->xDistance.value = ros_object.x_distance.value.distance_value;
                    myObj->xDistance.confidence = ros_object.x_distance.confidence.distance_confidence;
                    myObj->yDistance.value = ros_object.y_distance.value.distance_value;
                    myObj->yDistance.confidence = ros_object.y_distance.confidence.distance_confidence;
                    //OPTIONAL: zDistance
                    if(ros_object.z_distance_present){
                        myObj->bit_mask |= zDistance_present;
                        myObj->zDistance.value = ros_object.z_distance.value.distance_value;
                        myObj->zDistance.confidence = ros_object.z_distance.confidence.distance_confidence;
                    }    

                    // Speed
                    myObj->xSpeed.value = ros_object.x_speed.value.speed_value_extended;
                    myObj->xSpeed.confidence = ros_object.x_speed.confidence.speed_confidence;
                    myObj->ySpeed.value = ros_object.y_speed.value.speed_value_extended;
                    myObj->ySpeed.confidence = ros_object.y_speed.confidence.speed_confidence;
                    //OPTIONAL: zSpeed
                    if(ros_object.z_speed_present){
                        myObj->bit_mask |= zSpeed_present;
                        myObj->zSpeed.value = ros_object.z_speed.value.speed_value_extended;
                        myObj->zSpeed.confidence = ros_object.z_speed.confidence.speed_confidence;
                    }

                    //OPTIONAL: Acceleration
                    if(ros_object.x_acceleration_present){
                        myObj->bit_mask |= xAcceleration_present;
                        myObj->xAcceleration.longitudinalAccelerationValue = 
                            ros_object.x_acceleration.longitudinal_acceleration_value.longitudinal_acceleration_value;
                        myObj->xAcceleration.longitudinalAccelerationConfidence = 
                            ros_object.x_acceleration.longitudinal_acceleration_confidence.acceleration_confidence;
                    }
                    if(ros_object.y_acceleration_present){
                        myObj->bit_mask |= yAcceleration_present;
                        myObj->yAcceleration.lateralAccelerationValue = 
                            ros_object.y_acceleration.lateral_acceleration_value.lateral_acceleration_value;
                        myObj->yAcceleration.lateralAccelerationConfidence = 
                            ros_object.y_acceleration.lateral_acceleration_confidence.acceleration_confidence;
                    }
                    if(ros_object.z_acceleration_present){
                        myObj->bit_mask |= zAcceleration_present;
                        myObj->zAcceleration.verticalAccelerationValue = 
                            ros_object.z_acceleration.vertical_acceleration_value.vertical_acceleration_value;
                        myObj->zAcceleration.verticalAccelerationConfidence = 
                            ros_object.z_acceleration.vertical_acceleration_confidence.acceleration_confidence;
                    }

                    //OPTIONAL: YawAngle
                    if(ros_object.yaw_angle_present){
                        myObj->bit_mask |= yawAngle_present;
                        myObj->yawAngle.value = ros_object.yaw_angle.value.cartesian_angle_value;
                        myObj->yawAngle.confidence = ros_object.yaw_angle.confidence.angle_confidence;
                    }
                    
                    //OPTIONAL: Dimension
                    if(ros_object.planar_object_dimension1_present){
                        myObj->bit_mask |= planarObjectDimension1_present;
                        myObj->planarObjectDimension1.value = ros_object.planar_object_dimension1.value.object_dimension_value;
                        myObj->planarObjectDimension1.confidence = ros_object.planar_object_dimension1.confidence.object_dimension_confidence;
                    }
                    if(ros_object.planar_object_dimension2_present){
                        myObj->bit_mask |= planarObjectDimension2_present;
                        myObj->planarObjectDimension2.value = ros_object.planar_object_dimension2.value.object_dimension_value;
                        myObj->planarObjectDimension2.confidence = ros_object.planar_object_dimension2.confidence.object_dimension_confidence;
                    }
                    if(ros_object.vertical_object_dimension_present){
                        myObj->bit_mask |= verticalObjectDimension_present;
                        myObj->verticalObjectDimension.value = ros_object.vertical_object_dimension.value.object_dimension_value;
                        myObj->verticalObjectDimension.confidence = ros_object.vertical_object_dimension.confidence.object_dimension_confidence;
                    }

                    //objectRefPoint
                    myObj->bit_mask |= objectRefPoint_present;
                    myObj->objectRefPoint = ros_object.object_ref_point.object_ref_point;

                    //OPTIONAL: DynamicsStatus
                    // TODO:: Implement Dynamic Status (not needed yet)

                    //OPTIONAL: classification
                    if(ros_object.classification_present){
                        myObj->bit_mask |= classification_present;
                        ObjectClassDescription_ *myObjCLassDescription = (ObjectClassDescription_*)malloc(sizeof(ObjectClassDescription_));
                        memset((void *) myObjCLassDescription, 0, sizeof(ObjectClassDescription));
                        
                        utils::convert2ASN1ObjectClass(&myObjCLassDescription->value, ros_object);      
                        myObj->classification = myObjCLassDescription;
                    }

                    //OPTIONAL: matchedPosition
                    // TODO:: Implement Match position (not needed yet)

                    // Add object to container list

                    if(first){
                        ((CPM*)cpm_)->cpm.cpmParameters.perceivedObjectContainer = next_poc;
                        poc = next_poc;
                        first = false;
                    }else{
                        while(poc->next != nullptr){
                            poc = poc->next;
                            RCLCPP_WARN(GetNode()->get_logger(), "on wrong place of list, should not happen");
                        }
                        poc->next = next_poc;
                        poc = poc->next;
                    }                                      
                    
                }
                //RCLCPP_WARN(GetNode()->get_logger(), "Bitmask is : %x, Poc is: %p", ((CPM*)cpm_)->cpm.cpmParameters.bit_mask, ((CPM*)cpm_)->cpm.cpmParameters.perceivedObjectContainer);
            }
            //Number of perceived objects
            ((CPM*)cpm_)->cpm.cpmParameters.numberOfPerceivedObjects = ros_cpm.cpm.cpm_parameters.number_of_perceived_objects.number_of_perceived_objects;
        }
    }
}


void CPMHandler::InitCPM(){
    // reset data structure
    memset(cpm_, 0, sizeof(CPM));
    
    ((CPM*)cpm_)->header.protocolVersion = 2;
    ((CPM*)cpm_)->header.messageID = 14;
    ((CPM*)cpm_)->header.stationID = 0;

    // Management container
    ((CPM*)cpm_)->cpm.generationDeltaTime = GetGenerationDeltaTime();
    ((CPM*)cpm_)->cpm.cpmParameters.managementContainer.stationType = StationType_unknown;
    ((CPM*)cpm_)->cpm.cpmParameters.managementContainer.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
    ((CPM*)cpm_)->cpm.cpmParameters.managementContainer.referencePosition.longitude = Longitude_unavailable;
    ((CPM*)cpm_)->cpm.cpmParameters.managementContainer.referencePosition.latitude = Latitude_unavailable;
    ((CPM*)cpm_)->cpm.cpmParameters.managementContainer.bit_mask = 0;
    
    // TODO: uncomment when segment info present
    // ((CPM*)cpm_)->cpm.cpmParameters.managementContainer.bitmask |= perceivedObjectContainerSegmentInfo_present;

    ((CPM*)cpm_)->cpm.cpmParameters.numberOfPerceivedObjects = 0;

    ((CPM*)cpm_)->cpm.cpmParameters.bit_mask = 0;
}

v2x_msgs::msg::CPM CPMHandler::GetROSCPM(std::pair<void *, size_t> msg){
    // variables
    v2x_msgs::msg::CPM ros_cpm;
    CPM* asn_cpm = nullptr;
    int ret_code;

    int pdu_num=CPM_PDU;

    // decode
    if ((ret_code = ossDecode((ossGlobal*)world_, &pdu_num, (OssBuf*) msg.first, (void**) &asn_cpm)) != 0) {
        RCLCPP_ERROR(GetNode()->get_logger(), "Decode error: %d", ret_code);
    }

    auto& clk = *GetNode()->get_clock();
    RCLCPP_WARN_THROTTLE(GetNode()->get_logger(), clk, CPM_DEBUG_MSG_THROTTLE_MS,
                         "Not all parts implemented for translation to ROS CPM! -> search for TODO");

    // convert from asn_cam to ros_cam
    //Header
    ros_cpm.header.protocol_version = asn_cpm->header.protocolVersion;
    ros_cpm.header.message_id = asn_cpm->header.messageID;
    ros_cpm.header.station_id.station_id = asn_cpm->header.stationID;

    //Generation Delta Time
    ros_cpm.cpm.generation_delta_time.generation_delta_time = asn_cpm->cpm.generationDeltaTime;

    // Management container
    ros_cpm.cpm.cpm_parameters.management_container.station_type.station_type = asn_cpm->cpm.cpmParameters.managementContainer.stationType;

    //TODO implement optional perceived_object_container_segment_info (not needed yet)

    ros_cpm.cpm.cpm_parameters.management_container.reference_position.latitude.latitude = asn_cpm->cpm.cpmParameters.managementContainer.referencePosition.latitude;
    ros_cpm.cpm.cpm_parameters.management_container.reference_position.longitude.longitude = asn_cpm->cpm.cpmParameters.managementContainer.referencePosition.longitude;

    ros_cpm.cpm.cpm_parameters.management_container.reference_position.position_confidence_ellipse.semi_major_confidence.semi_axis_length = asn_cpm->cpm.cpmParameters.managementContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence;
    ros_cpm.cpm.cpm_parameters.management_container.reference_position.position_confidence_ellipse.semi_minor_confidence.semi_axis_length = asn_cpm->cpm.cpmParameters.managementContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence;
    ros_cpm.cpm.cpm_parameters.management_container.reference_position.position_confidence_ellipse.semi_major_orientation.heading_value = asn_cpm->cpm.cpmParameters.managementContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation;

    ros_cpm.cpm.cpm_parameters.management_container.reference_position.altitude.altitude_value.altitude_value = asn_cpm->cpm.cpmParameters.managementContainer.referencePosition.altitude.altitudeValue;
    ros_cpm.cpm.cpm_parameters.management_container.reference_position.altitude.altitude_confidence.altitude_confidence = asn_cpm->cpm.cpmParameters.managementContainer.referencePosition.altitude.altitudeConfidence;
    
    //Station Data Container
    if (asn_cpm->cpm.cpmParameters.bit_mask & stationDataContainer_present){
        ros_cpm.cpm.cpm_parameters.station_data_container_present = true;
        // originatingVehicleContainer
        ros_cpm.cpm.cpm_parameters.station_data_container.station_data_container_container_select = asn_cpm->cpm.cpmParameters.stationDataContainer.choice;
        
        if (ros_cpm.cpm.cpm_parameters.station_data_container.station_data_container_container_select ==
            ros_cpm.cpm.cpm_parameters.station_data_container.STATION_DATA_CONTAINER_ORIGINATING_VEHICLE_CONTAINER) {
            
            //Heading
            ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.heading.heading_value.heading_value =
                asn_cpm->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.heading.headingValue;
            ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.heading.heading_confidence.heading_confidence =
                asn_cpm->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.heading.headingConfidence;

            //Speed
            ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.speed.speed_value.speed_value =
                asn_cpm->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.speed.speedValue;
            ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.speed.speed_confidence.speed_confidence =
                asn_cpm->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.speed.speedConfidence;      
                    
            //Drive direction
            ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.drive_direction.drive_direction =
                asn_cpm->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.driveDirection;
            
            // TODO: inpmelent optional Vehicle orientation Angle, Acceleration, Yaw Rate (Not needed Yet)
            
            //Optional: Pitch Angle
            if(asn_cpm->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.bit_mask & pitchAngle_present){
                ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.pitch_angle_present = true;
                ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.pitch_angle.value.cartesian_angle_value =
                    asn_cpm->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.pitchAngle.value;
                ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.pitch_angle.confidence.angle_confidence =
                    asn_cpm->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.pitchAngle.confidence;           
            }
            
            //Optional: Roll Angle
            if(asn_cpm->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.bit_mask & rollAngle_present){
                ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.roll_angle_present = true;
                ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.roll_angle.value.cartesian_angle_value =
                    asn_cpm->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.rollAngle.value;
                ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.roll_angle.confidence.angle_confidence =
                    asn_cpm->cpm.cpmParameters.stationDataContainer.u.originatingVehicleContainer.rollAngle.confidence;           
            }

            //TODO: Implement Vehicle Length, width, height and trailer data container (not needed yet)

        }else if (ros_cpm.cpm.cpm_parameters.station_data_container.station_data_container_container_select ==
            ros_cpm.cpm.cpm_parameters.station_data_container.STATION_DATA_CONTAINER_ORIGINATING_RSUCONTAINER) {
            //TODO: implement originating rsu container (Not needed yet)
        }
    }

    //TODO: Implement Optional: sensor_information_container (not needed yet)

    // Perceived Object Container
    if( asn_cpm->cpm.cpmParameters.numberOfPerceivedObjects > 0 && (asn_cpm->cpm.cpmParameters.bit_mask & perceivedObjectContainer_present) != 0){
        ros_cpm.cpm.cpm_parameters.perceived_object_container_present = true;
        // TODO: I don't know if we can iterate through sequence like this
        
        PerceivedObjectContainer_* current_object_container = asn_cpm->cpm.cpmParameters.perceivedObjectContainer;
        for(int i = 0; i < asn_cpm->cpm.cpmParameters.numberOfPerceivedObjects; i++){
            PerceivedObject *object;
            v2x_msgs::msg::PerceivedObject ros_object;

            object = &(current_object_container->value);
            //Object Identifier
            ros_object.object_id.identifier = object->objectID;

            //OPTIONAL: SensorIDList
            // TODO::   SensorIDList (not needed yet)

            //Time of measurement
            ros_object.time_of_measurement.time_of_measurement = object->timeOfMeasurement;

            //OPTIONAL: ObjectAge
            // TODO:: Implement Age (not needed yet)

            //Object Confidence
            ros_object.object_confidence.object_confidence = object->objectConfidence;

            // Distance
            ros_object.x_distance.value.distance_value = object->xDistance.value;
            ros_object.x_distance.confidence.distance_confidence = object->xDistance.confidence;
            ros_object.y_distance.value.distance_value = object->yDistance.value;
            ros_object.y_distance.confidence.distance_confidence = object->yDistance.confidence;
            if(object->bit_mask & zDistance_present){
                ros_object.z_distance_present = true;
                ros_object.z_distance.value.distance_value = object->zDistance.value;
                ros_object.z_distance.confidence.distance_confidence = object->zDistance.confidence;
            }

            //Speed
            ros_object.x_speed.value.speed_value_extended = object->xSpeed.value;
            ros_object.x_speed.confidence.speed_confidence = object->xSpeed.confidence;
            ros_object.y_speed.value.speed_value_extended = object->ySpeed.value;
            ros_object.y_speed.confidence.speed_confidence = object->ySpeed.confidence;
            if(object->bit_mask & zSpeed_present){
                ros_object.z_speed_present = true;
                ros_object.z_speed.value.speed_value_extended = object->zSpeed.value;
                ros_object.z_speed.confidence.speed_confidence = object->zSpeed.confidence;
            }

            //OPTIONAL: Acceleration
            if(object->bit_mask & xAcceleration_present){
                ros_object.x_acceleration_present = true;
                ros_object.x_acceleration.longitudinal_acceleration_value.longitudinal_acceleration_value = object->xAcceleration.longitudinalAccelerationValue;
                ros_object.x_acceleration.longitudinal_acceleration_confidence.acceleration_confidence = object->xAcceleration.longitudinalAccelerationConfidence;
            }
            if(object->bit_mask & yAcceleration_present){
                ros_object.y_acceleration_present = true;
                ros_object.y_acceleration.lateral_acceleration_value.lateral_acceleration_value = object->yAcceleration.lateralAccelerationValue;
                ros_object.y_acceleration.lateral_acceleration_confidence.acceleration_confidence = object->yAcceleration.lateralAccelerationConfidence;
            }
            if(object->bit_mask & zAcceleration_present){
                ros_object.z_acceleration_present = true;
                ros_object.z_acceleration.vertical_acceleration_value.vertical_acceleration_value = object->zAcceleration.verticalAccelerationValue;
                ros_object.z_acceleration.vertical_acceleration_confidence.acceleration_confidence = object->zAcceleration.verticalAccelerationConfidence;
            }

            //OPTIONAL: yaw angle
            if(object->bit_mask & yawAngle_present){
                ros_object.yaw_angle_present = true;
                ros_object.yaw_angle.value.cartesian_angle_value = object->yawAngle.value;
                ros_object.yaw_angle.confidence.angle_confidence = object->yawAngle.confidence;
            }

            //OPTIONAL: Dimension
            if(object->bit_mask & planarObjectDimension1_present){
                ros_object.planar_object_dimension1_present = true;
                ros_object.planar_object_dimension1.value.object_dimension_value = object->planarObjectDimension1.value;
                ros_object.planar_object_dimension1.confidence.object_dimension_confidence = object->planarObjectDimension1.confidence;
            }
            if(object->bit_mask & planarObjectDimension2_present){
                ros_object.planar_object_dimension2_present = true;
                ros_object.planar_object_dimension2.value.object_dimension_value = object->planarObjectDimension2.value;
                ros_object.planar_object_dimension2.confidence.object_dimension_confidence = object->planarObjectDimension2.confidence;
            }
            if(object->bit_mask & verticalObjectDimension_present){
                ros_object.vertical_object_dimension_present = true;
                ros_object.vertical_object_dimension.value.object_dimension_value = object->verticalObjectDimension.value;
                ros_object.vertical_object_dimension.confidence.object_dimension_confidence = object->verticalObjectDimension.confidence;
            }
            
            // reference point
            if(object->bit_mask & objectRefPoint_present){
                ros_object.object_ref_point.object_ref_point = object->objectRefPoint;
            }
            //OPTIONAL: DynamicsStatus
            // TODO:: Implement Dynamic Status (not needed yet)

            //OPTIONAL: classification
            if(object->bit_mask & classification_present){
                ros_object.classification_present = true;
                // iterate over sequence items
                ObjectClassDescription_* ocd=object->classification;
                ros_object.classification.description.push_back(utils::getROSMsgsObjectClass(&ocd->value.ObjectClass_class));
                while(ocd->next != nullptr){
                    ocd = ocd->next;
                    ros_object.classification.description.push_back(utils::getROSMsgsObjectClass(&ocd->value.ObjectClass_class));
                }
            }

            // TODO:: Implement Match position (not needed yet)

            ros_cpm.cpm.cpm_parameters.perceived_object_container.container.push_back(ros_object);
            current_object_container = current_object_container->next;
        }
        
        //Number of perceived objects
        ros_cpm.cpm.cpm_parameters.number_of_perceived_objects.number_of_perceived_objects = asn_cpm->cpm.cpmParameters.numberOfPerceivedObjects;
    }
    // TODO: Implement

    // free memory
    if ((ret_code = ossFreePDU((ossGlobal*)world_, CPM_PDU, asn_cpm)) != 0) {
        RCLCPP_ERROR(GetNode()->get_logger(), "Free decoded error: %d", ret_code);
    }
    asn_cpm = nullptr;

    // return converted cam
    return ros_cpm;
}

void CPMHandler::PrintCPM(){
    
}