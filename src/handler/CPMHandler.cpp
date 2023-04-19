#include "CPMHandler.h"
#include "utils.h"
extern "C" {
#include "vcits/exceptions/V2XLibExceptions.h"

#include "vcits/parser/Decoder.h"
#include "vcits/parser/Encoder.h"
}


struct PerceivedObject;

CPMHandler::CPMHandler(rclcpp::Node *gateway_node)
        : V2XMHandler(MsgType::kCPM, gateway_node) {

    cpm_ = (CPM_t *) malloc(
            sizeof(CPM_t));  
    new_data_received_ = false;

    // configure
    ReadConfig();

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
        ASN_STRUCT_FREE(asn_DEF_CPM, cpm_);
        cpm_ = nullptr;
    }
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

    // static variables
    static rclcpp::Time prev_process_timestamp = current_timestamp;

    // max frequency check -> CPM specs <= 10Hz
    if(GetFrequency(current_timestamp, prev_process_timestamp) <= 10.0) {

        auto& clk = *GetNode()->get_clock();
        // CAM creation
        size_t final_cpm_size;
        void *final_cpm_buffer;

        try {
            Encoder::validate_constraints(&asn_DEF_CPM, cpm_);
            final_cpm_size = Encoder::encode(&asn_DEF_CPM, nullptr, cpm_, &final_cpm_buffer);

            if (final_cpm_size > 0) {
                cpm_queue.push(std::make_pair(final_cpm_buffer, final_cpm_size));
                auto& clk = *GetNode()->get_clock();
                RCLCPP_INFO_THROTTLE(GetNode()->get_logger(), clk, CPM_DEBUG_MSG_THROTTLE_MS,
                                     "CPM created successfully with size %ld", final_cpm_size);
            } else {
                RCLCPP_ERROR(GetNode()->get_logger(), "CPM creation failed - you should probably stop the program");
            }
        } catch (ValidateConstraintsException e) {
            RCLCPP_ERROR(GetNode()->get_logger(), e.what());
        } catch (EncodingException e) {
            RCLCPP_ERROR(GetNode()->get_logger(), e.what());
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
    GetNode()->get_parameter("cpm.handler_debug_msg_throttle_ms", CPM_DEBUG_MSG_THROTTLE_MS);

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
        if(cpm_->cpm.cpmParameters.stationDataContainer != nullptr){
            ASN_STRUCT_FREE(asn_DEF_StationDataContainer, cpm_->cpm.cpmParameters.stationDataContainer);
        }
        if(cpm_->cpm.cpmParameters.perceivedObjectContainer != nullptr){
            ASN_STRUCT_FREE(asn_DEF_PerceivedObjectContainer, cpm_->cpm.cpmParameters.perceivedObjectContainer);
        }
        // reset data structure
        memset((void *) cpm_, 0, sizeof(CPM_t));

        new_data_received_ = true;
        for(auto ros_cpm : ros_cpm_list->cpms){
            // header
            cpm_->header.protocolVersion = ros_cpm.header.protocol_version;
            cpm_->header.messageID = ros_cpm.header.message_id;
            cpm_->header.stationID = ros_cpm.header.station_id.station_id;   

            // Generation delta time
            cpm_->cpm.generationDeltaTime = ros_cpm.cpm.generation_delta_time.generation_delta_time;
            
            // Management container
            cpm_->cpm.cpmParameters.managementContainer.stationType = 
                ros_cpm.cpm.cpm_parameters.management_container.station_type.station_type;

            //TODO: Implement perceived_object_container_segment_info (not needed yet)
            
            cpm_->cpm.cpmParameters.managementContainer.referencePosition.latitude = 
                ros_cpm.cpm.cpm_parameters.management_container.reference_position.latitude.latitude;
            cpm_->cpm.cpmParameters.managementContainer.referencePosition.longitude = 
                ros_cpm.cpm.cpm_parameters.management_container.reference_position.longitude.longitude;

            cpm_->cpm.cpmParameters.managementContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence = 
                ros_cpm.cpm.cpm_parameters.management_container.reference_position.position_confidence_ellipse.semi_major_confidence.semi_axis_length;
            cpm_->cpm.cpmParameters.managementContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence = 
                ros_cpm.cpm.cpm_parameters.management_container.reference_position.position_confidence_ellipse.semi_minor_confidence.semi_axis_length;
            cpm_->cpm.cpmParameters.managementContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation = 
                ros_cpm.cpm.cpm_parameters.management_container.reference_position.position_confidence_ellipse.semi_major_orientation.heading_value;

            cpm_->cpm.cpmParameters.managementContainer.referencePosition.altitude.altitudeValue = 
                ros_cpm.cpm.cpm_parameters.management_container.reference_position.altitude.altitude_value.altitude_value;
            cpm_->cpm.cpmParameters.managementContainer.referencePosition.altitude.altitudeConfidence = 
                ros_cpm.cpm.cpm_parameters.management_container.reference_position.altitude.altitude_confidence.altitude_confidence;

            // Station data container
            if (ros_cpm.cpm.cpm_parameters.station_data_container_present){
                //originatingVehicleContainer
                if(ros_cpm.cpm.cpm_parameters.station_data_container.station_data_container_container_select == v2x_msgs::msg::StationDataContainer::STATION_DATA_CONTAINER_ORIGINATING_VEHICLE_CONTAINER){
                    cpm_->cpm.cpmParameters.stationDataContainer = (StationDataContainer *) malloc(sizeof(StationDataContainer));
                    memset((void *) cpm_->cpm.cpmParameters.stationDataContainer, 0, sizeof(StationDataContainer));

                    cpm_->cpm.cpmParameters.stationDataContainer->present = StationDataContainer_PR_originatingVehicleContainer;
                    
                    //Heading
                    cpm_->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.heading.headingValue = 
                        ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.heading.heading_value.heading_value;
                    cpm_->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.heading.headingConfidence = 
                        ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.heading.heading_confidence.heading_confidence;

                    //Speed
                    cpm_->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.speed.speedValue = 
                        ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.speed.speed_value.speed_value;
                    cpm_->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.speed.speedConfidence = 
                        ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.speed.speed_confidence.speed_confidence;

                    //Drive direction
                    cpm_->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.driveDirection = 
                        ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.drive_direction.drive_direction;
                    
                    // TODO: inpmelent optional Vehicle orientation Angle, Acceleration, Yaw Rate (Not needed Yet)

                    //Optional: Pitch Angle
                    if(ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.pitch_angle_present){
                        cpm_->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.pitchAngle = (CartesianAngle *) malloc(sizeof(CartesianAngle));
                        memset((void *) cpm_->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.pitchAngle, 0, sizeof(CartesianAngle));
                        cpm_->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.pitchAngle->value = 
                            ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.pitch_angle.value.cartesian_angle_value;
                        cpm_->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.pitchAngle->confidence =
                            ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.pitch_angle.confidence.angle_confidence;
                    }
                    
                    //Optional: Roll Angle
                    if(ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.roll_angle_present){
                        cpm_->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.rollAngle = (CartesianAngle *) malloc(sizeof(CartesianAngle));
                        memset((void *) cpm_->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.rollAngle, 0, sizeof(CartesianAngle));
                        cpm_->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.rollAngle->value = 
                            ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.roll_angle.value.cartesian_angle_value;
                        cpm_->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.rollAngle->confidence =
                            ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.roll_angle.confidence.angle_confidence;
                    }

                    //TODO: Implement Vehicle Length, width, height and trailer data container (not needed yet)
                } else {
                    //TODO: Implement originating rsu container (Not needed yet)
                } 
            }  

            //TODO: Implement Optional: sensor_information_container (not needed yet)

            //Optional: PerceivedObjectContainer 
            if(ros_cpm.cpm.cpm_parameters.perceived_object_container_present){
                cpm_->cpm.cpmParameters.perceivedObjectContainer = (PerceivedObjectContainer *) malloc(sizeof(PerceivedObjectContainer));
                memset((void *) cpm_->cpm.cpmParameters.perceivedObjectContainer, 0, sizeof(PerceivedObjectContainer));

                for(auto ros_object : ros_cpm.cpm.cpm_parameters.perceived_object_container.container){
                    PerceivedObject *myObj = (PerceivedObject*) malloc(sizeof(PerceivedObject));
                    memset((void *) myObj, 0, sizeof(PerceivedObject));
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
                        myObj->zDistance = (ObjectDistanceWithConfidence*) malloc(sizeof(ObjectDistanceWithConfidence));
                        memset((void *) myObj->zDistance, 0, sizeof(ObjectDistanceWithConfidence));
                        myObj->zDistance->value = ros_object.z_distance.value.distance_value;
                        myObj->zDistance->confidence = ros_object.z_distance.confidence.distance_confidence;
                    }    

                    // Speed
                    myObj->xSpeed.value = ros_object.x_speed.value.speed_value_extended;
                    myObj->xSpeed.confidence = ros_object.x_speed.confidence.speed_confidence;
                    myObj->ySpeed.value = ros_object.y_speed.value.speed_value_extended;
                    myObj->ySpeed.confidence = ros_object.y_speed.confidence.speed_confidence;
                    //OPTIONAL: zSpeed
                    if(ros_object.z_speed_present){
                        myObj->zSpeed = (SpeedExtended*)malloc(sizeof(SpeedExtended));
                        memset((void *) myObj->zSpeed, 0, sizeof(SpeedExtended));
                        myObj->zSpeed->value = ros_object.z_speed.value.speed_value_extended;
                        myObj->zSpeed->confidence = ros_object.z_speed.confidence.speed_confidence;
                    }

                    //OPTIONAL: Acceleration
                    if(ros_object.x_acceleration_present){
                        myObj->xAcceleration = (LongitudinalAcceleration *)malloc(sizeof(LongitudinalAcceleration));
                        memset((void *) myObj->xAcceleration, 0, sizeof(LongitudinalAcceleration));
                        myObj->xAcceleration->longitudinalAccelerationValue = 
                            ros_object.x_acceleration.longitudinal_acceleration_value.longitudinal_acceleration_value;
                        myObj->xAcceleration->longitudinalAccelerationConfidence = 
                            ros_object.x_acceleration.longitudinal_acceleration_confidence.acceleration_confidence;
                    }
                    if(ros_object.y_acceleration_present){
                        myObj->yAcceleration = (LateralAcceleration *)malloc(sizeof(LateralAcceleration));
                        memset((void *) myObj->yAcceleration, 0, sizeof(LateralAcceleration));
                        myObj->yAcceleration->lateralAccelerationValue = 
                            ros_object.y_acceleration.lateral_acceleration_value.lateral_acceleration_value;
                        myObj->yAcceleration->lateralAccelerationConfidence = 
                            ros_object.y_acceleration.lateral_acceleration_confidence.acceleration_confidence;
                    }
                    if(ros_object.z_acceleration_present){
                        myObj->zAcceleration = (VerticalAcceleration *)malloc(sizeof(VerticalAcceleration));
                        memset((void *) myObj->zAcceleration, 0, sizeof(VerticalAcceleration));
                        myObj->zAcceleration->verticalAccelerationValue = 
                            ros_object.z_acceleration.vertical_acceleration_value.vertical_acceleration_value;
                        myObj->zAcceleration->verticalAccelerationConfidence = 
                            ros_object.z_acceleration.vertical_acceleration_confidence.acceleration_confidence;
                    }

                    //OPTIONAL: YawAngle
                    if(ros_object.yaw_angle_present){
                        myObj->yawAngle = (CartesianAngle *) malloc(sizeof(CartesianAngle));
                        memset((void *) myObj->yawAngle, 0, sizeof(CartesianAngle));
                        myObj->yawAngle->value = ros_object.yaw_angle.value.cartesian_angle_value;
                        myObj->yawAngle->confidence = ros_object.yaw_angle.confidence.angle_confidence;
                    }
                    
                    //OPTIONAL: Dimension
                    if(ros_object.planar_object_dimension1_present){
                        myObj->planarObjectDimension1 = (ObjectDimension*)malloc(sizeof(ObjectDimension));
                        memset((void *) myObj->planarObjectDimension1, 0, sizeof(ObjectDimension));
                        myObj->planarObjectDimension1->value = ros_object.planar_object_dimension1.value.object_dimension_value;
                        myObj->planarObjectDimension1->confidence = ros_object.planar_object_dimension1.confidence.object_dimension_confidence;
                    }
                    if(ros_object.planar_object_dimension2_present){
                        myObj->planarObjectDimension2 = (ObjectDimension*)malloc(sizeof(ObjectDimension));
                        memset((void *) myObj->planarObjectDimension2, 0, sizeof(ObjectDimension));
                        myObj->planarObjectDimension2->value = ros_object.planar_object_dimension2.value.object_dimension_value;
                        myObj->planarObjectDimension2->confidence = ros_object.planar_object_dimension2.confidence.object_dimension_confidence;
                    }
                    if(ros_object.vertical_object_dimension_present){
                        myObj->verticalObjectDimension = (ObjectDimension*)malloc(sizeof(ObjectDimension));
                        memset((void *) myObj->verticalObjectDimension, 0, sizeof(ObjectDimension));
                        myObj->verticalObjectDimension->value = ros_object.vertical_object_dimension.value.object_dimension_value;
                        myObj->verticalObjectDimension->confidence = ros_object.vertical_object_dimension.confidence.object_dimension_confidence;
                    }

                    //objectRefPoint
                    myObj->objectRefPoint = ros_object.object_ref_point.object_ref_point;

                    //OPTIONAL: DynamicsStatus
                    // TODO:: Implement Dynamic Status (not needed yet)

                    //OPTIONAL: classification
                    if(ros_object.classification_present){
                        ObjectClassDescription *myObjCLassDescription = (ObjectClassDescription*)malloc(sizeof(ObjectClassDescription));
                        ObjectClass *myObjClass = (ObjectClass*) malloc(sizeof(ObjectClass));
                        memset((void *) myObjCLassDescription, 0, sizeof(ObjectClassDescription));
                        memset((void *) myObjClass, 0, sizeof(ObjectClass));
                        utils::convert2ASN1ObjectClass(myObjClass, ros_object);                   
        
                        const int result = ASN_SET_ADD(&myObjCLassDescription->list, myObjClass);
                        if(result != 0){
                            ASN_STRUCT_FREE(asn_DEF_ObjectClassDescription, myObjCLassDescription);
                            RCLCPP_ERROR(GetNode()->get_logger(), "asn_set_add() for ObjectClassDescription failed");
                            exit(EXIT_FAILURE);
                        }
                        myObj->classification = myObjCLassDescription;
                    }

                    //OPTIONAL: matchedPosition
                    // TODO:: Implement Match position (not needed yet)

                    // Add object to container list
                    const int result = ASN_SET_ADD(&cpm_->cpm.cpmParameters.perceivedObjectContainer->list, myObj);
                    if(result != 0){
                        ASN_STRUCT_FREE(asn_DEF_PerceivedObjectContainer, cpm_->cpm.cpmParameters.perceivedObjectContainer);
                        RCLCPP_ERROR(GetNode()->get_logger(), "asn_set_add() failed");
                        exit(EXIT_FAILURE);
                    }
                }
            }

            //Number of perceived objects
            cpm_->cpm.cpmParameters.numberOfPerceivedObjects = ros_cpm.cpm.cpm_parameters.number_of_perceived_objects.number_of_perceived_objects;

        }
    }
}


void CPMHandler::InitCPM(){
    // reset data structure
    memset((void *) cpm_, 0, sizeof(CPM_t));

    cpm_->header.protocolVersion = 2;
    cpm_->header.messageID = 14;
    cpm_->header.stationID = 0;

    // Management container
    cpm_->cpm.generationDeltaTime = GetGenerationDeltaTime();
    cpm_->cpm.cpmParameters.managementContainer.stationType = StationType::StationType_unknown;
    cpm_->cpm.cpmParameters.managementContainer.referencePosition.altitude.altitudeValue = AltitudeValue::AltitudeValue_unavailable;
    cpm_->cpm.cpmParameters.managementContainer.referencePosition.longitude = Longitude::Longitude_unavailable;
    cpm_->cpm.cpmParameters.managementContainer.referencePosition.latitude = Latitude::Latitude_unavailable;
    cpm_->cpm.cpmParameters.managementContainer.perceivedObjectContainerSegmentInfo = nullptr;

    // Not set 
    cpm_->cpm.cpmParameters.numberOfPerceivedObjects = 0;
    cpm_->cpm.cpmParameters.perceivedObjectContainer = nullptr;
    cpm_->cpm.cpmParameters.sensorInformationContainer = nullptr;
    cpm_->cpm.cpmParameters.stationDataContainer = nullptr;
    cpm_->cpm.cpmParameters.freeSpaceAddendumContainer = nullptr;
}

v2x_msgs::msg::CPM CPMHandler::GetROSCPM(std::pair<void *, size_t> msg){
    // variables
    v2x_msgs::msg::CPM ros_cpm;
    CPM_t *asn_cpm = nullptr;

    // decode
    try {
        asn_cpm = (CPM_t *) Decoder::decode(&asn_DEF_CPM, msg.first, msg.second);
    } catch (DecodingException e) {
        RCLCPP_ERROR(GetNode()->get_logger(), e.what());
        RCLCPP_INFO(GetNode()->get_logger(),
                    "If decoding fails, we throw away everything, as we would have to check how far we were able to decode");
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
    if (asn_cpm->cpm.cpmParameters.stationDataContainer != nullptr){
        ros_cpm.cpm.cpm_parameters.station_data_container_present = true;
        // originatingVehicleContainer
        ros_cpm.cpm.cpm_parameters.station_data_container.station_data_container_container_select = asn_cpm->cpm.cpmParameters.stationDataContainer->present;
        
        if (ros_cpm.cpm.cpm_parameters.station_data_container.station_data_container_container_select ==
            ros_cpm.cpm.cpm_parameters.station_data_container.STATION_DATA_CONTAINER_ORIGINATING_VEHICLE_CONTAINER) {
            
            //Heading
            ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.heading.heading_value.heading_value =
                asn_cpm->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.heading.headingValue;
            ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.heading.heading_confidence.heading_confidence =
                asn_cpm->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.heading.headingConfidence;

            //Speed
            ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.speed.speed_value.speed_value =
                asn_cpm->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.speed.speedValue;
            ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.speed.speed_confidence.speed_confidence =
                asn_cpm->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.speed.speedConfidence;      
                    
            //Drive direction
            ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.drive_direction.drive_direction =
                asn_cpm->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.driveDirection;
            
            // TODO: inpmelent optional Vehicle orientation Angle, Acceleration, Yaw Rate (Not needed Yet)
            
            //Optional: Pitch Angle
            if(asn_cpm->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.pitchAngle != nullptr){
                ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.pitch_angle_present = true;
                ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.pitch_angle.value.cartesian_angle_value =
                    asn_cpm->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.pitchAngle->value;
                ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.pitch_angle.confidence.angle_confidence =
                    asn_cpm->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.pitchAngle->confidence;           
            }
            
            //Optional: Roll Angle
            if(asn_cpm->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.rollAngle != nullptr){
                ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.roll_angle_present = true;
                ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.roll_angle.value.cartesian_angle_value =
                    asn_cpm->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.rollAngle->value;
                ros_cpm.cpm.cpm_parameters.station_data_container.originating_vehicle_container.roll_angle.confidence.angle_confidence =
                    asn_cpm->cpm.cpmParameters.stationDataContainer->choice.originatingVehicleContainer.rollAngle->confidence;           
            }

            //TODO: Implement Vehicle Length, width, height and trailer data container (not needed yet)

        }else if (ros_cpm.cpm.cpm_parameters.station_data_container.station_data_container_container_select ==
            ros_cpm.cpm.cpm_parameters.station_data_container.STATION_DATA_CONTAINER_ORIGINATING_RSUCONTAINER) {
            //TODO: implement originating rsu container (Not needed yet)
        }
    }

    //TODO: Implement Optional: sensor_information_container (not needed yet)

    // Perceived Object Container
    if( asn_cpm->cpm.cpmParameters.numberOfPerceivedObjects > 0 && asn_cpm->cpm.cpmParameters.perceivedObjectContainer != nullptr){
        ros_cpm.cpm.cpm_parameters.perceived_object_container_present = true;
        // TODO: I don't know if we can iterate through sequence like this
        
        for(int i = 0; i < asn_cpm->cpm.cpmParameters.perceivedObjectContainer->list.count; i++){
            PerceivedObject *object;
            v2x_msgs::msg::PerceivedObject ros_object;

            object = asn_cpm->cpm.cpmParameters.perceivedObjectContainer->list.array[i];
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
            if(object->zDistance != nullptr){
                ros_object.z_distance_present = true;
                ros_object.z_distance.value.distance_value = object->zDistance->value;
                ros_object.z_distance.confidence.distance_confidence = object->zDistance->confidence;
            }

            //Speed
            ros_object.x_speed.value.speed_value_extended = object->xSpeed.value;
            ros_object.x_speed.confidence.speed_confidence = object->xSpeed.confidence;
            ros_object.y_speed.value.speed_value_extended = object->ySpeed.value;
            ros_object.y_speed.confidence.speed_confidence = object->ySpeed.confidence;
            if(object->zSpeed != nullptr){
                ros_object.z_speed_present = true;
                ros_object.z_speed.value.speed_value_extended = object->zSpeed->value;
                ros_object.z_speed.confidence.speed_confidence = object->zSpeed->confidence;
            }

            //OPTIONAL: Acceleration
            if(object->xAcceleration != nullptr){
                ros_object.x_acceleration_present = true;
                ros_object.x_acceleration.longitudinal_acceleration_value.longitudinal_acceleration_value = object->xAcceleration->longitudinalAccelerationValue;
                ros_object.x_acceleration.longitudinal_acceleration_confidence.acceleration_confidence = object->xAcceleration->longitudinalAccelerationConfidence;
            }
            if(object->yAcceleration != nullptr){
                ros_object.y_acceleration_present = true;
                ros_object.y_acceleration.lateral_acceleration_value.lateral_acceleration_value = object->yAcceleration->lateralAccelerationValue;
                ros_object.y_acceleration.lateral_acceleration_confidence.acceleration_confidence = object->yAcceleration->lateralAccelerationConfidence;
            }
            if(object->zAcceleration != nullptr){
                ros_object.z_acceleration_present = true;
                ros_object.z_acceleration.vertical_acceleration_value.vertical_acceleration_value = object->zAcceleration->verticalAccelerationValue;
                ros_object.z_acceleration.vertical_acceleration_confidence.acceleration_confidence = object->zAcceleration->verticalAccelerationConfidence;
            }

            //OPTIONAL: yaw angle
            if(object->yawAngle != nullptr){
                ros_object.yaw_angle_present = true;
                ros_object.yaw_angle.value.cartesian_angle_value = object->yawAngle->value;
                ros_object.yaw_angle.confidence.angle_confidence = object->yawAngle->confidence;
            }

            //OPTIONAL: Dimension
            if(object->planarObjectDimension1 != nullptr){
                ros_object.planar_object_dimension1_present = true;
                ros_object.planar_object_dimension1.value.object_dimension_value = object->planarObjectDimension1->value;
                ros_object.planar_object_dimension1.confidence.object_dimension_confidence = object->planarObjectDimension1->confidence;
            }
            if(object->planarObjectDimension2 != nullptr){
                ros_object.planar_object_dimension2_present = true;
                ros_object.planar_object_dimension2.value.object_dimension_value = object->planarObjectDimension2->value;
                ros_object.planar_object_dimension2.confidence.object_dimension_confidence = object->planarObjectDimension2->confidence;
            }
            if(object->verticalObjectDimension != nullptr){
                ros_object.vertical_object_dimension_present = true;
                ros_object.vertical_object_dimension.value.object_dimension_value = object->verticalObjectDimension->value;
                ros_object.vertical_object_dimension.confidence.object_dimension_confidence = object->verticalObjectDimension->confidence;
            }
            
            // reference point
            ros_object.object_ref_point.object_ref_point = object->objectRefPoint;

            //OPTIONAL: DynamicsStatus
            // TODO:: Implement Dynamic Status (not needed yet)

            //OPTIONAL: classification
            if(object->classification != nullptr) {
                ros_object.classification_present = true;
                // iterate over sequence items
                for(int j = 0; j< object->classification->list.count; j++){
                    ObjectClass* obj_class = object->classification->list.array[j];
                    ros_object.classification.description.push_back(utils::getROSMsgsObjectClass(obj_class));
                }
            }

            // TODO:: Implement Match position (not needed yet)

            ros_cpm.cpm.cpm_parameters.perceived_object_container.container.push_back(ros_object);
        }
        
        //Number of perceived objects
        ros_cpm.cpm.cpm_parameters.number_of_perceived_objects.number_of_perceived_objects = asn_cpm->cpm.cpmParameters.numberOfPerceivedObjects;
    }
    // TODO: Implement

    // free memory
    ASN_STRUCT_FREE(asn_DEF_CPM, asn_cpm);
    asn_cpm = nullptr;

    // return converted cam
    return ros_cpm;
}

void CPMHandler::PrintCPM(){
    
}