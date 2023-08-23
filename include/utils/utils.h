//
// Created by Alina Steinberger
//
// Description:
// This uitls file contains a few supporting methods for copying message parts
//
// Author(s): "Alina Steinberger"
// Copyright: "Copyright 2023, vehicleCAPTAIN toolbox"
// Credits: ["Alina Steinberger"]
// License: "BSD-3-clause"
// Version: "1.0.0"
// Maintainer: "Alina Steinberger"
// E-Mail: "alina.steinberger@v2c2.at"
// Status = "Production"
//
// Reference to used code:
// <Description1 what was used> (<Link>)
// <Description2 what was used> (<Link>)
//
// Possible Improvements:
// [] <Bug 1>
// [] <Refactoring Idea 2>
// [] <Feature Idea 3>
//

#include <v2x_msgs/msg/cpm_list.hpp>
#include "vcits/cpm/PerceivedObjectContainer.h"
#include "vcits/cpm/SensorInformationContainer.h"
#include "vcits/cpm/StationDataContainer.h"
#include "vcits/cpm/FreeSpaceAddendumContainer.h"
#include "vcits/cpm/PerceivedObject.h"
#include "vcits/cpm/ObjectClassDescription.h"
#include "vcits/cpm/ObjectClass.h"
#include "vcits/cpm/LongitudinalAcceleration.h"
#include "vcits/cpm/LateralAcceleration.h"
#include "vcits/cpm/VerticalAcceleration.h"
#include "vcits/cpm/ObjectRefPoint.h"
#include "vcits/cpm/ObjectDimension.h"
#include "vcits/cpm/ObjectDistanceWithConfidence.h"
#include "vcits/cpm/CartesianAngle.h"

namespace utils
{
        

static void convert2ASN1ObjectClass(ObjectClass *myObjClass, v2x_msgs::msg::PerceivedObject object)
{
    switch(object.classification.description.at(0).class_container_select){
        case v2x_msgs::msg::ObjectClass::CLASS_NOTHING:
            break;
        case v2x_msgs::msg::ObjectClass::CLASS_PERSON_SUBCLASS:
            myObjClass->Class.present = ObjectClass__class_PR::ObjectClass__class_PR_person;
            myObjClass->confidence = object.classification.description.at(0).confidence.class_confidence;
            myObjClass->Class.choice.person.confidence = object.classification.description.at(0).person.confidence.class_confidence;
            myObjClass->Class.choice.person.type = object.classification.description.at(0).person.type.person_subclass_type;
            break;
        case v2x_msgs::msg::ObjectClass::CLASS_VEHICLE_SUBCLASS:
            myObjClass->Class.present = ObjectClass__class_PR::ObjectClass__class_PR_vehicle;
            myObjClass->confidence = object.classification.description.at(0).confidence.class_confidence;
            myObjClass->Class.choice.vehicle.confidence = object.classification.description.at(0).vehicle.confidence.class_confidence;
            myObjClass->Class.choice.vehicle.type = object.classification.description.at(0).vehicle.type.vehicle_subclass_type;
            break;
        case v2x_msgs::msg::ObjectClass::CLASS_ANIMAL_SUBCLASS:
            myObjClass->Class.present = ObjectClass__class_PR::ObjectClass__class_PR_animal;
            myObjClass->confidence = object.classification.description.at(0).confidence.class_confidence;
            myObjClass->Class.choice.animal.confidence = object.classification.description.at(0).animal.confidence.class_confidence;
            myObjClass->Class.choice.animal.type = object.classification.description.at(0).animal.type.animal_subclass_type;
            break;
        case v2x_msgs::msg::ObjectClass::CLASS_OTHER_SUBCLASS:
            myObjClass->Class.present = ObjectClass__class_PR::ObjectClass__class_PR_other;
            myObjClass->confidence = object.classification.description.at(0).confidence.class_confidence;
            myObjClass->Class.choice.other.confidence = object.classification.description.at(0).other.confidence.class_confidence;
            myObjClass->Class.choice.other.type = object.classification.description.at(0).other.type.other_sublass_type;
        default:
            break;
    }
}

/**
* convert2ROSObjectClass
*
* This function does convert the asn1 object class to a v2x_msgs ROS specific object class.
*
* @param ObjectClass asn1 object class
* @return v2x_msgs::msg::ObjectClass
*/
v2x_msgs::msg::ObjectClass getROSMsgsObjectClass(ObjectClass *asn1Class)
{
    v2x_msgs::msg::ObjectClass ros_class;
    switch(asn1Class->Class.present){
        case ObjectClass__class_PR::ObjectClass__class_PR_NOTHING:
            break;
        case ObjectClass__class_PR::ObjectClass__class_PR_person:
            ros_class.class_container_select = ros_class.CLASS_PERSON_SUBCLASS;
            ros_class.person.confidence.class_confidence = asn1Class->Class.choice.person.confidence;
            ros_class.person.type.person_subclass_type = asn1Class->Class.choice.person.type;
            break;
        case ObjectClass__class_PR::ObjectClass__class_PR_vehicle:
            ros_class.class_container_select = ros_class.CLASS_VEHICLE_SUBCLASS;
            ros_class.vehicle.confidence.class_confidence = asn1Class->Class.choice.vehicle.confidence;
            ros_class.vehicle.type.vehicle_subclass_type = asn1Class->Class.choice.vehicle.type;
            break;
        case ObjectClass__class_PR::ObjectClass__class_PR_animal:
            ros_class.class_container_select = ros_class.CLASS_ANIMAL_SUBCLASS;
            ros_class.animal.confidence.class_confidence = asn1Class->Class.choice.animal.confidence;
            ros_class.animal.type.animal_subclass_type = asn1Class->Class.choice.animal.type;
            break;
        case ObjectClass__class_PR::ObjectClass__class_PR_other:
            ros_class.class_container_select = ros_class.CLASS_OTHER_SUBCLASS;
            ros_class.other.confidence.class_confidence = asn1Class->Class.choice.other.confidence;
            ros_class.other.type.other_sublass_type = asn1Class->Class.choice.other.type;
            break;
        default:
            break;
    }

    return ros_class;
}

} // namespace utils