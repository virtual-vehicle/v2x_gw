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

extern "C" {
#include "ossits/include/ossits/ossits.h"
}
namespace utils
{
    /**
    * convert2ROSObjectClass
    *
    * This function does convert the asn1 object class to a v2x_msgs ROS specific object class.
    *
    * @param ObjectClass asn1 object class
    * @return v2x_msgs::msg::ObjectClass
    */
    v2x_msgs::msg::ObjectClass getROSMsgsObjectClass(_choice5 *asn1Class)
    {
        v2x_msgs::msg::ObjectClass ros_class;
        switch(asn1Class->choice){
            case person_chosen:
                ros_class.class_container_select = ros_class.CLASS_PERSON_SUBCLASS;
                
                if(asn1Class->u.person.bit_mask & PersonSubclass_type_present){
                    ros_class.person.type.person_subclass_type = asn1Class->u.person.type;
                }else{
                    ros_class.person.type.person_subclass_type = v2x_msgs::msg::PersonSubclassType::PERSON_SUBCLASS_TYPE_UNKNOWN;
                }

                if(asn1Class->u.person.bit_mask & PersonSubclass_confidence_present){
                    ros_class.person.confidence.class_confidence = asn1Class->u.person.confidence;
                }else{
                    ros_class.person.confidence.class_confidence = v2x_msgs::msg::ClassConfidence::CLASS_CONFIDENCE_UNAVAILABLE;
                }
                break;
            case class_vehicle_chosen:
                ros_class.class_container_select = ros_class.CLASS_VEHICLE_SUBCLASS;
                
                if(asn1Class->u.vehicle.bit_mask & VehicleSubclass_type_present){
                    ros_class.vehicle.type.vehicle_subclass_type = asn1Class->u.animal.type;
                }else{
                    ros_class.vehicle.type.vehicle_subclass_type = v2x_msgs::msg::VehicleSubclassType::VEHICLE_SUBCLASS_TYPE_UNKNOWN;
                }

                if(asn1Class->u.vehicle.bit_mask & VehicleSubclass_confidence_present){
                    ros_class.vehicle.confidence.class_confidence = asn1Class->u.vehicle.confidence;
                }else{
                    ros_class.vehicle.confidence.class_confidence = v2x_msgs::msg::ClassConfidence::CLASS_CONFIDENCE_UNAVAILABLE;
                }
                break;
            case animal_chosen:
                ros_class.class_container_select = ros_class.CLASS_ANIMAL_SUBCLASS;

                if(asn1Class->u.animal.bit_mask & AnimalSubclass_type_present){
                    ros_class.animal.type.animal_subclass_type = asn1Class->u.animal.type;
                }else{
                    ros_class.animal.type.animal_subclass_type = v2x_msgs::msg::AnimalSubclassType::ANIMAL_SUBCLASS_TYPE_UNKNOWN;
                }

                if(asn1Class->u.animal.bit_mask & AnimalSubclass_confidence_present){
                    ros_class.animal.confidence.class_confidence = asn1Class->u.animal.confidence;
                }else{
                    ros_class.animal.confidence.class_confidence = v2x_msgs::msg::ClassConfidence::CLASS_CONFIDENCE_UNAVAILABLE;
                }
                break;
            case other_chosen:
                ros_class.class_container_select = ros_class.CLASS_OTHER_SUBCLASS;

                if(asn1Class->u.other.bit_mask & OtherSubclass_type_present){
                    ros_class.other.type.other_sublass_type = asn1Class->u.other.type;
                }else{
                    ros_class.other.type.other_sublass_type = v2x_msgs::msg::OtherSublassType::OTHER_SUBLASS_TYPE_UNKNOWN;
                }

                if(asn1Class->u.other.bit_mask & OtherSubclass_confidence_present){
                    ros_class.other.confidence.class_confidence = asn1Class->u.other.confidence;
                }else{
                    ros_class.other.confidence.class_confidence = v2x_msgs::msg::ClassConfidence::CLASS_CONFIDENCE_UNAVAILABLE;
                }
                break;
            default:
                break;
        }
        return ros_class;
    }

    static void convert2ASN1ObjectClass(ObjectClass *myObjClass, v2x_msgs::msg::PerceivedObject object)
{
    switch(object.classification.description.at(0).class_container_select){
        case v2x_msgs::msg::ObjectClass::CLASS_NOTHING:
            break;
        case v2x_msgs::msg::ObjectClass::CLASS_PERSON_SUBCLASS:
            myObjClass->ObjectClass_class.choice = person_chosen;
            myObjClass->ObjectClass_class.u.person.bit_mask &= PersonSubclass_confidence_present&PersonSubclass_type_present;
            myObjClass->ObjectClass_class.u.person.confidence = object.classification.description.at(0).confidence.class_confidence;
            myObjClass->confidence = object.classification.description.at(0).person.confidence.class_confidence;
            myObjClass->ObjectClass_class.u.person.type = object.classification.description.at(0).person.type.person_subclass_type;
            break;
        case v2x_msgs::msg::ObjectClass::CLASS_VEHICLE_SUBCLASS:
            myObjClass->ObjectClass_class.choice = class_vehicle_chosen;
            myObjClass->ObjectClass_class.u.vehicle.bit_mask &= VehicleSubclass_confidence_present&VehicleSubclass_type_present;
            myObjClass->ObjectClass_class.u.vehicle.confidence = object.classification.description.at(0).confidence.class_confidence;
            myObjClass->confidence = object.classification.description.at(0).vehicle.confidence.class_confidence;
            myObjClass->ObjectClass_class.u.vehicle.type = object.classification.description.at(0).vehicle.type.vehicle_subclass_type;
            break;
        case v2x_msgs::msg::ObjectClass::CLASS_ANIMAL_SUBCLASS:
            myObjClass->ObjectClass_class.choice = animal_chosen;
            myObjClass->ObjectClass_class.u.animal.bit_mask &= PersonSubclass_confidence_present&PersonSubclass_type_present;
            myObjClass->ObjectClass_class.u.animal.confidence = object.classification.description.at(0).confidence.class_confidence;
            myObjClass->confidence = object.classification.description.at(0).animal.confidence.class_confidence;
            myObjClass->ObjectClass_class.u.animal.type = object.classification.description.at(0).animal.type.animal_subclass_type;
            break;
        case v2x_msgs::msg::ObjectClass::CLASS_OTHER_SUBCLASS:
            myObjClass->ObjectClass_class.choice = other_chosen;
            myObjClass->ObjectClass_class.u.other.bit_mask &= PersonSubclass_confidence_present&PersonSubclass_type_present;
            myObjClass->ObjectClass_class.u.other.confidence = object.classification.description.at(0).confidence.class_confidence;
            myObjClass->confidence = object.classification.description.at(0).other.confidence.class_confidence;
            myObjClass->ObjectClass_class.u.other.type = object.classification.description.at(0).other.type.other_sublass_type;
            break;
        default:
            break;
    }
}
}