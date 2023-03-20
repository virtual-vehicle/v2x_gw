//
// Created by Christoph Pilz
//
// Description:
// This file contains helper functions for IVIM conversion
//
// Author(s): "Christoph Pilz, Nives Krizanec"
// Copyright: "Copyright 2023, vehicleCAPTAIN toolbox"
// Credits: ["Christoph Pilz", "Nives Krizanec""]
// License: "BSD-3-clause"
// Version: "1.0.0"
// Maintainer: "Christoph Pilz"
// E-Mail: "christoph.pilz@v2c2.at"
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

#ifndef V2X_GW_IVIMUTILS_H
#define V2X_GW_IVIMUTILS_H

#include <v2x_msgs/msg/ivim_list.hpp>

#include "V2XMHandler.h"

extern "C" {
#include "vcits/ivim/IVIM.h"
#include "vcits/ivim/IviContainers.h"
#include "vcits/ivim/IviContainer.h"
#include "vcits/ivim/GlcParts.h"
#include "vcits/ivim/GlcPart.h"
#include "vcits/ivim/Zone.h"
#include "vcits/ivim/DeltaPositions.h"
#include "vcits/ivim/DeltaPosition.h"
#include "vcits/ivim/TextContainer.h"
#include "vcits/ivim/TcPart.h"
#include "vcits/ivim/ZoneIds.h"
#include "vcits/ivim/Zid.h"
#include "vcits/ivim/LanePositions.h"
#include "vcits/ivim/LanePosition.h"
#include "vcits/ivim/TextLines.h"
#include "vcits/ivim/Text.h"
}

#define CHECK_FAIL(result, message) do { \
                                      if (result) { \
                                        if (node_) RCLCPP_ERROR(node_->get_logger(), message); \
                                        exit(EXIT_FAILURE); \
                                      } \
                                    } while(0);
class IVIMUtils {
public:

  static void setNode(rclcpp::Node * node);

  static void fillASNIviContainer(v2x_msgs::msg::IviContainer ros_ivi_container, IviContainer_t* asn_ivi_container);
  static void fillASNGlcPart(v2x_msgs::msg::GlcPart ros_glc_part, GlcPart_t* asn_glc_part);
  static void fillASNZone(v2x_msgs::msg::Zone ros_zone, Zone_t* asn_zone);
  static void fillASNPolygonalLine(v2x_msgs::msg::PolygonalLine ros_polygonal_line, PolygonalLine_t* asn_polygonal_line);
  static void fillASNDeltaPosition(v2x_msgs::msg::DeltaPosition ros_delta_position, DeltaPosition_t* asn_delta_position);

  static void fillASNTcPart(v2x_msgs::msg::TcPart ros_tc_part, TcPart_t* asn_tc_part);
  static void fillASNZoneId(v2x_msgs::msg::Zid ros_zone_id, Zid_t* asn_zone_id);
  static void fillASNLanePosition(v2x_msgs::msg::LanePosition ros_lane_position, LanePosition_t* asn_lane_position);
  static void fillASNText(v2x_msgs::msg::Text ros_text, Text_t* asn_text);

  static v2x_msgs::msg::IviContainer GetROSIviContainer(IviContainer_t* asn_ivi_container);
  static v2x_msgs::msg::GlcPart GetROSGlcPart(GlcPart_t* asn_glc_part);
  static v2x_msgs::msg::Zone GetROSZone(Zone_t* asn_zone);
  static v2x_msgs::msg::PolygonalLine GetROSPolygonalLine(PolygonalLine_t* asn_polygonal_line);
  static v2x_msgs::msg::DeltaPosition GetROSDeltaPosition(DeltaPosition_t* asn_delta_position);

  static v2x_msgs::msg::TcPart GetROSTcPart(TcPart_t* asn_tc_part);
  static v2x_msgs::msg::Zid GetROSZoneId(Zid_t* asn_zone_id);
  static v2x_msgs::msg::LanePosition GetROSLanePosition(LanePosition_t* asn_lane_position);
  static v2x_msgs::msg::Text GetROSText(Text_t* asn_text);

  static void* AllocateClearedMemory(size_t bytes);

private:
  static rclcpp::Node * node_;
};

#endif //V2X_GW_IVIMUTILS_H
