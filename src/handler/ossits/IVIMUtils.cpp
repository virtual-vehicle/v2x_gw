// -*- lsst-c++ -*-

/*
* <Name of Software>
* See COPYRIGHT file at the top of the source tree.
*
* (Optional)
* This product includes software developed by the
* <Project Name> (<http://projectlink.org/>).
*
* (Licensing)
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the LSST License Statement and
* the GNU General Public License along with this program. If not,
* see <http://www.lsstcorp.org/LegalNotices/>.
*/

#include "handler/ossits/specifics/IVIMUtils.h"
#include "handler/IVIMHandler.h"

rclcpp::Node * IVIMUtils::node_ = nullptr;

void IVIMUtils::setNode(rclcpp::Node * node)
{
  node_ = node;
}

void IVIMUtils::fillASNIviContainer(v2x_msgs::msg::IviContainer ros_ivi_container, IviContainer* asn_ivi_container) {
  assert(asn_ivi_container);

  switch (ros_ivi_container.ivi_container_container_select) {
    case ros_ivi_container.IVI_CONTAINER_GLC: {
      asn_ivi_container->choice = glc_chosen;

      auto &asn_glc = asn_ivi_container->u.glc;
      auto &ros_glc = ros_ivi_container.glc;

      asn_glc.referencePosition.latitude = ros_glc.reference_position.latitude.latitude;
      asn_glc.referencePosition.longitude = ros_glc.reference_position.longitude.longitude;

      asn_glc.referencePosition.positionConfidenceEllipse.semiMajorConfidence = ros_glc.reference_position.position_confidence_ellipse.semi_major_confidence.semi_axis_length;
      asn_glc.referencePosition.positionConfidenceEllipse.semiMinorConfidence = ros_glc.reference_position.position_confidence_ellipse.semi_minor_confidence.semi_axis_length;
      asn_glc.referencePosition.positionConfidenceEllipse.semiMajorOrientation = ros_glc.reference_position.position_confidence_ellipse.semi_major_orientation.heading_value;

      asn_glc.referencePosition.altitude.altitudeValue = ros_glc.reference_position.altitude.altitude_value.altitude_value;
      asn_glc.referencePosition.altitude.altitudeConfidence = AltitudeConfidence(ros_glc.reference_position.altitude.altitude_confidence.altitude_confidence);

      if (ros_glc.reference_position_time_present) {
          asn_glc.referencePositionTime = ros_glc.reference_position_time.timestamp_its;
      }

      if (ros_glc.reference_position_heading_present) {
        // not implemented
      }

      if (ros_glc.reference_position_speed_present) {
        // not implemented
      }

      
      auto &ros_glc_parts = ros_glc.parts.parts;
      asn_glc.parts = (GlcParts_ *) IVIMUtils::AllocateClearedMemory(sizeof(GlcParts_));
      GlcParts_* asn_glc_parts = asn_glc.parts;
      for (int i = 0; i < ros_glc_parts.size(); ++i) {
        // allocate and switch to next in linked list
        if (i != 0){
          asn_glc_parts->next = (GlcParts_ *) IVIMUtils::AllocateClearedMemory(sizeof(GlcParts_));
          asn_glc_parts = asn_glc_parts->next;
        }
        auto &ros_glc_part = ros_glc_parts.at(i);

        IVIMUtils::fillASNGlcPart(ros_glc_part, &(asn_glc_parts->value));
      }
    }
      break;

    case ros_ivi_container.IVI_CONTAINER_TC: {
      asn_ivi_container->choice = tc_chosen;

      asn_ivi_container->u.tc = (TextContainer_ *) IVIMUtils::AllocateClearedMemory(sizeof(TextContainer_));
      TextContainer_* asn_tc = asn_ivi_container->u.tc;
      auto &ros_tc = ros_ivi_container.tc;

      auto &ros_tc_parts = ros_tc.container;

      for (int i = 0; i < ros_tc_parts.size(); ++i) {
        // allocate and switch to next in linked list
        if(i != 0){
          asn_tc->next = (TextContainer_ *) IVIMUtils::AllocateClearedMemory(sizeof(TextContainer_));
          asn_tc = asn_tc->next;
        }

        auto& ros_tc_part = ros_tc_parts.at(i);

        fillASNTcPart(ros_tc_part, &(asn_tc->value));
      }

    }
      break;

    default: if (node_) {
      RCLCPP_INFO(node_->get_logger(),
                  "fillASNIviContainer: This ivi container is not implemented: %ld",
                  ros_ivi_container.ivi_container_container_select);
    }
      break;
  }
}

void IVIMUtils::fillASNGlcPart(v2x_msgs::msg::GlcPart ros_glc_part, GlcPart* asn_glc_part) {
  assert(asn_glc_part);

  asn_glc_part->zoneId = ros_glc_part.zone_id.zid;

  if (ros_glc_part.lane_number_present) {
    asn_glc_part->bit_mask |= laneNumber_present;
    asn_glc_part->laneNumber = ros_glc_part.lane_number.lane_position;
  }

  if (ros_glc_part.zone_extension_present) {
    asn_glc_part->bit_mask |= zoneExtension_present;
    asn_glc_part->zoneExtension = ros_glc_part.zone_extension;
  }

  if (ros_glc_part.zone_heading_present) {
    asn_glc_part->bit_mask |= zoneHeading_present;
    asn_glc_part->zoneHeading = ros_glc_part.zone_heading.heading_value;
  }

  if (ros_glc_part.zone_present) {
    asn_glc_part->bit_mask |= zone_present;
    IVIMUtils::fillASNZone(ros_glc_part.zone, &(asn_glc_part->zone));
  }
}

void IVIMUtils::fillASNZone(v2x_msgs::msg::Zone ros_zone, Zone* asn_zone) {
  assert(asn_zone);

  switch (ros_zone.zone_container_select) {

    case ros_zone.ZONE_NOTHING: {
      asn_zone->choice = 0;
    }
      break;

    case ros_zone.ZONE_SEGMENT: {
      asn_zone->choice = segment_chosen;

      IVIMUtils::fillASNPolygonalLine(ros_zone.segment.line, &(asn_zone->u.segment.line));

      if (ros_zone.segment.lane_width_present) {
        asn_zone->u.segment.bit_mask |= Segment_laneWidth_present;
        asn_zone->u.segment.laneWidth = ros_zone.segment.lane_width.ivi_lane_width;
      }
    }
      break;

    case ros_zone.ZONE_AREA: {
      //NOT IMPLEMENTED
      //asn_zone->choice = area_chosen;
      RCLCPP_INFO(node_->get_logger(),
                  "fillASNZone: ZONE_AREA is not implemented: %ld",
                  ros_zone.zone_container_select);
    }
      break;

    case ros_zone.ZONE_COMPUTED_SEGMENT: {
      //NOT IMPLEMENTED
      //asn_zone->choice = computedSegment_chosen;
      RCLCPP_INFO(node_->get_logger(),
                  "fillASNZone: ZONE_COMPUTED_SEGMENT is not implemented: %ld",
                  ros_zone.zone_container_select);
    }
      break;
    default:  if (node_) {
      RCLCPP_INFO(node_->get_logger(),"fillASNZone: This zone does not exist");
    }
      break;
  }
}

void IVIMUtils::fillASNPolygonalLine(v2x_msgs::msg::PolygonalLine ros_polygonal_line, PolygonalLine* asn_polygonal_line) {
  assert(asn_polygonal_line);

  switch(ros_polygonal_line.polygonal_line_container_select) {

    case ros_polygonal_line.POLYGONAL_LINE_NOTHING: {
      asn_polygonal_line->choice = 0;
    }
      break;

    case ros_polygonal_line.POLYGONAL_LINE_DELTA_POSITIONS: {
      asn_polygonal_line->choice = deltaPositions_chosen;

      asn_polygonal_line->u.deltaPositions = (DeltaPositions_ *) IVIMUtils::AllocateClearedMemory(sizeof(DeltaPositions_));
      DeltaPositions_* asn_polygonal_line_delta_positions = asn_polygonal_line->u.deltaPositions;
      auto &ros_polygonal_line_delta_positions = ros_polygonal_line.delta_positions;
      auto &ros_delta_positions = ros_polygonal_line_delta_positions.positions;

      for (int i = 0; i < ros_delta_positions.size(); ++i) {
        // allocate and switch to next in linked list
        if(i != 0){
          asn_polygonal_line_delta_positions->next = (DeltaPositions_ *) IVIMUtils::AllocateClearedMemory(sizeof(DeltaPositions_));
          asn_polygonal_line_delta_positions = asn_polygonal_line_delta_positions->next;
        }
        auto &ros_delta_position = ros_delta_positions.at(i);

        IVIMUtils::fillASNDeltaPosition(ros_delta_position, &(asn_polygonal_line_delta_positions->value));
      }
    }
      break;

    default: if (node_) {
      RCLCPP_INFO(node_->get_logger(),
                  "fillASNPolygonalLine: This polygonal line is not implemented: %ld",
                  ros_polygonal_line.polygonal_line_container_select);
    }
      break;
  }
}

void IVIMUtils::fillASNDeltaPosition(v2x_msgs::msg::DeltaPosition ros_delta_position, DeltaPosition* asn_delta_position) {
  asn_delta_position->deltaLatitude = ros_delta_position.delta_latitude.delta_latitude;
  asn_delta_position->deltaLongitude = ros_delta_position.delta_longitude.delta_longitude;
}

void IVIMUtils::fillASNTcPart(v2x_msgs::msg::TcPart ros_tc_part, TcPart* asn_tc_part) {
  assert(asn_tc_part);

  //OPTIONAL: detectionZoneIds
  if (ros_tc_part.detection_zone_ids_present) {
    asn_tc_part->bit_mask |= TcPart_detectionZoneIds_present;

    asn_tc_part->detectionZoneIds = (ZoneIds_*) IVIMUtils::AllocateClearedMemory(sizeof(ZoneIds_));
    ZoneIds_* asn_direction_zone_ids = asn_tc_part->detectionZoneIds;

    auto &ros_direction_zone_ids = ros_tc_part.detection_zone_ids.ids;

    for (int i = 0; i < ros_direction_zone_ids.size(); ++i) {
      // allocate and switch to next in linked list
      if(i != 0){
        asn_direction_zone_ids->next = (ZoneIds_*) IVIMUtils::AllocateClearedMemory(sizeof(ZoneIds_));
        asn_direction_zone_ids = asn_direction_zone_ids->next;
      }
      auto &ros_direction_zone_id = ros_direction_zone_ids.at(i);

      IVIMUtils::fillASNZoneId(ros_direction_zone_id, &(asn_direction_zone_ids->value));
    }
  }

  //relevanceZoneIds
  asn_tc_part->relevanceZoneIds = (ZoneIds_*) IVIMUtils::AllocateClearedMemory(sizeof(ZoneIds_));
  ZoneIds_* asn_relevance_zone_ids = asn_tc_part->relevanceZoneIds;
  auto &ros_relevance_zone_ids = ros_tc_part.relevance_zone_ids.ids;

  for (int i = 0; i < ros_relevance_zone_ids.size(); ++i) {
    // allocate and switch to next in linked list
    if (i != 0){
      asn_relevance_zone_ids->next = (ZoneIds_*) IVIMUtils::AllocateClearedMemory(sizeof(ZoneIds_));
      asn_relevance_zone_ids = asn_relevance_zone_ids->next;
    }
    auto &ros_relevance_zone_id = ros_relevance_zone_ids.at(i);

    IVIMUtils::fillASNZoneId(ros_relevance_zone_id, &(asn_relevance_zone_ids->value));
  }

  //OPTIONAL: direction
  if (ros_tc_part.direction_present) {
    asn_tc_part->bit_mask |= TcPart_direction_present;
    asn_tc_part->direction = ros_tc_part.direction.direction;
  }

  //OPTIONAL: driverAwarenessZoneIds
  if (ros_tc_part.driver_awareness_zone_ids_present) {
    asn_tc_part->bit_mask |= TcPart_driverAwarenessZoneIds_present;

    asn_tc_part->driverAwarenessZoneIds = (ZoneIds_ *) IVIMUtils::AllocateClearedMemory(sizeof(ZoneIds_));
    ZoneIds_* asn_driver_awareness_zone_ids = asn_tc_part->driverAwarenessZoneIds;
    auto &ros_driver_awareness_zone_ids = ros_tc_part.driver_awareness_zone_ids.ids;

    for (int i = 0; i < ros_relevance_zone_ids.size(); ++i) {
      // allocate and switch to next in linked list
      if (i != 0){
        asn_driver_awareness_zone_ids->next = (ZoneIds_ *) IVIMUtils::AllocateClearedMemory(sizeof(ZoneIds_));
        asn_driver_awareness_zone_ids = asn_driver_awareness_zone_ids->next;
      }

      auto &ros_driver_awareness_zone_id = ros_driver_awareness_zone_ids.at(i);

      IVIMUtils::fillASNZoneId(ros_driver_awareness_zone_id, &(asn_driver_awareness_zone_ids->value));
    }
  }

  //OPTIONAL: minimumAwarenessTime
  if (ros_tc_part.minimum_awareness_time_present) {
    asn_tc_part->bit_mask |= TcPart_minimumAwarenessTime_present;
    asn_tc_part->minimumAwarenessTime = ros_tc_part.minimum_awareness_time;
  }

  //OPTIONAL: applicableLanes
  if (ros_tc_part.applicable_lanes_present) {
    asn_tc_part->bit_mask |= TcPart_applicableLanes_present;

    asn_tc_part->applicableLanes = (LanePositions_ *) IVIMUtils::AllocateClearedMemory(sizeof(LanePositions_));
    LanePositions_* asn_applicable_lanes = asn_tc_part->applicableLanes;

    auto &ros_applicable_lanes = ros_tc_part.applicable_lanes.positions;

    for (int i = 0; i < ros_applicable_lanes.size(); ++i) {
      // allocate and switch to next in linked list
      if (i != 0){
        asn_applicable_lanes->next = (LanePositions_ *) IVIMUtils::AllocateClearedMemory(sizeof(LanePositions_));
        asn_applicable_lanes = asn_applicable_lanes->next;
      }
      auto &ros_applicable_lane = ros_applicable_lanes.at(i);

      IVIMUtils::fillASNLanePosition(ros_applicable_lane, &(asn_applicable_lanes->value));
    }
  }


  //OPTIONAL: layoutId
  if (ros_tc_part.layout_id_present) {
    asn_tc_part->bit_mask |= TcPart_layoutId_present;
    asn_tc_part->layoutId = ros_tc_part.layout_id;
  }

  //OPTIONAL: preStoredlayoutId
  if (ros_tc_part.pre_storedlayout_id_present) {
    asn_tc_part->bit_mask |= TcPart_preStoredlayoutId_present;
    asn_tc_part->preStoredlayoutId = ros_tc_part.pre_storedlayout_id;
  }

  //OPTIONAL: text
  if (ros_tc_part.text_present) {
    asn_tc_part->bit_mask |= text_present;

    asn_tc_part->text = (TextLines_ *) IVIMUtils::AllocateClearedMemory(sizeof(TextLines_));
    auto asn_text_lines = asn_tc_part->text;

    auto &ros_text_lines = ros_tc_part.text.lines;

    for (int i = 0; i < ros_text_lines.size(); ++i) {
      // allocate and switch to next in linked list
      if (i != 0){
        asn_text_lines->next = (TextLines_ *) IVIMUtils::AllocateClearedMemory(sizeof(TextLines_));
        asn_text_lines = asn_text_lines->next;
      }
      auto &ros_text = ros_text_lines.at(i);

      IVIMUtils::fillASNText(ros_text, &(asn_text_lines->value));
    }
  }

  //data //Not implemented
  //ossConvertTBCDStringToOctet((OssGlobal*) world_, ros_tc_part.data, ros_tc_part.data.size(), &asn_tc_part->data);

  // Extension in V2
  //OPTIONAL: iviType
  asn_tc_part->bit_mask |= iviType_present;
  asn_tc_part->iviType = ros_tc_part.ivi_type.ivi_type;

  //OPTIONAL: laneStatus
  if (ros_tc_part.lane_status_present) {
    asn_tc_part->bit_mask |= TcPart_laneStatus_present;
    asn_tc_part->laneStatus = ros_tc_part.lane_status.lane_status;
  }

  //OPTIONAL: vehicleCharacteristics
  if (ros_tc_part.vehicle_characteristics_present) {
    // not implemented
    //asn_tc_part->bit_mask |= TcPart_vehicleCharacteristics_present;
    asn_tc_part->vehicleCharacteristics = nullptr;
  }
}

void IVIMUtils::fillASNZoneId(v2x_msgs::msg::Zid ros_zone_id, Zid* asn_zone_id) {
  *asn_zone_id = ros_zone_id.zid;
}

void IVIMUtils::fillASNLanePosition(v2x_msgs::msg::LanePosition ros_lane_position, LanePosition* asn_lane_position) {
  *asn_lane_position = ros_lane_position.lane_position;
}

void IVIMUtils::fillASNText(v2x_msgs::msg::Text ros_text, Text* asn_text) {
  if (ros_text.layout_component_id_present) {
    asn_text->bit_mask |= Text_layoutComponentId_present;
    asn_text->layoutComponentId = ros_text.layout_component_id;
  }
  //TODO: check
  asn_text->language = int64_t_to_bit2(ros_text.language);
  asn_text->textContent = (unsigned char *)IVIMUtils::AllocateClearedMemory(sizeof(unsigned char)*ros_text.text_content.length() +1);
  std::strcpy ((char*)asn_text->textContent, ros_text.text_content.c_str());
}


v2x_msgs::msg::IviContainer IVIMUtils::GetROSIviContainer(IviContainer* asn_ivi_container) {

  v2x_msgs::msg::IviContainer ros_ivi_container;

  if (asn_ivi_container->choice == glc_chosen) {
    ros_ivi_container.ivi_container_container_select = ros_ivi_container.IVI_CONTAINER_GLC;

    GeographicLocationContainer* asn_glc = &(asn_ivi_container->u.glc);
    auto &ros_glc = ros_ivi_container.glc;

    ros_glc.reference_position.latitude.latitude = asn_glc->referencePosition.latitude;
    ros_glc.reference_position.longitude.longitude = asn_glc->referencePosition.longitude;

    ros_glc.reference_position.position_confidence_ellipse.semi_major_confidence.semi_axis_length = asn_glc->referencePosition.positionConfidenceEllipse.semiMajorConfidence;
    ros_glc.reference_position.position_confidence_ellipse.semi_minor_confidence.semi_axis_length = asn_glc->referencePosition.positionConfidenceEllipse.semiMinorConfidence;
    ros_glc.reference_position.position_confidence_ellipse.semi_major_orientation.heading_value = asn_glc->referencePosition.positionConfidenceEllipse.semiMajorOrientation;

    ros_glc.reference_position.altitude.altitude_value.altitude_value = asn_glc->referencePosition.altitude.altitudeValue;
    ros_glc.reference_position.altitude.altitude_confidence.altitude_confidence = asn_glc->referencePosition.altitude.altitudeConfidence;

    if (asn_glc->bit_mask & referencePositionTime_present) {
      ros_glc.reference_position_time_present = true;
      ros_glc.reference_position_time.timestamp_its = asn_glc->referencePositionTime;
    }

    if (asn_glc->bit_mask & referencePositionHeading_present) {
      ros_glc.reference_position_heading_present = false;
      // not implemented
    }

    if (asn_glc->bit_mask & referencePositionSpeed_present) {
      ros_glc.reference_position_speed_present = false;
      // not implemented
    }

    GlcParts_* asn_glc_parts = asn_glc->parts;
    auto &ros_glc_parts = ros_glc.parts.parts;

    while(asn_glc_parts != nullptr){
      auto ros_glc_part = IVIMUtils::GetROSGlcPart(&(asn_glc_parts->value));

      ros_glc_parts.push_back(ros_glc_part);
      asn_glc_parts = asn_glc_parts->next;
    }
  }else if (asn_ivi_container->choice == tc_chosen) {
    ros_ivi_container.ivi_container_container_select = ros_ivi_container.IVI_CONTAINER_TC;

    TextContainer_* asn_tc = asn_ivi_container->u.tc;

    auto &ros_tc = ros_ivi_container.tc;
    auto &ros_tc_parts = ros_tc.container;

    while(asn_tc != nullptr){
      auto ros_tc_part = IVIMUtils::GetROSTcPart(&(asn_tc->value));

      ros_tc_parts.push_back(ros_tc_part);
    }
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "GetROSIviContainer: This ivi container is not implemented: %u",
                asn_ivi_container->choice);
  }

  return ros_ivi_container;
}

v2x_msgs::msg::GlcPart IVIMUtils::GetROSGlcPart(GlcPart* asn_glc_part) {

  v2x_msgs::msg::GlcPart ros_glc_part;

  ros_glc_part.zone_id.zid = asn_glc_part->zoneId;

  if (asn_glc_part->bit_mask & laneNumber_present) {
    ros_glc_part.lane_number_present = true;
    ros_glc_part.lane_number.lane_position = asn_glc_part->laneNumber;
  }

  if (asn_glc_part->bit_mask & zoneExtension_present) {
    ros_glc_part.zone_extension_present = true;
    ros_glc_part.zone_extension = asn_glc_part->zoneExtension;
  }

  if (asn_glc_part->bit_mask & zoneHeading_present) {
    ros_glc_part.zone_heading_present = true;
    ros_glc_part.zone_heading.heading_value = asn_glc_part->zoneHeading;
  }

  if (asn_glc_part->bit_mask & zone_present) {
    ros_glc_part.zone_present = true;
    ros_glc_part.zone = IVIMUtils::GetROSZone(asn_glc_part->zone);
  }

  return ros_glc_part;
}

v2x_msgs::msg::Zone IVIMUtils::GetROSZone(Zone asn_zone) {

  v2x_msgs::msg::Zone ros_zone;

  if (asn_zone.choice == segment_chosen) {
    ros_zone.zone_container_select = ros_zone.ZONE_SEGMENT;

    auto &asn_segment_zone = asn_zone.u.segment;
    auto &ros_segment_zone = ros_zone.segment;

    ros_segment_zone.line = IVIMUtils::GetROSPolygonalLine(&asn_segment_zone.line);

    if (asn_segment_zone.laneWidth) {
      ros_segment_zone.lane_width_present = true;
      ros_segment_zone.lane_width.ivi_lane_width = asn_segment_zone.laneWidth;
    }
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "GetROSZone: This zone is not implemented: %u",
                asn_zone.choice);
  }

  return ros_zone;
}

v2x_msgs::msg::PolygonalLine IVIMUtils::GetROSPolygonalLine(PolygonalLine* asn_polygonal_line)
{
  v2x_msgs::msg::PolygonalLine ros_polygonal_line;

  if(asn_polygonal_line->choice == deltaPositions_chosen) {
    ros_polygonal_line.polygonal_line_container_select = ros_polygonal_line.POLYGONAL_LINE_DELTA_POSITIONS;

    DeltaPositions_* asn_polygonal_line_delta_positions = asn_polygonal_line->u.deltaPositions;

    auto &ros_polygonal_line_delta_positions = ros_polygonal_line.delta_positions;
    auto &ros_delta_positions = ros_polygonal_line_delta_positions.positions;

    while(asn_polygonal_line_delta_positions != nullptr){
      auto ros_delta_position = IVIMUtils::GetROSDeltaPosition(&asn_polygonal_line_delta_positions->value);
      ros_delta_positions.push_back(ros_delta_position);
      asn_polygonal_line_delta_positions = asn_polygonal_line_delta_positions->next;
    }
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "GetROSPolygonalLine: This polygonal line is not implemented: %u",
                asn_polygonal_line->choice);
  }

  return ros_polygonal_line;
}

v2x_msgs::msg::DeltaPosition IVIMUtils::GetROSDeltaPosition(DeltaPosition* asn_delta_position) {

  v2x_msgs::msg::DeltaPosition ros_delta_position;

  ros_delta_position.delta_latitude.delta_latitude = asn_delta_position->deltaLatitude;
  ros_delta_position.delta_longitude.delta_longitude = asn_delta_position->deltaLongitude;

  return ros_delta_position;
}

v2x_msgs::msg::TcPart IVIMUtils::GetROSTcPart(TcPart* asn_tc_part) {

  v2x_msgs::msg::TcPart ros_tc_part;

  if (asn_tc_part->bit_mask & TcPart_detectionZoneIds_present) {
    ros_tc_part.detection_zone_ids_present = true;

    ZoneIds_* asn_direction_zone_ids = asn_tc_part->detectionZoneIds;
    auto &ros_direction_zone_ids = ros_tc_part.detection_zone_ids.ids;

    while(asn_direction_zone_ids != nullptr){
      auto ros_direction_zone_id = IVIMUtils::GetROSZoneId(&(asn_direction_zone_ids->value));

      ros_direction_zone_ids.push_back(ros_direction_zone_id);
      asn_direction_zone_ids = asn_direction_zone_ids->next;
    }
  }

  ZoneIds_* asn_relevance_zone_ids = asn_tc_part->relevanceZoneIds;
  auto &ros_relevance_zone_ids = ros_tc_part.relevance_zone_ids.ids;

  while(asn_relevance_zone_ids != nullptr){
    auto ros_relevance_zone_id = IVIMUtils::GetROSZoneId(&(asn_relevance_zone_ids->value));

    ros_relevance_zone_ids.push_back(ros_relevance_zone_id);
    asn_relevance_zone_ids = asn_relevance_zone_ids->next;
  }

  if (asn_tc_part->bit_mask & TcPart_direction_present) {
    ros_tc_part.direction_present = true;
    ros_tc_part.direction.direction = asn_tc_part->direction;
  }

  if (asn_tc_part->bit_mask & TcPart_driverAwarenessZoneIds_present) {
    ros_tc_part.driver_awareness_zone_ids_present = true;

    ZoneIds_* asn_driver_awareness_zone_ids = asn_tc_part->driverAwarenessZoneIds;
    auto &ros_driver_awareness_zone_ids = ros_tc_part.driver_awareness_zone_ids.ids;

    while( asn_driver_awareness_zone_ids != nullptr){
      auto ros_driver_awareness_zone_id = IVIMUtils::GetROSZoneId(&(asn_driver_awareness_zone_ids->value));

      ros_driver_awareness_zone_ids.push_back(ros_driver_awareness_zone_id);
      asn_driver_awareness_zone_ids = asn_driver_awareness_zone_ids->next;
    }
  }

  if (asn_tc_part->bit_mask & TcPart_minimumAwarenessTime_present) {
    ros_tc_part.minimum_awareness_time_present = true;
    ros_tc_part.minimum_awareness_time = asn_tc_part->minimumAwarenessTime;
  }

  if (asn_tc_part->bit_mask & TcPart_applicableLanes_present) {
    ros_tc_part.applicable_lanes_present = true;

    LanePositions_* asn_applicable_lanes = asn_tc_part->applicableLanes;
    auto &ros_applicable_lanes = ros_tc_part.applicable_lanes.positions;

    while (asn_applicable_lanes != nullptr){
      auto ros_applicable_lane = IVIMUtils::GetROSLanePosition(&(asn_applicable_lanes->value));

      ros_applicable_lanes.push_back(ros_applicable_lane);
      asn_applicable_lanes = asn_applicable_lanes->next;
    }
  }

  if (asn_tc_part->bit_mask & TcPart_layoutId_present) {
    ros_tc_part.layout_id_present = true;
    ros_tc_part.layout_id = asn_tc_part->layoutId;
  }

  if (asn_tc_part->bit_mask & TcPart_preStoredlayoutId_present) {
    ros_tc_part.pre_storedlayout_id_present = true;
    ros_tc_part.pre_storedlayout_id = asn_tc_part->preStoredlayoutId;
  }

  if (asn_tc_part->bit_mask & text_present) {
    ros_tc_part.text_present = true;

    TextLines_* asn_text_lines = asn_tc_part->text;
    auto &ros_text_lines = ros_tc_part.text.lines;

    while(asn_text_lines != nullptr){
      auto ros_text = IVIMUtils::GetROSText(&(asn_text_lines->value));

      ros_text_lines.push_back(ros_text);
      asn_text_lines = asn_text_lines->next;
    }
  }

  //not implemented
  ros_tc_part.data;

  if (asn_tc_part->bit_mask & iviType_present) {
    ros_tc_part.ivi_type.ivi_type = asn_tc_part->iviType;

    if (asn_tc_part->bit_mask & TcPart_laneStatus_present) {
      ros_tc_part.lane_status_present = true;
      ros_tc_part.lane_status.lane_status =  asn_tc_part->laneStatus;
    }

    if (asn_tc_part->bit_mask & TcPart_vehicleCharacteristics_present) {
      ros_tc_part.vehicle_characteristics_present = false;
      // not implemented
    }
  }

  return ros_tc_part;
}

v2x_msgs::msg::Zid IVIMUtils::GetROSZoneId(Zid* asn_zone_id) {
  v2x_msgs::msg::Zid ros_zone_id;
  ros_zone_id.zid = *asn_zone_id;
  return ros_zone_id;
}

v2x_msgs::msg::LanePosition IVIMUtils::GetROSLanePosition(LanePosition* asn_lane_position) {
  v2x_msgs::msg::LanePosition ros_lane_position;
  ros_lane_position.lane_position = *asn_lane_position;
  return ros_lane_position;
}

v2x_msgs::msg::Text IVIMUtils::GetROSText(Text* asn_text) {

  v2x_msgs::msg::Text ros_text;

  if (asn_text->bit_mask & Text_layoutComponentId_present) {
    ros_text.layout_component_id_present = true;
    ros_text.layout_component_id = asn_text->layoutComponentId;
  }

  ros_text.language = bitstream_to_int64(asn_text->language.length, asn_text->language.value);
  ros_text.text_content = std::string(reinterpret_cast<char*>(asn_text->textContent));

  return ros_text;
}


void* IVIMUtils::AllocateClearedMemory(size_t bytes) {
  void* allocated_memory = malloc(bytes);
  memset(allocated_memory, 0, bytes);
  return allocated_memory;
}


_bit2 IVIMUtils::int64_t_to_bit2(int64_t int_64_t) {
  _bit2 bit_string;

  // calc nr of used bits
  uint64_t unused_bits = 0;
  uint64_t bitmask = 1;
  bitmask = bitmask << 63;

  while(!(int_64_t & bitmask) && unused_bits<64){
    bitmask = bitmask >> 1;
    unused_bits ++;
  }

  bit_string.length = 64-unused_bits;

  int char_length = bit_string.length / sizeof(uint8_t) + (bit_string.length % sizeof(uint8_t) == 0 ? 0 : 1);
  bit_string.value = 0;

  if (!bit_string.length)
    return bit_string;

  bit_string.value = (unsigned char *) AllocateClearedMemory(sizeof(unsigned char) * char_length);

  uint64_t char_num = char_length-1;
  uint64_t i = 0;

  for (; i < char_length - 1; ++i, --char_num)
     bit_string.value[i] = (unsigned char)(int_64_t >> (sizeof(unsigned char) * i)| (0xff));

  return bit_string;
}

int64_t IVIMUtils::bitstream_to_int64(int length, unsigned char* value){
  int64_t number = 0;
  int char_length = length / sizeof(uint8_t) + (length % sizeof(uint8_t) == 0 ? 0 : 1);
  for(int i = 0; i<char_length; i++){
    number |= ((int64_t)value[i])<<(i*sizeof(unsigned char));
  }
  return number;
}
