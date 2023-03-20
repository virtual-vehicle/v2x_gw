#include "IVIMUtils.h"
#include "IVIMHandler.h"

rclcpp::Node * IVIMUtils::node_ = nullptr;

void IVIMUtils::setNode(rclcpp::Node * node)
{
  node_ = node;
}

void IVIMUtils::fillASNIviContainer(v2x_msgs::msg::IviContainer ros_ivi_container, IviContainer_t* asn_ivi_container) {
  assert(asn_ivi_container);

  switch (ros_ivi_container.ivi_container_container_select) {

    case ros_ivi_container.IVI_CONTAINER_NOTHING: {
      asn_ivi_container->present = IviContainer_PR_NOTHING;
    }
      break;

    case ros_ivi_container.IVI_CONTAINER_GLC: {
      asn_ivi_container->present = IviContainer_PR_glc;

      auto &asn_glc = asn_ivi_container->choice.glc;
      auto &ros_glc = ros_ivi_container.glc;

      asn_glc.referencePosition.latitude = ros_glc.reference_position.latitude.latitude;
      asn_glc.referencePosition.longitude = ros_glc.reference_position.longitude.longitude;

      asn_glc.referencePosition.positionConfidenceEllipse.semiMajorConfidence = ros_glc.reference_position.position_confidence_ellipse.semi_major_confidence.semi_axis_length;
      asn_glc.referencePosition.positionConfidenceEllipse.semiMinorConfidence = ros_glc.reference_position.position_confidence_ellipse.semi_minor_confidence.semi_axis_length;
      asn_glc.referencePosition.positionConfidenceEllipse.semiMajorOrientation = ros_glc.reference_position.position_confidence_ellipse.semi_major_orientation.heading_value;

      asn_glc.referencePosition.altitude.altitudeValue = ros_glc.reference_position.altitude.altitude_value.altitude_value;
      asn_glc.referencePosition.altitude.altitudeConfidence = ros_glc.reference_position.altitude.altitude_confidence.altitude_confidence;

      if (ros_glc.reference_position_time_present) {
        // asn_imax2INTEGER allocates memory for asn_glc.referencePositionTime
        if (asn_imax2INTEGER(asn_glc.referencePositionTime, ros_glc.reference_position_time.timestamp_its) == -1) {
          // TODO: handle error
        }
      }

      if (ros_glc.reference_position_heading_present) {
        asn_glc.referencePositionHeading = nullptr;
        // not implemented
      }

      if (ros_glc.reference_position_speed_present) {
        asn_glc.referencePositionSpeed = nullptr;
        // not implemented
      }

      auto asn_glc_parts = &asn_glc.parts.list;
      auto &ros_glc_parts = ros_glc.parts.parts;

      for (int i = 0; i < ros_glc_parts.size(); ++i) {
        auto asn_glc_part = (GlcPart_t *) IVIMUtils::AllocateClearedMemory(sizeof(GlcPart_t));
        auto &ros_glc_part = ros_glc_parts.at(i);

        IVIMUtils::fillASNGlcPart(ros_glc_part, asn_glc_part);

        CHECK_FAIL(ASN_SEQUENCE_ADD(asn_glc_parts, asn_glc_part),
                   "ASN_SEQUENCE_ADD failed for asn_glc_parts");
      }
    }
      break;

    case ros_ivi_container.IVI_CONTAINER_TC: {
      asn_ivi_container->present = IviContainer_PR_tc;

      auto &asn_tc = asn_ivi_container->choice.tc;
      auto &ros_tc = ros_ivi_container.tc;

      auto asn_tc_parts = &asn_tc.list;
      auto &ros_tc_parts = ros_tc.container;

      for (int i = 0; i < ros_tc_parts.size(); ++i) {
        auto asn_tc_part = (TcPart_t *) IVIMUtils::AllocateClearedMemory(sizeof(TcPart_t));
        auto& ros_tc_part = ros_tc_parts.at(i);

//        fillASNTcPart(ros_tc_part, asn_tc_part);

        CHECK_FAIL(ASN_SEQUENCE_ADD(asn_tc_parts, asn_tc_part),
                   "ASN_SEQUENCE_ADD failed for asn_tc_parts");
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

void IVIMUtils::fillASNGlcPart(v2x_msgs::msg::GlcPart ros_glc_part, GlcPart_t* asn_glc_part) {
  assert(asn_glc_part);

  asn_glc_part->zoneId = ros_glc_part.zone_id.zid;

  if (ros_glc_part.lane_number_present) {
    asn_glc_part->laneNumber = (LanePosition_t *) IVIMUtils::AllocateClearedMemory(sizeof(LanePosition_t));
    *asn_glc_part->laneNumber = ros_glc_part.lane_number.lane_position;
  }

  if (ros_glc_part.zone_extension_present) {
    asn_glc_part->zoneExtension = (long *) IVIMUtils::AllocateClearedMemory(sizeof(long));
    *asn_glc_part->zoneExtension = ros_glc_part.zone_extension;
  }

  if (ros_glc_part.zone_heading_present) {
    asn_glc_part->zoneHeading = (HeadingValue_t *) IVIMUtils::AllocateClearedMemory(sizeof(HeadingValue_t));
    *asn_glc_part->zoneHeading = ros_glc_part.zone_heading.heading_value;
  }

  if (ros_glc_part.zone_present) {
    asn_glc_part->zone = (Zone_t *) IVIMUtils::AllocateClearedMemory(sizeof(Zone_t));
    IVIMUtils::fillASNZone(ros_glc_part.zone, asn_glc_part->zone);
  }
}

void IVIMUtils::fillASNZone(v2x_msgs::msg::Zone ros_zone, Zone_t* asn_zone) {
  assert(asn_zone);

  switch (ros_zone.zone_container_select) {

    case ros_zone.ZONE_NOTHING: {
      asn_zone->present = Zone_PR_NOTHING;
    }
      break;

    case ros_zone.ZONE_SEGMENT: {
      asn_zone->present = Zone_PR_segment;

      auto &asn_segment_zone = asn_zone->choice.segment;
      auto &ros_segment_zone = ros_zone.segment;

      IVIMUtils::fillASNPolygonalLine(ros_segment_zone.line, &asn_segment_zone.line);

      if (ros_segment_zone.lane_width_present) {
        asn_segment_zone.laneWidth = (IviLaneWidth_t *) IVIMUtils::AllocateClearedMemory(sizeof(IviLaneWidth_t));
        *asn_segment_zone.laneWidth = ros_segment_zone.lane_width.ivi_lane_width;
      }
    }
      break;

    default:  if (node_) {
      RCLCPP_INFO(node_->get_logger(),
                  "fillASNZone: This zone is not implemented: %ld",
                  ros_zone.zone_container_select);
    }
      break;
  }
}

void IVIMUtils::fillASNPolygonalLine(v2x_msgs::msg::PolygonalLine ros_polygonal_line, PolygonalLine_t* asn_polygonal_line) {
  assert(asn_polygonal_line);

  switch(ros_polygonal_line.polygonal_line_container_select) {

    case ros_polygonal_line.POLYGONAL_LINE_NOTHING: {
      asn_polygonal_line->present = PolygonalLine_PR_NOTHING;
    }
      break;

    case ros_polygonal_line.POLYGONAL_LINE_DELTA_POSITIONS: {
      asn_polygonal_line->present = PolygonalLine_PR_deltaPositions;

      auto &asn_polygonal_line_delta_positions = asn_polygonal_line->choice.deltaPositions;
      auto &ros_polygonal_line_delta_positions = ros_polygonal_line.delta_positions;

      auto asn_delta_positions = &asn_polygonal_line_delta_positions.list;
      auto &ros_delta_positions = ros_polygonal_line_delta_positions.positions;

      for (int i = 0; i < ros_delta_positions.size(); ++i) {
        auto asn_delta_position = (DeltaPosition_t *) IVIMUtils::AllocateClearedMemory(sizeof(DeltaPosition_t));
        auto &ros_delta_position = ros_delta_positions.at(i);

        IVIMUtils::fillASNDeltaPosition(ros_delta_position, asn_delta_position);

        CHECK_FAIL(ASN_SEQUENCE_ADD(asn_delta_positions, asn_delta_position),
                   "ASN_SEQUENCE_ADD failed for asn_delta_positions");
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

void IVIMUtils::fillASNDeltaPosition(v2x_msgs::msg::DeltaPosition ros_delta_position, DeltaPosition_t* asn_delta_position) {
  asn_delta_position->deltaLatitude = ros_delta_position.delta_latitude.delta_latitude;
  asn_delta_position->deltaLongitude = ros_delta_position.delta_longitude.delta_longitude;
}

void IVIMUtils::fillASNTcPart(v2x_msgs::msg::TcPart ros_tc_part, TcPart_t* asn_tc_part) {
  assert(asn_tc_part);

  if (ros_tc_part.detection_zone_ids_present) {
    asn_tc_part->detectionZoneIds = (ZoneIds_t*) IVIMUtils::AllocateClearedMemory(sizeof(ZoneIds_t));

    auto asn_direction_zone_ids = &asn_tc_part->detectionZoneIds->list;
    auto &ros_direction_zone_ids = ros_tc_part.detection_zone_ids.ids;

    for (int i = 0; i < ros_direction_zone_ids.size(); ++i) {
      auto asn_direction_zone_id = (Zid_t *) IVIMUtils::AllocateClearedMemory(sizeof(Zid_t));
      auto &ros_direction_zone_id = ros_direction_zone_ids.at(i);

      IVIMUtils::fillASNZoneId(ros_direction_zone_id, asn_direction_zone_id);

      CHECK_FAIL(ASN_SEQUENCE_ADD(asn_direction_zone_ids, asn_direction_zone_id),
                 "ASN_SEQUENCE_ADD failed for asn_direction_zone_ids");
    }
  }

  auto asn_relevance_zone_ids = &asn_tc_part->relevanceZoneIds.list;
  auto &ros_relevance_zone_ids = ros_tc_part.relevance_zone_ids.ids;

  for (int i = 0; i < ros_relevance_zone_ids.size(); ++i) {
    auto asn_relevance_zone_id = (Zid_t *) IVIMUtils::AllocateClearedMemory(sizeof(Zid_t));
    auto &ros_relevance_zone_id = ros_relevance_zone_ids.at(i);

    IVIMUtils::fillASNZoneId(ros_relevance_zone_id, asn_relevance_zone_id);

    CHECK_FAIL(ASN_SEQUENCE_ADD(asn_relevance_zone_ids, asn_relevance_zone_id),
               "ASN_SEQUENCE_ADD failed for asn_relevance_zone_ids");
  }

  if (ros_tc_part.direction_present) {
    asn_tc_part->direction = (Direction_t *) IVIMUtils::AllocateClearedMemory(sizeof(Direction_t));
    *asn_tc_part->direction = ros_tc_part.direction.direction;
  }

  if (ros_tc_part.driver_awareness_zone_ids_present) {
    asn_tc_part->driverAwarenessZoneIds = (ZoneIds_t *) IVIMUtils::AllocateClearedMemory(sizeof(ZoneIds_t));

    auto asn_driver_awareness_zone_ids = &asn_tc_part->driverAwarenessZoneIds->list;
    auto &ros_driver_awareness_zone_ids = ros_tc_part.driver_awareness_zone_ids.ids;

    for (int i = 0; i < ros_relevance_zone_ids.size(); ++i) {
      auto asn_driver_awareness_zone_id = (Zid_t *) IVIMUtils::AllocateClearedMemory(sizeof(Zid_t));
      auto &ros_driver_awareness_zone_id = ros_driver_awareness_zone_ids.at(i);

      IVIMUtils::fillASNZoneId(ros_driver_awareness_zone_id, asn_driver_awareness_zone_id);

      CHECK_FAIL(ASN_SEQUENCE_ADD(asn_driver_awareness_zone_ids, asn_driver_awareness_zone_id),
                 "ASN_SEQUENCE_ADD failed for asn_driver_awareness_zone_ids");
    }
  }

  if (ros_tc_part.minimum_awareness_time_present) {
    asn_tc_part->minimumAwarenessTime = (long *) IVIMUtils::AllocateClearedMemory(sizeof(long));
    *asn_tc_part->minimumAwarenessTime = ros_tc_part.minimum_awareness_time;
  }

  if (ros_tc_part.applicable_lanes_present) {
    asn_tc_part->applicableLanes = (LanePositions_t *) IVIMUtils::AllocateClearedMemory(sizeof(LanePositions_t));

    auto asn_applicable_lanes = &asn_tc_part->applicableLanes->list;
    auto &ros_applicable_lanes = ros_tc_part.applicable_lanes.positions;

    for (int i = 0; i < ros_applicable_lanes.size(); ++i) {
      auto asn_applicable_lane = (LanePosition_t *) IVIMUtils::AllocateClearedMemory(sizeof(LanePosition_t));
      auto &ros_applicable_lane = ros_applicable_lanes.at(i);

      IVIMUtils::fillASNLanePosition(ros_applicable_lane, asn_applicable_lane);

      CHECK_FAIL(ASN_SEQUENCE_ADD(asn_applicable_lanes, asn_applicable_lane),
                 "ASN_SEQUENCE_ADD failed for asn_applicable_lanes");
    }
  }

  if (ros_tc_part.layout_id_present) {
    asn_tc_part->layoutId = (long *) IVIMUtils::AllocateClearedMemory(sizeof(long));
    *asn_tc_part->layoutId = ros_tc_part.layout_id;
  }

  if (ros_tc_part.pre_storedlayout_id_present) {
    asn_tc_part->preStoredlayoutId = (long *) IVIMUtils::AllocateClearedMemory(sizeof(long));
    *asn_tc_part->preStoredlayoutId = ros_tc_part.pre_storedlayout_id;
  }

  if (ros_tc_part.text_present) {
    asn_tc_part->text = (TextLines_t *) IVIMUtils::AllocateClearedMemory(sizeof(TextLines_t));

    auto asn_text_lines = &asn_tc_part->text->list;
    auto &ros_text_lines = ros_tc_part.text.lines;

    for (int i = 0; i < ros_text_lines.size(); ++i) {
      auto asn_text = (Text_t *) IVIMUtils::AllocateClearedMemory(sizeof(Text_t));
      auto &ros_text = ros_text_lines.at(i);

      IVIMUtils::fillASNText(ros_text, asn_text);

      CHECK_FAIL(ASN_SEQUENCE_ADD(asn_text_lines, asn_text),
                 "ASN_SEQUENCE_ADD failed for asn_text_lines");
    }
  }

  // TODO: asn_tc_part.data - how do i translate an OCTET_STRING ?

  asn_tc_part->ext1 = (TcPart_t::TcPart__ext1 *) IVIMUtils::AllocateClearedMemory(sizeof(TcPart_t::TcPart__ext1));

  asn_tc_part->ext1->iviType = ros_tc_part.ivi_type.ivi_type;

  if (ros_tc_part.lane_status_present) {
    asn_tc_part->ext1->laneStatus = (LaneStatus_t *) IVIMUtils::AllocateClearedMemory(sizeof(LaneStatus_t));
    *asn_tc_part->ext1->laneStatus = ros_tc_part.lane_status.lane_status;
  }

  if (ros_tc_part.vehicle_characteristics_present) {
    asn_tc_part->ext1->vehicleCharacteristics = nullptr;
    // not implemented
  }
}

void IVIMUtils::fillASNZoneId(v2x_msgs::msg::Zid ros_zone_id, Zid_t* asn_zone_id) {
  *asn_zone_id = ros_zone_id.zid;
}

void IVIMUtils::fillASNLanePosition(v2x_msgs::msg::LanePosition ros_lane_position, LanePosition_t* asn_lane_position) {
  *asn_lane_position = ros_lane_position.lane_position;
}

void IVIMUtils::fillASNText(v2x_msgs::msg::Text ros_text, Text_t* asn_text) {
  if (ros_text.layout_component_id_present) {
    asn_text->layoutComponentId = (long *) IVIMUtils::AllocateClearedMemory(sizeof(long));
    *asn_text->layoutComponentId = ros_text.layout_component_id;
  }

//  TODO: ros_text.language - issue : how do i translate the asn BIT_STRING ?

//  TODO: ros_text.text_content - issue : how do i translate the asn UTF8String ?

}


v2x_msgs::msg::IviContainer IVIMUtils::GetROSIviContainer(IviContainer_t* asn_ivi_container) {

  v2x_msgs::msg::IviContainer ros_ivi_container;

  switch (asn_ivi_container->present) {

    case IviContainer_PR_NOTHING: {
      ros_ivi_container.ivi_container_container_select = ros_ivi_container.IVI_CONTAINER_NOTHING;
    }
      break;

    case 	IviContainer_PR_glc: {
      ros_ivi_container.ivi_container_container_select = ros_ivi_container.IVI_CONTAINER_GLC;

      auto &asn_glc = asn_ivi_container->choice.glc;
      auto &ros_glc = ros_ivi_container.glc;

      ros_glc.reference_position.latitude.latitude = asn_glc.referencePosition.latitude;
      ros_glc.reference_position.longitude.longitude = asn_glc.referencePosition.longitude;

      ros_glc.reference_position.position_confidence_ellipse.semi_major_confidence.semi_axis_length = asn_glc.referencePosition.positionConfidenceEllipse.semiMajorConfidence;
      ros_glc.reference_position.position_confidence_ellipse.semi_minor_confidence.semi_axis_length = asn_glc.referencePosition.positionConfidenceEllipse.semiMinorConfidence;
      ros_glc.reference_position.position_confidence_ellipse.semi_major_orientation.heading_value = asn_glc.referencePosition.positionConfidenceEllipse.semiMajorOrientation;

      ros_glc.reference_position.altitude.altitude_value.altitude_value = asn_glc.referencePosition.altitude.altitudeValue;
      ros_glc.reference_position.altitude.altitude_confidence.altitude_confidence = asn_glc.referencePosition.altitude.altitudeConfidence;

      if (asn_glc.referencePositionTime) {
        ros_glc.reference_position_time_present = true;
        if (asn_INTEGER2imax(asn_glc.referencePositionTime, &ros_glc.reference_position_time.timestamp_its) == -1) {
          // TODO: handle error
        }
      }

      if (asn_glc.referencePositionHeading) {
        ros_glc.reference_position_heading_present = false;
        // not implemented
      }

      if (asn_glc.referencePositionSpeed) {
        ros_glc.reference_position_speed_present = false;
        // not implemented
      }

      auto &asn_glc_parts = asn_glc.parts.list;
      auto &ros_glc_parts = ros_glc.parts.parts;

      for (int i = 0; i < asn_glc_parts.count; ++i) {
        auto asn_glc_part = asn_glc_parts.array[i];
        auto ros_glc_part = IVIMUtils::GetROSGlcPart(asn_glc_part);

        ros_glc_parts.push_back(ros_glc_part);
      }
    }
      break;

    case IviContainer_PR_tc: {
      ros_ivi_container.ivi_container_container_select = ros_ivi_container.IVI_CONTAINER_TC;

      auto &asn_tc = asn_ivi_container->choice.tc;
      auto &ros_tc = ros_ivi_container.tc;

      auto &asn_tc_parts = asn_tc.list;
      auto &ros_tc_parts = ros_tc.container;

      for (int i = 0; i < asn_tc_parts.count; ++i) {
        auto asn_tc_part = asn_tc_parts.array[i];
        auto ros_tc_part = IVIMUtils::GetROSTcPart(asn_tc_part);

        ros_tc_parts.push_back(ros_tc_part);
      }

    }
      break;

    default: if (node_) {
      RCLCPP_INFO(node_->get_logger(),
                  "GetROSIviContainer: This ivi container is not implemented: %u",
                  asn_ivi_container->present);
    }
      break;
  }

  return ros_ivi_container;
}

v2x_msgs::msg::GlcPart IVIMUtils::GetROSGlcPart(GlcPart_t* asn_glc_part) {

  v2x_msgs::msg::GlcPart ros_glc_part;

  ros_glc_part.zone_id.zid = asn_glc_part->zoneId;

  if (asn_glc_part->laneNumber) {
    ros_glc_part.lane_number_present = true;
    ros_glc_part.lane_number.lane_position = *asn_glc_part->laneNumber;
  }

  if (asn_glc_part->zoneExtension) {
    ros_glc_part.zone_extension_present = true;
    ros_glc_part.zone_extension = *asn_glc_part->zoneExtension;
  }

  if (asn_glc_part->zoneHeading) {
    ros_glc_part.zone_heading_present = true;
    ros_glc_part.zone_heading.heading_value = *asn_glc_part->zoneHeading;
  }

  if (asn_glc_part->zone) {
    ros_glc_part.zone_present = true;
    ros_glc_part.zone = IVIMUtils::GetROSZone(asn_glc_part->zone);
  }

  return ros_glc_part;
}

v2x_msgs::msg::Zone IVIMUtils::GetROSZone(Zone_t* asn_zone) {

  v2x_msgs::msg::Zone ros_zone;

  switch (asn_zone->present) {

    case Zone_PR_NOTHING: {
      ros_zone.zone_container_select = ros_zone.ZONE_NOTHING;
    }
      break;

    case Zone_PR_segment: {
      ros_zone.zone_container_select = ros_zone.ZONE_SEGMENT;

      auto &asn_segment_zone = asn_zone->choice.segment;
      auto &ros_segment_zone = ros_zone.segment;

      ros_segment_zone.line = IVIMUtils::GetROSPolygonalLine(&asn_segment_zone.line);

      if (asn_segment_zone.laneWidth) {
        ros_segment_zone.lane_width_present = true;
        ros_segment_zone.lane_width.ivi_lane_width = *asn_segment_zone.laneWidth;
      }
    }
      break;

    default: if (node_) {
      RCLCPP_INFO(node_->get_logger(),
                  "GetROSZone: This zone is not implemented: %u",
                  asn_zone->present);
    }
      break;
  }

  return ros_zone;
}

v2x_msgs::msg::PolygonalLine IVIMUtils::GetROSPolygonalLine(PolygonalLine_t* asn_polygonal_line)
{
  v2x_msgs::msg::PolygonalLine ros_polygonal_line;

  switch(asn_polygonal_line->present) {

    case PolygonalLine_PR_NOTHING: {
      ros_polygonal_line.polygonal_line_container_select = ros_polygonal_line.POLYGONAL_LINE_NOTHING;
    }
      break;

    case PolygonalLine_PR_deltaPositions: {
      ros_polygonal_line.polygonal_line_container_select = ros_polygonal_line.POLYGONAL_LINE_DELTA_POSITIONS;

      auto &asn_polygonal_line_delta_positions = asn_polygonal_line->choice.deltaPositions;
      auto &ros_polygonal_line_delta_positions = ros_polygonal_line.delta_positions;

      auto &asn_delta_positions = asn_polygonal_line_delta_positions.list;
      auto &ros_delta_positions = ros_polygonal_line_delta_positions.positions;

      for (int i = 0; i < asn_delta_positions.count; ++i) {
        auto asn_delta_position = asn_delta_positions.array[i];
        auto ros_delta_position = IVIMUtils::GetROSDeltaPosition(asn_delta_position);

        ros_delta_positions.push_back(ros_delta_position);
      }
    }
      break;

    default: if (node_) {
      RCLCPP_INFO(node_->get_logger(),
                  "GetROSPolygonalLine: This polygonal line is not implemented: %u",
                  asn_polygonal_line->present);
    }
      break;
  }

  return ros_polygonal_line;
}

v2x_msgs::msg::DeltaPosition IVIMUtils::GetROSDeltaPosition(DeltaPosition_t* asn_delta_position) {

  v2x_msgs::msg::DeltaPosition ros_delta_position;

  ros_delta_position.delta_latitude.delta_latitude = asn_delta_position->deltaLatitude;
  ros_delta_position.delta_longitude.delta_longitude = asn_delta_position->deltaLongitude;

  return ros_delta_position;
}

v2x_msgs::msg::TcPart IVIMUtils::GetROSTcPart(TcPart_t* asn_tc_part) {

  v2x_msgs::msg::TcPart ros_tc_part;

  if (asn_tc_part->detectionZoneIds) {
    ros_tc_part.detection_zone_ids_present = true;

    auto &asn_direction_zone_ids = asn_tc_part->detectionZoneIds->list;
    auto &ros_direction_zone_ids = ros_tc_part.detection_zone_ids.ids;

    for (int i = 0; i < asn_direction_zone_ids.count; ++i) {
      auto asn_direction_zone_id = asn_direction_zone_ids.array[i];
      auto ros_direction_zone_id = IVIMUtils::GetROSZoneId(asn_direction_zone_id);

      ros_direction_zone_ids.push_back(ros_direction_zone_id);
    }
  }

  auto &asn_relevance_zone_ids = asn_tc_part->relevanceZoneIds.list;
  auto &ros_relevance_zone_ids = ros_tc_part.relevance_zone_ids.ids;

  for (int i = 0; i < asn_relevance_zone_ids.count; ++i) {
    auto asn_relevance_zone_id = asn_relevance_zone_ids.array[i];
    auto ros_relevance_zone_id = IVIMUtils::GetROSZoneId(asn_relevance_zone_id);

    ros_relevance_zone_ids.push_back(ros_relevance_zone_id);
  }

  if (asn_tc_part->direction) {
    ros_tc_part.direction_present = true;
    ros_tc_part.direction.direction = *asn_tc_part->direction;
  }

  if (asn_tc_part->driverAwarenessZoneIds) {
    ros_tc_part.driver_awareness_zone_ids_present = true;

    auto &asn_driver_awareness_zone_ids = asn_tc_part->driverAwarenessZoneIds->list;
    auto &ros_driver_awareness_zone_ids = ros_tc_part.driver_awareness_zone_ids.ids;

    for (int i = 0; i < asn_driver_awareness_zone_ids.count; ++i) {
      auto asn_driver_awareness_zone_id = asn_driver_awareness_zone_ids.array[i];
      auto ros_driver_awareness_zone_id = IVIMUtils::GetROSZoneId(asn_driver_awareness_zone_id);

      ros_driver_awareness_zone_ids.push_back(ros_driver_awareness_zone_id);
    }
  }

  if (asn_tc_part->minimumAwarenessTime) {
    ros_tc_part.minimum_awareness_time_present = true;
    ros_tc_part.minimum_awareness_time = *asn_tc_part->minimumAwarenessTime;
  }

  if (asn_tc_part->applicableLanes) {
    ros_tc_part.applicable_lanes_present = true;

    auto &asn_applicable_lanes = asn_tc_part->applicableLanes->list;
    auto &ros_applicable_lanes = ros_tc_part.applicable_lanes.positions;

    for (int i = 0; i < asn_applicable_lanes.count; ++i) {
      auto asn_applicable_lane = asn_applicable_lanes.array[i];
      auto ros_applicable_lane = IVIMUtils::GetROSLanePosition(asn_applicable_lane);

      ros_applicable_lanes.push_back(ros_applicable_lane);
    }
  }

  if (asn_tc_part->layoutId) {
    ros_tc_part.layout_id_present = true;
    ros_tc_part.layout_id = *asn_tc_part->layoutId;
  }

  if (asn_tc_part->preStoredlayoutId) {
    ros_tc_part.pre_storedlayout_id_present = true;
    ros_tc_part.pre_storedlayout_id = *asn_tc_part->preStoredlayoutId;
  }

  if (asn_tc_part->text) {
    ros_tc_part.text_present = true;

    auto &asn_text_lines = asn_tc_part->text->list;
    auto &ros_text_lines = ros_tc_part.text.lines;

    for (int i = 0; i < asn_text_lines.count; ++i) {
      auto asn_text = asn_text_lines.array[i];
      auto ros_text = IVIMUtils::GetROSText(asn_text);

      ros_text_lines.push_back(ros_text);
    }
  }

// TODO: asn_tc_part.data - how do i translate an OCTET_STRING ?

  if (asn_tc_part->ext1) {
    ros_tc_part.ivi_type.ivi_type = asn_tc_part->ext1->iviType;

    if (asn_tc_part->ext1->laneStatus) {
      ros_tc_part.lane_status_present = true;
      ros_tc_part.lane_status.lane_status =  *asn_tc_part->ext1->laneStatus;
    }

    if (asn_tc_part->ext1->vehicleCharacteristics) {
      ros_tc_part.vehicle_characteristics_present = false;
      // not implemented
    }
  }

  return ros_tc_part;
}

v2x_msgs::msg::Zid IVIMUtils::GetROSZoneId(Zid_t* asn_zone_id) {
  v2x_msgs::msg::Zid ros_zone_id;
  ros_zone_id.zid = *asn_zone_id;
  return ros_zone_id;
}

v2x_msgs::msg::LanePosition IVIMUtils::GetROSLanePosition(LanePosition_t* asn_lane_position) {
  v2x_msgs::msg::LanePosition ros_lane_position;
  ros_lane_position.lane_position = *asn_lane_position;
  return ros_lane_position;
}

v2x_msgs::msg::Text IVIMUtils::GetROSText(Text_t* asn_text) {

  v2x_msgs::msg::Text ros_text;

  if (asn_text->layoutComponentId) {
    ros_text.layout_component_id_present = true;
    ros_text.layout_component_id = *asn_text->layoutComponentId;
  }

//  TODO: ros_text.language - issue : how do i translate the asn BIT_STRING ?

//  TODO: ros_text.text_content - issue : how do i translate the asn UTF8String ?

  return ros_text;
}


void* IVIMUtils::AllocateClearedMemory(size_t bytes) {
  void* allocated_memory = malloc(bytes);
  memset(allocated_memory, 0, bytes);
  return allocated_memory;
}

