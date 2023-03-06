
#ifndef V2X_GW_IVIMHANDLER_H
#define V2X_GW_IVIMHANDLER_H


#include "V2XMHandler.h"

#include <nav_msgs/msg/odometry.hpp>

#include <v2x_msgs/msg/ivim_list.hpp>

extern "C" {
#include "vcits/ivim/IVIM.h"
#include "vcits/ivim/IviContainer.h"
#include "vcits/ivim/GlcPart.h"
#include "vcits/ivim/Zone.h"
#include "vcits/ivim/PolygonalLine.h"
#include "vcits/ivim/DeltaPosition.h"
#include "vcits/ivim/TcPart.h"
#include "vcits/ivim/Zid.h"
#include "vcits/ivim/LanePosition.h"
#include "vcits/ivim/Text.h"
}


class IVIMHandler : public V2XMHandler {
public:
  /// Initializes the IVIM handler
  IVIMHandler(rclcpp::Node *gateway_node);

  /// Destroys IVIM handler
  ~IVIMHandler();

  /// Get the most recent IVIM
  /// IVIM is produced in the background with ros subscriber magic; this method encodes it before returning.
  /// \return most recent IVIM
  std::queue<std::pair<void *, size_t>> GetMessages() override;

  /// Process the received IVIM(s)
  /// IVIM(s) will be processed and ros published
  /// \param messages most recent set of received IVIM(s)
  void PutMessages(std::queue<std::pair<void *, size_t>> messages) override;

protected:
  /// Read the configuration for the IVIM
  void ReadConfig() override;

private:
  // necessary handler processing constants
  bool IVIM_ACTIVE;
  long IVIM_DEBUG_MSG_THROTTLE_MS;

  bool new_data_received_;

  // IVIM attributes
  std::vector <IVIM_t*> ivim_list_;

  // Published topics
  rclcpp::Publisher<v2x_msgs::msg::IVIMList>::SharedPtr ivim_pub_;

  // Subsribed topics
  rclcpp::Subscription<v2x_msgs::msg::IVIMList>::SharedPtr ros_ivim_sub_;

  // List maniputaliton functions
  void addIVIM();
  void removeIVIM();

  // Callbacks
  void RosIVIMCallback(const v2x_msgs::msg::IVIMList::SharedPtr ros_ivim_list);

  // IVIM generation
  void InitIVIM();
  void fillIVIM(v2x_msgs::msg::IVIM ros_ivim, IVIM_t* ivim);


  v2x_msgs::msg::IVIM GetROSIVIM(std::pair<void *, size_t> message);
  v2x_msgs::msg::IviContainer GetROSIviContainer(IviContainer_t* asn_ivi_container);
  v2x_msgs::msg::GlcPart GetROSGlcPart(GlcPart_t* asn_glc_part);
  v2x_msgs::msg::Zone GetROSZone(Zone_t* asn_zone);
  v2x_msgs::msg::PolygonalLine GetROSPolygonalLine(PolygonalLine_t* asn_polygonal_line);
  v2x_msgs::msg::DeltaPosition GetROSDeltaPosition(DeltaPosition_t* asn_delta_position);

  v2x_msgs::msg::TcPart GetROSTcPart(TcPart_t* asn_tc_part);
  v2x_msgs::msg::Zid GetROSZoneId(Zid_t* asn_zone_id);
  v2x_msgs::msg::LanePosition GetROSLanePosition(LanePosition_t* asn_lane_position);
  v2x_msgs::msg::Text GetROSText(Text_t* asn_text);

  // debug
  void PrintIVIM();

};

#endif // V2X_GW_IVIMHANDLER_H
