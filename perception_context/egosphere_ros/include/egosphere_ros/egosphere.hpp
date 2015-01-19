#ifndef _EGOSPHERE_ROS_HPP_
#define _EGOSPHERE_ROS_HPP_

#include <egosphere/egosphere.hpp>
#include <egosphere/information.hpp>
#include <ros/ros.h>
#include <boost/functional/hash.hpp>
#include <boost/bimap.hpp>

#include "egosphere_ros/MsgGenericChange.h"
#include "egosphere_ros/params.hpp"
#include "egosphere_ros/info.hpp"

namespace egosphere_ros {

class UnknownInformation : std::exception {};

class Egosphere : public egosphere::Egosphere<Info> {

public:

  typedef boost::bimap<uint64_t, Info::Ptr> IdToInfo;

  Egosphere(ros::NodeHandle & nh, const std::string & topic);

  Info::Ptr newInfo(
      const std::string & tags, const Params & params, const ros::Time & t
      );

  Info::Ptr newInfo(
      const std::string & tags, const Params & params
      );

  void associate(
      const Info::Ptr & i1, const Info::Ptr & i2, bool published = false
      );

  void dissociate(
      const Info::Ptr & i1, const Info::Ptr & i2, bool published = false
      );

  uint32_t uid() const;

  uint64_t info_uid();

  // Returns an information given its id
  Info::Ptr idToInfo(uint64_t id);
  // Returns an id given its information
  uint64_t  infoToId(const Info::Ptr & info);

private:

  typedef egosphere::Egosphere<Info> super;

  /*
   * Used internally to make the difference between a new info
   * added by the user, or a new info comming from other egospheres
   */
  Info::Ptr newInfo(
      const std::string & tags, const Params & params, const ros::Time & t,
      bool publish
      );

  // Associates an id with an information
  void idToInfo(uint64_t id, const Info::Ptr & info);

  void msgReceived(const MsgGenericChange::ConstPtr& msg);

  void publishNewInfo(const Info::Ptr & info);

  void publishAssociation(const Info::Ptr & i1, const InfoPtr & i2);

  void publishInfoUpdate(const Info::Ptr & info);

  ros::NodeHandle nh_;
  std::string topic_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  uint32_t uid_;
  uint32_t info_uid_;
  // Keeps track of which id belongs to which info
  IdToInfo id_to_info_;

  ros::Duration default_info_cache_;
};

}

#endif

