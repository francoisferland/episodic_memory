#include "egosphere_ros/egosphere.hpp"
#include <cassert>
using namespace egosphere_ros;

Egosphere::Egosphere(ros::NodeHandle & nh, const std::string & topic) :
    nh_(nh),
    topic_(topic),
    info_uid_(1),
    default_info_cache_(10.0)
{
  // Generates an unique id used to assign a unique id to new informations
  //
  // name_hash is used to make the distinction between different nodes
  // nsec is used to make the distinction between nodelets or multiple
  // egospheres within the same node.
  //
  // TODO check if 32 bit is ok
  // maybe using a protocol to negociate each id would be better

  uint16_t name_hash = boost::hash<std::string>()(ros::this_node::getName());
  uint16_t time_nsec = ros::Time::now().toNSec();
  uid_ = time_nsec << 16 | name_hash;

  // TODO what to use for the queue?
  pub_ = nh.advertise<MsgGenericChange>(topic, 1000);
  sub_ = nh.subscribe(topic, 1000, &Egosphere::msgReceived, this);
}

void Egosphere::msgReceived(const MsgGenericChange::ConstPtr& msg)
{
  if(msg->src == uid_) { return; }
  // ROS_INFO_STREAM(uid_ << ": Msg received");

  try {
    if(msg->action == MsgGenericChange::ADD_INFORMATION)
    {
      assert(msg->infos.size() == 1);
      assert(msg->params.size() == 1);
      uint64_t id = msg->infos[0].id;
      const std::string & tags = msg->infos[0].tags;
      // ROS_INFO_STREAM(uid_ << ": ADD_INFO "  << id << " " << tags);

      assert(msg->params.size() == 1);
      Params params;
      params.fromMsg(msg->params[0]);

      Info::Ptr info = newInfo(
        tags, params, msg->params[0].stamp, false);
      idToInfo(id, info);
    }
    else if (msg->action == MsgGenericChange::SET_PARAMETERS)
    {
      // ROS_INFO_STREAM(uid_ << ": SET_PARAMS");
      // Identifies the information where the param have been updated
      assert(msg->infos.size() == 1);
      const Info::Ptr & info = idToInfo(msg->infos[0].id);
      // Build the parameters
      assert(msg->params.size() == 1);
      Params params;
      params.published(true) // Do not advertise others of this change
            .fromMsg(msg->params[0]);
      // Update the parameters in this information
      info->set(params, msg->params[0].stamp);
    } else if (msg->action == MsgGenericChange::REMOVE_INFORMATION) {
      assert(msg->infos.size() == 1);
      const Info::Ptr & info = idToInfo(msg->infos[0].id);
      info->expire(true); // true = already published
    } else {
      ROS_ERROR_STREAM("Unknown generic change type.\n");
    }
  } catch(UnknownInformation e) {
    // ie. changing params of an unexisting (locally) information
    // TODO is it worth a message like resend information ## details?
    // ROS_WARN("Message received concerning not previously seen information");
  }
}

Info::Ptr Egosphere::newInfo(
    const std::string & tags, const Params & params, const ros::Time & t,
    bool publish
    ){
  Info::Ptr info = super::newInfo(tags, default_info_cache_, params, t);
  info->onUpdate(boost::bind(&Egosphere::publishInfoUpdate, this, _1));
  // ROS_INFO_STREAM(uid_ << ": Adding info");
  if (publish) {
    publishNewInfo(info);
  }
  return info;
}

Info::Ptr Egosphere::newInfo(
    const std::string & tags, const Params & params, const ros::Time & t
    ){
  return newInfo(tags, params, t, true);
}

Info::Ptr Egosphere::newInfo(
    const std::string & tags, const Params & params
    ){
  return newInfo(tags, params, ros::Time::now(), true);
}

void Egosphere::associate(
    const Info::Ptr & i1, const Info::Ptr & i2, bool published
    ){
  super::associate(i1, i2);
}

void Egosphere::dissociate(
    const Info::Ptr & i1, const Info::Ptr & i2, bool published
    ){
  super::dissociate(i1, i2);
}

uint32_t Egosphere::uid() const
{
  return uid_;
}

uint64_t Egosphere::info_uid()
{
  if(info_uid_ == 0) {
    ROS_ERROR("Information uniqid overflow.");
  }
  return (uint64_t)uid_ << 32 | info_uid_++;
}

void Egosphere::publishNewInfo(const Info::Ptr & info)
{
  MsgGenericChange msg;

  // Meta data
  msg.src = uid_;
  msg.action = MsgGenericChange::ADD_INFORMATION;

  // Info
  MsgInformation msg_info;
  uint64_t id = info_uid();
  idToInfo(id, info);
  msg_info.id = id;
  msg_info.tags = static_cast<std::string>(info->tags());
  msg.infos.push_back(msg_info);

  BOOST_FOREACH(const Info::map::value_type & sp, info->params())
  {
    MsgParameters msg_params;
    msg_params.stamp = sp.first;
    sp.second.first.toMsg(msg_params);
    msg.params.push_back(msg_params);
  }

  //ROS_INFO_STREAM(uid_ << ": Publishing new info "
  //    << msg_info.id << " "
  //    << msg_info.tags;
  //);

  pub_.publish(msg);
}

void Egosphere::publishInfoUpdate(const Info::Ptr & info)
{
  MsgGenericChange msg;
  msg.src = uid_;

  if(info->expired())
  {
    uint64_t id = infoToId(info);
    // releases uneeded references to the expired info, so it can
    // be deleted
    id_to_info_.right.erase(info);

    if(info->expirationPublished()) {
      return;
    }
    MsgInformation msg_info;
    msg_info.id = id;
    msg.infos.push_back(msg_info);
    msg.action = MsgGenericChange::REMOVE_INFORMATION;
    // ROS_INFO_STREAM(uid_ << ": Publishing info remove " << msg_info.id);
  } else {
    // Info Id
    MsgInformation msg_info;
    msg_info.id = infoToId(info);
    msg.infos.push_back(msg_info);
    BOOST_FOREACH(const Info::map::value_type & sp, info->params())
    {
      const ros::Time & stamp = sp.first;
      const Params & params = sp.second.first;
      const Info::UpId & upId = sp.second.second;
      if(upId == info->lastUpId())
      {
        // Stop if params of this upid have already been published
        if (params.published()) return;

        MsgParameters msg_params;
        msg_params.stamp = stamp;
        params.toMsg(msg_params);
        msg.params.push_back(msg_params);
      }
    }
    msg.action = MsgGenericChange::SET_PARAMETERS;
  }

  // ROS_INFO_STREAM(uid_ << ": Publishing info update " << msg_info.id);

  pub_.publish(msg);
}

void Egosphere::idToInfo(uint64_t id, const Info::Ptr & info)
{
  // An id should only belong to one info
  assert(id_to_info_.left.find(id) == id_to_info_.left.end());
  id_to_info_.insert(IdToInfo::value_type(id, info));
}

Info::Ptr Egosphere::idToInfo(uint64_t id)
{
  if (id_to_info_.left.find(id) == id_to_info_.left.end())
  {
    throw UnknownInformation();
  }
  return id_to_info_.left.find(id)->second;
}

uint64_t  Egosphere::infoToId(const Info::Ptr & info)
{
  if(id_to_info_.right.find(info) == id_to_info_.right.end())
  {
    throw UnknownInformation();
  }
  return id_to_info_.right.find(info)->second;
}
