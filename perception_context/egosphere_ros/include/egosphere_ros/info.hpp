#ifndef _EGOSPHERE_ROS_INFO_HPP_
#define _EGOSPHERE_ROS_INFO_HPP_

#include "egosphere_ros/params.hpp"
#include "egosphere/information.hpp"

namespace egosphere_ros {

// Information with our own parameters map, and ros::Time
// to encode the time in the application.
class Info : public egosphere::Information<Info, Params, ros::Time, ros::Duration>
{
private:

  typedef egosphere::Information<Info, Params, ros::Time, ros::Duration> super;

public:

  // Not to have to redefine every overloaded versions of set
  // http://stackoverflow.com/questions/888235/overriding-a-bases-overloaded-function-in-c
  using super::set;

  // Set parameters with no time specified, using now.
  const Ptr & set(const Params & params)
  {
    return set(params, ros::Time::now());
  }

  void expire(bool published = false)
  {
    expiration_published_ = published;
    super::expire();
  }

  bool expirationPublished() const
  {
    return expiration_published_;
  }

  std::string debug() const;

private:

  bool expiration_published_;

};

// Used for interpolation of a parameter between t1 and t2
// (t0->t)/(t0->t1)

double operator/ (const ros::Duration & d1, const ros::Duration & d2);

// Other specialization should be provided by the user for each data type

double interpolate(double v0, double v1, double ratio);

}

#endif
