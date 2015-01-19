#ifndef _EGOSPHERE_TEST_COMMON_HPP_
#define _EGOSPHERE_TEST_COMMON_HPP_

#include <gtest/gtest.h>
#include <string>
#include "egosphere/parameter.hpp"
#include "egosphere/information.hpp"
#include "egosphere/egosphere.hpp"

using namespace egosphere;


// Defines a standard parameter, using a ParametersMap
// with strings as keys

class Params : public ParametersMap<Params, std::string> {};

// Information being a set of timestamped parameters, using a
// double to encode time

class Info : public Information<Info, Params, double, double> {};

typedef Egosphere<Info> Egos;

class Length
{

public :

  Length(double m)
  {
    m_ = m;
  }

  double val()
  {
    return m_;
  }

  operator double() const { return m_; }

private:

  double m_;
};


class Angle
{

public :

  Angle(double radians)
  {
    rad_ = normalize(radians);
  }

  // There must be a more effective way to do this...
  static double normalize(double angle)
  {
    while (angle <= -M_PI) angle += 2*M_PI;
    while (angle >   M_PI) angle -= 2*M_PI;
    return angle;
  }

  double val() const
  {
    return rad_;
  }

  operator double() const { return rad_; }

private:

  // TODO add uncertainty
  double rad_;
};

double interpolate(double v0, double v1, double ratio)
{
  return v0 + ratio * (v1 - v0);
}

Angle operator-(const Angle & a1, const Angle & a2)
{
  return Angle(a1.val() - a2.val());
}

Angle interpolate(const Angle & a0, const Angle & a1, double ratio)
{
  return Angle(interpolate(a0.val(), a1.val(), ratio));
}

#endif
