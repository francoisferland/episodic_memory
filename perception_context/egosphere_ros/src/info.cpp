#include "egosphere_ros/egosphere.hpp"
#include "egosphere_ros/info.hpp"

#include <iomanip>

using namespace egosphere_ros;

double egosphere_ros::operator/ (const ros::Duration & d1, const ros::Duration & d2)
{
  return d1.toSec() / d2.toSec();
}

double egosphere_ros::interpolate(double v0, double v1, double ratio)
{
  return v0 + ratio * (v1 - v0);
}

std::string Info::debug() const
{
  std::ostringstream str;

  str << "  tags: "
      << (std::string) tags() << "\n";

  ros::Time now = ros::Time::now();

  // Contains a parameters map associated with their timestamp
  typedef Info::map TimeParamsMap;
  // Contains key/value for each parameters in the map
  typedef std::map<std::string, egosphere::Parameter> ParamsMap;

  if (params().size() > 0) {

    str << "\n";

    // TODO rename Info::Params to Info::Samples ?
    str << "  " << params().size() << " samples\n";

    // Indexes each parameter by name, counting them to
    // find their update frequency
    //
    // TODO display the bandwidth used by each parameter
    // (challenging if we include the whole messages size)

    std::map<std::string, unsigned> param_stats;

    for (
        TimeParamsMap::const_iterator it = params().begin();
        it != params().end();
        it++
    ) {
      const ParamsMap & params = it->second.first.values();
      for (
        ParamsMap::const_iterator p = params.begin();
        p != params.end();
        p++
      ){
        const std::string name = p->first;
        if (param_stats.find(name) == param_stats.end()) {
          param_stats[name] = 0;
        }
        param_stats[name]++;
      }
    }

    for(
      std::map<std::string, unsigned>::iterator it = param_stats.begin();
      it != param_stats.end();
      it++
    ) {
      str << "    " << it->first << " (" << it->second << ")\n";
    }

    // Displays histroy samples as a table

    str << "\n     ";
    for(
      std::map<std::string, unsigned>::iterator it = param_stats.begin();
      it != param_stats.end();
      it++
    ) {
      str << " | " << it->first;
    }
    str << " |\n";

    unsigned count = 0;
    for (
        TimeParamsMap::const_iterator it = params().begin();
        it != params().end();
        it++
    ) {
      const ros::Time & t = it->first;
      const ParamsMap & pmap = it->second.first.values();

      char tmp[50];
      sprintf(tmp, "%5.2f", (now - t).toSec());
      str << tmp;

      for(
          std::map<std::string, unsigned>::iterator p = param_stats.begin();
          p != param_stats.end();
          p++
      ) {
        std::string field(p->first.size(), ' ');
        char present = ' ';
        if (pmap.find(p->first) != pmap.end()) {
          present = 'x';
        }
        field[field.size()/2] = present;
        str << " | " << field;
      }
      str << " |\n";

      // Advances the iterator to go directly to the 10 last
      // samples
      count++;
      if (count == 10) {
        str << " ...\n";
        std::advance(it, params().size() - count - 10);
      }
    }
  }
  return str.str();
}
