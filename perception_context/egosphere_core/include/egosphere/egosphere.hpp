#ifndef _EGOSPHERE_HPP_
#define _EGOSPHERE_HPP_

#include <map>
#include <boost/signals.hpp>

#include "information.hpp"
#include "associations.hpp"

// TODO more elaborate requests instead of infosBy...() ?
//      (see rails activerecord)

namespace egosphere {

template<typename Info>
class Egosphere : boost::noncopyable
{
 public:

  typedef typename Info::Time     Time;
  typedef typename Info::Duration Duration;
  typedef typename Info::Params   Params;
  typedef typename Info::Ptr      InfoPtr;
  typedef typename Info::Itr      InfoItr;

  //Egosphere();

  virtual void deleteExpiredInfo(const InfoPtr & info)
  {
    if(info->expired()) {
      infos_.erase(info);
      associations_.erase(info);
    }
  }

  InfoPtr newInfo(
      const std::string & tags,
      const Duration & cache_duration,
      const Params & params,
      const Time & t
  ){
    InfoPtr ptr = Info::create(cache_duration)
        ->set(Tags(tags))
        ->set(params, t);
    infos_.insert(ptr);
    ptr->onUpdate(boost::bind(&Egosphere::deleteExpiredInfo, this, _1));
    new_info_notifier_(ptr);
    return ptr;
  }

  InfoPtr newInfo(
      const std::string & tags,
      const Duration & cache_duration,
      const Params & params
  ){
    return newInfo(tags, cache_duration, params, Time());
  }

  void onNewInfo(boost::function<void (const InfoPtr &)> f)
  {
    new_info_notifier_.connect(f);
  }

  void associate(const InfoPtr & i1, const InfoPtr & i2)
  {
    associations_.associate(i1, i2);
  }

  void dissociate(const InfoPtr & i1, const InfoPtr & i2)
  {
    associations_.dissociate(i1, i2);
  }

  bool associated(const InfoPtr & i1, const InfoPtr & i2) const
  {
    return associations_.associated(i1, i2);
  }

  const Associations<InfoPtr> & associations() const
  {
    return associations_;
  }

  InfoItr infoItr() const
  {
    return InfoItr(infos_.begin(), infos_.end());
  }

  InfoItr infoByTag(const std::string& tag) const
  {
    InfoItr i(
        boost::make_filter_iterator(
            boost::bind(hasTag, _1, tag),
            infos_.begin(), infos_.end()
            ),
        boost::make_filter_iterator(
            boost::bind(hasTag, _1, tag),
            infos_.end(), infos_.end()
            )
        );
    return i;
  }

  InfoItr infoByAssociation(
    const InfoPtr & with,
    const std::string & tag = ""
  ){
    if(tag.empty()) {
      return InfoItr(associations_.begin(with));
    } else {
      return InfoItr(
        associations_.begin(with, boost::bind(hasTag, _1, tag))
      );
    }
  }

 private:

  /* Used by filter iterators to filter by tags */
  static bool hasTag(const InfoPtr & info, const std::string & tag)
  {
    return info->is(tag);
  }

  std::set<InfoPtr> infos_;
  Associations<InfoPtr> associations_;
  boost::signal<void (const InfoPtr &)> new_info_notifier_;
  boost::signal<void (const InfoPtr &, const InfoPtr &)> association_notifier_;
};

}

#endif
