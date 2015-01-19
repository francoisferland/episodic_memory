#ifndef _EGOSPHERE_TAGS_HPP_
#define _EGOSPHERE_TAGS_HPP_

#include <list>
#include <string>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

namespace egosphere {

/*
 * TODO ensure uniqueness ?
 * TODO check if splice flushes the parameter
 */

class Tags
{
public:

	Tags() {}

	Tags(const std::string & s)
	{
		splitToList(s, tags_);
	}

	/*
	 * Inserts all the tags contained in a string to the end of
	 * a given list.
	 */
	static void splitToList(
		const std::string & s,
		std::list<std::string> & l
	) {
		std::list<std::string> tags;
		boost::split(tags, s, boost::is_any_of(","));
		BOOST_FOREACH(std::string& tag, tags) {
			boost::trim(tag);
		}
		l.splice(l.end(), tags);
	}

	operator std::string() const
	{
		return boost::algorithm::join(tags_, ",");
	}

	bool contains(const std::string& tag)
	{
		return tags_.end() != std::find(
			tags_.begin(),
			tags_.end(),
			tag
		);
	}

	void splice(Tags & tags)
	{
		tags_.splice(tags_.end(), tags.tags_);
	}

	unsigned int size() const
	{
		return tags_.size();
	}

private:

	std::list<std::string> tags_;
};


}

#endif
