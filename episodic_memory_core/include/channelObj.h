#ifndef CHANNELOBJ_H_
#define CHANNELOBJ_H_

#include <boost/thread.hpp>

class ChannelObj;
typedef boost::shared_ptr<ChannelObj> ChannelObjPtr;

/**
 * This class represent an object of the Channel Table in the database
 */
class ChannelObj {
public:
	/**
	 * default Constructor
	 */
	ChannelObj();
	/**
	 * Constructor
	 * @param number: id number in the layer
	 * @param desc: a textual description of the channel
	 */
	ChannelObj(int uid, int indexChannel, std::string desc, float relevance, int layerId) ;
	/**
	 * Destructor
	 */
	virtual ~ChannelObj(){};

	std::string getDescription() const {
		return m_description;
	}
	void setDescription(std::string description) {
		this->m_description = description;
	}
	int getIndexChannel() const {
		return m_indexChannel;
	}
	void setIndexChannel(int idNumber) {
		this->m_indexChannel = idNumber;
	}
	float getRelevance() const {
		return m_relevance;
	}
	void setRelevance(float relevance) {
		this->m_relevance = relevance;
	}
	int getLayerId() const {
		return m_layerId;
	}
	void setLayerId(int layerId) {
		this->m_layerId = layerId;
	}
	int getUid() const {
		return m_uid;
	}
	void setUid(int uid) {
		m_uid = uid;
	}

private:

	int m_indexChannel;
	std::string m_description;
    float m_relevance;
    int m_layerId;
    int m_uid;

};

#endif /* CHANNELOBJ_H_ */
