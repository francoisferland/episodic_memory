#include "channelObj.h"

ChannelObj::ChannelObj()
{
	m_uid = -1;
	m_indexChannel = -1;
	m_description = "";
	m_layerId = -1;
	m_relevance = 0;
}

ChannelObj::ChannelObj(int uid, int indexChannel, std::string desc, float relevance, int layerId)
{
	m_uid = uid;
	m_indexChannel = indexChannel;
	m_description = desc;
	m_layerId = layerId;
	m_relevance = relevance;
}
