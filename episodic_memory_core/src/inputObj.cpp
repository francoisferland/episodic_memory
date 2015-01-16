#include "inputObj.h"

InputObj::InputObj() : CategoryART()
{
	m_description = "";
}

InputObj::InputObj(int uid, int indexInput, std::string desc) : CategoryART(uid)
{
	setIndexCategory(indexInput);
	m_description = desc;
}

