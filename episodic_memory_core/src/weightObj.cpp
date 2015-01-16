#include "weightObj.h"

bool WeightObj::isConnected(CategoryARTptr category)
{
	bool isConnected = false;

	if(this->getUpCategoryPtr() == category || this->getDownCategoryPtr() == category)
		isConnected = true;

	return isConnected;
}
