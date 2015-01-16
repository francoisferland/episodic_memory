#ifndef _GUI_MSG_OBJECT_
#define _GUI_MSG_OBJECT_

#include <boost/thread.hpp>
#include <QtGui/QMainWindow>

class GuiActivation;
typedef boost::shared_ptr<GuiActivation> GuiActivationPtr;

class GuiActivation
{
public:
	GuiActivation(int index, int idChannel, float value, int idLayer)
	{
		index_ = index;
		idChannel_ = idChannel;
		value_ = value;
		idLayer_ = idLayer;
	}

	int getIdChannel() const {
		return idChannel_;
	}

	void setIdChannel(int idChannel) {
		idChannel_ = idChannel;
	}

	int getIdLayer() const {
		return idLayer_;
	}

	void setIdLayer(int idLayer) {
		idLayer_ = idLayer;
	}

	int getIndex() const {
		return index_;
	}

	void setIndex(int index) {
		index_ = index;
	}

	float getValue() const {
		return value_;
	}

	void setValue(float value) {
		value_ = value;
	}

private:
	int index_;
	int idChannel_;
	float value_;
	int idLayer_;
};

class GuiAnticipatedEvent;
typedef boost::shared_ptr<GuiAnticipatedEvent> GuiAnticipatedEventPtr;

class GuiAnticipatedEvent
{
public:
	GuiAnticipatedEvent(int index, float newVigilance, float newLearningRate, bool anticipated)
	{
		index_ = index;
		newVigilance_ = newVigilance;
		newLearningRate_ = newLearningRate;
		anticipated_ = anticipated;
	}

	int getIndex() const {
		return index_;
	}

	void setIndex(int index) {
		index_ = index;
	}

	float getNewVigilance() const {
		return newVigilance_;
	}

	void setNewVigilance(float newVigilance) {
		newVigilance_ = newVigilance;
	}

	bool isAnticipated() const {
		return anticipated_;
	}

	void setAnticipated(bool anticipated) {
		anticipated_ = anticipated;
	}

	float getNewLearningRate() const {
		return newLearningRate_;
	}

	void setNewLearningRate(float newLearningRate) {
		newLearningRate_ = newLearningRate;
	}

private:
	int index_;
	float newVigilance_;
	float newLearningRate_;
	bool anticipated_;

};

class GuiCategory;
typedef boost::shared_ptr<GuiCategory> GuiCategoryPtr;

class GuiCategory
{
public:

	GuiCategory(int index, int channelId, QString description, float vigilance, float learningRate, int layerId)
	{
		index_ = index;
		channelId_ = channelId;
		description_ = description;
		layerId_ = layerId;
		vigilance_ = vigilance;
		learningRate_ = learningRate;
	}

	int getChannelId() const {
		return channelId_;
	}

	void setChannelId(int channelId) {
		channelId_ = channelId;
	}

	const QString& getDescription() const {
		return description_;
	}

	void setDescription(const QString& description) {
		description_ = description;
	}

	int getIndex() const {
		return index_;
	}

	void setIndex(int index) {
		index_ = index;
	}

	int getLayerId() const {
		return layerId_;
	}

	void setLayerId(int layerId) {
		layerId_ = layerId;
	}

	float getLearningRate() const {
		return learningRate_;
	}

	void setLearningRate(float learningRate) {
		learningRate_ = learningRate;
	}

	float getVigilance() const {
		return vigilance_;
	}

	void setVigilance(float vigilance) {
		vigilance_ = vigilance;
	}

private:
	int index_;
	int channelId_;
	QString description_;
	int layerId_;
	float vigilance_;
	float learningRate_;
};

class GuiChannel;
typedef boost::shared_ptr<GuiChannel> GuiChannelPtr;

class GuiChannel
{
public:
	GuiChannel(int channelId, QString description, double relevance, int layer)
	{
		channelId_ = channelId;
		description_ = description;
		relevance_ = relevance;
		layer_ = layer;
	}

	int getChannelId() const {
		return channelId_;
	}

	void setChannelId(int channelId) {
		channelId_ = channelId;
	}

	const QString& getDescription() const {
		return description_;
	}

	void setDescription(const QString& description) {
		description_ = description;
	}

	int getLayer() const {
		return layer_;
	}

	void setLayer(int layer) {
		layer_ = layer;
	}

	double getRelevance() const {
		return relevance_;
	}

	void setRelevance(double relevance) {
		relevance_ = relevance;
	}

private:
	int channelId_;
	QString description_;
	double relevance_;
	int layer_;
};

class GuiWeight;
typedef boost::shared_ptr<GuiWeight> GuiWeightPtr;

class GuiWeight
{
public:
	GuiWeight(int id, int iTopCat,int iBotCat,int layer,float val)
	{
		id_ = id;
		indexTopCategory_ = iTopCat;
		indexBottomLayer_ = iBotCat;
		idLayer_ = layer;
		value_ = val;
	}

	int getId() const {
		return id_;
	}

	void setId(int id) {
		id_ = id;
	}

	int getIdLayer() const {
		return idLayer_;
	}

	void setIdLayer(int idLayer) {
		idLayer_ = idLayer;
	}

	int getIndexBottomLayer() const {
		return indexBottomLayer_;
	}

	void setIndexBottomLayer(int indexBottomLayer) {
		indexBottomLayer_ = indexBottomLayer;
	}

	int getIndexTopCategory() const {
		return indexTopCategory_;
	}

	void setIndexTopCategory(int indexTopCategory) {
		indexTopCategory_ = indexTopCategory;
	}

	float getValue() const {
		return value_;
	}

	void setValue(float value) {
		value_ = value;
	}

private:

	int id_;
	int indexTopCategory_;
	int indexBottomLayer_;
	int idLayer_;
	float value_;
};

#endif
