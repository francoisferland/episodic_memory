/*
 * test_init_model.cpp
 *
 *  Created on: 2013-01-14
 *      Author: frank
 */
#ifndef TEST_INIT_MODEL
#define TEST_INIT_MODEL

#include <gtest/gtest.h>
#include "episodicMemoryCore.h"
#include <config.h>

using namespace EM_CORE;

class InitEpisodicMemory : public testing::Test
{
protected:
	void SetUp() {

		const char* databasePath = EPISODIC_MEMORY_TEST_DATABASE_PATH;
		e = EpisodicMemoryCorePtr(new EpisodicMemoryCore(&databasePath));
		e->initialize() ;
		e->clearEpisodicMemory();
	}
	EpisodicMemoryCorePtr e;
};

TEST_F (InitEpisodicMemory, AddChannel)
{
	EXPECT_TRUE(e->addInputChannel("Persons" , 0.92 ));
}

TEST_F (InitEpisodicMemory, AddTwoIdenticalChannel)
{
	EXPECT_TRUE(e->addInputChannel("Persons" , 0.92 ));
	EXPECT_FALSE(e->addInputChannel("Persons" , 0.7 ));
}

TEST_F (InitEpisodicMemory, AddTwoDifferentChannel)
{
	EXPECT_TRUE(e->addInputChannel("Persons" , 0.92 ));
	EXPECT_TRUE(e->addInputChannel("Object" , 0.7 ));
}

TEST_F (InitEpisodicMemory, AddNewInputInChannel)
{
	EXPECT_TRUE(e->addInputChannel("Persons" , 0.92 ));
	CategoryARTptr category = e->addInput("TestInput","Persons");
	ASSERT_TRUE(category);
	EXPECT_EQ(1, category->getUid());
	//the next index' input must be two because each input has its complement
	category.reset();
	category = e->addInput("TestInput2","Persons");
	ASSERT_TRUE(category);
	EXPECT_EQ(3, category->getUid());
}

TEST_F (InitEpisodicMemory, AddTwoIdenticalInputInChannel )
{
	EXPECT_TRUE(e->addInputChannel("Persons" , 0.92 ));
	CategoryARTptr category = e->addInput("TestInput","Persons");
	ASSERT_TRUE(category);
	EXPECT_EQ(1, category->getUid());
	category.reset();
	category = e->addInput("TestInput","Persons");
	ASSERT_FALSE(category);

}

TEST_F (InitEpisodicMemory, AddInputInMissingChannel)
{
	EXPECT_TRUE(e->addInputChannel("Persons" , 0.92 ));
	CategoryARTptr category = e->addInput("TestInput","Object");
	ASSERT_FALSE(category);
}

TEST_F (InitEpisodicMemory, LoadChannel)
{
	//create a channel
	EXPECT_TRUE(e->addInputChannel("Persons" , 0.92 ));
	//load it from database
	ChannelDaoPtr channelDao = DaoFactory::getChannelDao(e->getDbPtr());
	std::map<int,ChannelARTptr> channelVect = channelDao->findAll(INPUT_LAYER);
	ASSERT_EQ(1,channelVect.size());
	//first index start at 1
	ASSERT_STREQ( "Persons", channelVect[1]->getDescription().c_str() );
	ASSERT_NEAR(0.92, channelVect[1]->getRelevance(), 0.001);

	channelDao.reset();
}

TEST_F (InitEpisodicMemory, LoadInput)
{
	//load all input
	InputDaoPtr inputDao = DaoFactory::getInputDao(e->getDbPtr());
	std::map<int,CategoryARTptr> inputVect = inputDao->findAll();

	ASSERT_EQ(0, inputVect.size());

	boost::shared_ptr<ChannelART> channelPerson = e->addInputChannel("Persons" , 0.92 );
	boost::shared_ptr<ChannelART> channelObject = e->addInputChannel("Object" , 0.90 );
	EXPECT_TRUE(channelPerson);
	EXPECT_TRUE(channelObject);

	//create an input
	CategoryARTptr category = e->addInput("Personne A","Persons");
	ASSERT_TRUE(category);
	EXPECT_EQ(1, category->getUid());

	//add another input in the same channel
	category.reset();
	category = e->addInput("Personne B","Persons");
	ASSERT_TRUE(category);
	EXPECT_EQ(3, category->getUid());
	//add another input in a different channel
	category.reset();
	category = e->addInput("Object 1", "Object");
	ASSERT_TRUE(category);
	EXPECT_EQ(5, category->getUid());

	//load all input
	inputVect = inputDao->findAllByChannel(channelPerson);

	ASSERT_EQ(4, inputVect.size());
	InputObjPtr inputObj = boost::dynamic_pointer_cast<InputObj>(inputVect[1]);
	EXPECT_STREQ("Personne A",inputObj->getDescription().c_str());
	EXPECT_STREQ("Persons",inputObj->getChannel()->getDescription().c_str());
	EXPECT_EQ(0,inputObj->getIndexCategory());
	EXPECT_EQ(1,inputObj->getUid());

	inputObj = boost::dynamic_pointer_cast<InputObj>(inputVect[3]);
	EXPECT_STREQ("Personne B",inputObj->getDescription().c_str());
	EXPECT_STREQ("Persons",inputObj->getChannel()->getDescription().c_str());
	EXPECT_EQ(3,inputObj->getUid());
	EXPECT_EQ(2,inputObj->getIndexCategory());

	inputVect = inputDao->findAllByChannel(channelObject);
	ASSERT_EQ(2, inputVect.size());
	inputObj = boost::dynamic_pointer_cast<InputObj>(inputVect[5]);
	EXPECT_STREQ("Object 1",inputObj->getDescription().c_str());
	EXPECT_STREQ("Object",inputObj->getChannel()->getDescription().c_str());
	EXPECT_EQ(4,inputObj->getIndexCategory());
	EXPECT_EQ(5,inputObj->getUid());

	inputDao.reset();
}

/*
 * Create an input vector that matches existing channel and categories
 */
TEST_F (InitEpisodicMemory, ActivateAllInputs)
{
	//initialize a network
	EXPECT_TRUE(e->addInputChannel("Persons" , 0.92 ));
	EXPECT_TRUE(e->addInputChannel("Object" , 0.7 ));
	CategoryARTptr category = e->addInput("Person A","Persons");
	ASSERT_TRUE(category);
	EXPECT_EQ(1, category->getUid());
	category = e->addInput("Person B","Persons");
	ASSERT_TRUE(category);
	EXPECT_EQ(3, category->getUid());
	category = e->addInput("Object 1","Object");
	ASSERT_TRUE(category);
	EXPECT_EQ(5, category->getUid());

	//create a input vector that matches the current network
	std::map<std::string,InputROSmsgPtr> testMap;

	testMap["Person A"] = InputROSmsgPtr(new InputROSmsg("Person A","Persons",1,10));
	testMap["Person B"] = InputROSmsgPtr(new InputROSmsg("Person B","Persons",1,10));
	testMap["Object 1"] = InputROSmsgPtr(new InputROSmsg("Object 1","Object",0.7,10));

	ASSERT_TRUE(e->activateInputs(testMap));

}

/*
 * Create an input vector that matches existing channel and categories. Not all categories are activated
 */
TEST_F (InitEpisodicMemory, ActivatePartialInputs)
{
	//initialize a network
	EXPECT_TRUE(e->addInputChannel("Persons" , 0.92 ));
	EXPECT_TRUE(e->addInputChannel("Object" , 0.7 ));
	CategoryARTptr category = e->addInput("Person A","Persons");
	ASSERT_TRUE(category);
	EXPECT_EQ(1, category->getUid());
	category = e->addInput("Person B","Persons");
	ASSERT_TRUE(category);
	EXPECT_EQ(3, category->getUid());
	category = e->addInput("Object 1","Object");
	ASSERT_TRUE(category);
	EXPECT_EQ(5, category->getUid());

	std::map<std::string,InputROSmsgPtr> testMap;

	testMap["Person B"] = InputROSmsgPtr(new InputROSmsg("Person B","Persons",1,10));

	ASSERT_TRUE(e->activateInputs(testMap));
}

/*
 * Create a vector with input that don't exist in the network
 * the network should create the new inputs
 */
TEST_F (InitEpisodicMemory, ActivateNewInputs)
{
	//initialize a network
	EXPECT_TRUE(e->addInputChannel("Persons" , 0.92 ));
	EXPECT_TRUE(e->addInputChannel("Object" , 0.7 ));
	CategoryARTptr category = e->addInput("Person A","Persons");
	ASSERT_TRUE(category);
	EXPECT_EQ(1, category->getUid());
	category = e->addInput("Person B","Persons");
	ASSERT_TRUE(category);
	EXPECT_EQ(3, category->getUid());
	category = e->addInput("Object 1","Object");
	ASSERT_TRUE(category);
	EXPECT_EQ(5, category->getUid());

	std::map<std::string,InputROSmsgPtr> testMap;

	testMap["Person B"] = InputROSmsgPtr(new InputROSmsg("Person B","Persons",1,10));
	testMap["Person C"] = InputROSmsgPtr(new InputROSmsg("Person C","Persons",1,10));

	ASSERT_TRUE(e->activateInputs(testMap));
}

/*
 * Create a vector with input and channel that don't exist in the network
 * the network should create the new inputs and channel
 */
TEST_F (InitEpisodicMemory, ActivateNewInputInNewChannel)
{
	//initialize a network
	EXPECT_TRUE(e->addInputChannel("Persons" , 0.92 ));
	EXPECT_TRUE(e->addInputChannel("Object" , 0.7 ));
	CategoryARTptr category = e->addInput("Person A","Persons");
	ASSERT_TRUE(category);
	EXPECT_EQ(1, category->getUid());
	category = e->addInput("Person B","Persons");
	ASSERT_TRUE(category);
	EXPECT_EQ(3, category->getUid());
	category = e->addInput("Object 1","Object");
	ASSERT_TRUE(category);
	EXPECT_EQ(5, category->getUid());

	std::map<std::string,InputROSmsgPtr> testMap;

	testMap["Person A"] = InputROSmsgPtr(new InputROSmsg("Person A","Persons",1,10));
	testMap["Goto"] = InputROSmsgPtr(new InputROSmsg("Goto","Motivation",0.7,10));

	ASSERT_TRUE(e->activateInputs(testMap));
}

TEST_F (InitEpisodicMemory, ClearMemory)
{

}

#endif
