#ifndef TEST_RESONANCE
#define TEST_RESONANCE

#include <gtest/gtest.h>
#include "episodicMemoryCore.h"
#include <config.h>

using namespace EM_CORE;

class EpisodicMemoryResonance : public testing::Test
{
protected:
	void SetUp() {

        ros::Time::init();
		const char* databasePath = EPISODIC_MEMORY_TEST_DATABASE_PATH;
		e = EpisodicMemoryCorePtr(new EpisodicMemoryCore(&databasePath));
		e->initialize() ;
		e->clearEpisodicMemory();
	}
	EpisodicMemoryCorePtr e;
};

TEST_F(EpisodicMemoryResonance, resonanceSameInput)
{
	//resonance with one pattern
	std::map<std::string,InputROSmsgPtr> testMap;

	testMap["SpongeBob"] = InputROSmsgPtr(new InputROSmsg("SpongeBob","Persons",1,10));
	testMap["Orange ball"] = InputROSmsgPtr(new InputROSmsg("Orange ball","Object",1,10));
	testMap["Labo"] = InputROSmsgPtr(new InputROSmsg("Labo","Location",1,10));
	testMap["Surprise"] = InputROSmsgPtr(new InputROSmsg("Surprise","Emotion",0.7,10));

	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_FALSE( e->processInput() );

	//verify number of input category (complement coded input)
	EXPECT_EQ( 8 , e->getInputLayer()->getListCategory().size() );
	//verify number of events
	EXPECT_EQ( 1, e->getEventLayer()->getListCategory().size() );

	//resonance with same input pattern
	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_FALSE( e->processInput() );

	//resonance with same input again, considering the learning (should not have change anything)
	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_FALSE( e->processInput() );

	//verify number of events (must be the same)
	EXPECT_EQ( 1, e->getEventLayer()->getListCategory().size() );

}

TEST_F(EpisodicMemoryResonance, resonanceNewInputCategory)
{
	//resonance with one pattern
	std::map<std::string,InputROSmsgPtr> testMap;

	testMap["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));
	testMap["Neutre"] =InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_FALSE( e->processInput() );

	//verify number of input category (complement coded input)
	EXPECT_EQ( 8 , e->getInputLayer()->getListCategory().size() );
	//verify number of events
	EXPECT_EQ( 1, e->getEventLayer()->getListCategory().size() );

	//resonance with differentPattern that need to create a new input
	testMap["Personne C"] = InputROSmsgPtr(new InputROSmsg("Personne C","Persons",1,10));
	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_FALSE( e->processInput() );

	//verify number of input category (complement coded input)
	EXPECT_EQ( 10 , e->getInputLayer()->getListCategory().size() );
	//verify number of events (increase 1)
	EXPECT_EQ( 2, e->getEventLayer()->getListCategory().size() );

	//resonance with the same pattern as initial
	ASSERT_EQ(1,testMap.erase("Personne C"));
	ASSERT_TRUE(e->activateInputs(testMap));
	//compare vector, must be the same as initial
	EXPECT_FALSE( e->processInput() );

	//verify number of input category (complement coded input)
	EXPECT_EQ( 10 , e->getInputLayer()->getListCategory().size() );
	//verify number of events (must not increase again)
	EXPECT_EQ( 2, e->getEventLayer()->getListCategory().size() );

}

TEST_F(EpisodicMemoryResonance, resonanceNewInputCategoryInNewChannel)
{
	//resonance with one pattern
	std::map<std::string,InputROSmsgPtr> testMap;

	testMap["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));
	testMap["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_FALSE( e->processInput() );
	//verify number of input category (complement coded input)
	EXPECT_EQ( 8 , e->getInputLayer()->getListCategory().size() );
	//verify number of events
	EXPECT_EQ( 1, e->getEventLayer()->getListCategory().size() );

	//resonance with differentPattern that need to create a new input and channel
	testMap["Livre"] = InputROSmsgPtr(new InputROSmsg("Livre","Object",1,10));

	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_FALSE( e->processInput() );
	//verify number of input category (complement coded input)
	EXPECT_EQ( 10 , e->getInputLayer()->getListCategory().size());
	//verify number of events (increase 1)
	EXPECT_EQ( 2, e->getEventLayer()->getListCategory().size() );

	//resonance with the same pattern as initial
	ASSERT_EQ(1,testMap.erase("Livre"));
	ASSERT_TRUE(e->activateInputs(testMap));
	//compare vector, must be the same as initial
	EXPECT_FALSE( e->processInput() );
	//verify number of input category (complement coded input)
	EXPECT_EQ( 10 , e->getInputLayer()->getListCategory().size() );
	//verify number of events (must not increase again)
	EXPECT_EQ( 2, e->getEventLayer()->getListCategory().size() );
}

TEST_F(EpisodicMemoryResonance, createEpisode)
{
	std::map<std::string,InputROSmsgPtr> testMap;
	std::map<std::string,InputROSmsgPtr> testMap1;
	std::map<std::string,InputROSmsgPtr> testMap2;
	std::map<std::string,InputROSmsgPtr> testMap3;
	std::map<std::string,InputROSmsgPtr> testMap4;

	testMap["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap1["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap1["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap1["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap1["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap2["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap2["Goto"] = InputROSmsgPtr(new InputROSmsg("Goto","Motivation",1,10));
	testMap2["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap2["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap3["Goto"] = InputROSmsgPtr(new InputROSmsg("Goto","Motivation",1,10));
	testMap3["Corridor"] = InputROSmsgPtr(new InputROSmsg("Corridor","Location",1,10));
	testMap3["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap4["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap4["LaboExperimental"] = InputROSmsgPtr(new InputROSmsg("LaboExperimental","Location",1,10));
	testMap4["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",0.7,10));

	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap1));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap2));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap3));
	EXPECT_FALSE( e->processInput() );

	e->setLearningRateMode(LEARNING);
	ASSERT_TRUE(e->activateInputs(testMap4));
	EXPECT_TRUE( e->processInput() );

	EXPECT_EQ( 5 , e->getEventLayer()->getListCategory().size() );
	EXPECT_EQ( 1 , e->getEpisodeLayer()->getListCategory().size() );
}

TEST_F(EpisodicMemoryResonance, recognizeExactEpisode)
{
	std::map<std::string,InputROSmsgPtr> testMap;
	std::map<std::string,InputROSmsgPtr> testMap1;
	std::map<std::string,InputROSmsgPtr> testMap2;
	std::map<std::string,InputROSmsgPtr> testMap3;
	std::map<std::string,InputROSmsgPtr> testMap4;

	testMap["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap1["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap1["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap1["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap1["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap2["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap2["Goto"] = InputROSmsgPtr(new InputROSmsg("Goto","Motivation",1,10));
	testMap2["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap2["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap3["Goto"] = InputROSmsgPtr(new InputROSmsg("Goto","Motivation",1,10));
	testMap3["Corridor"] = InputROSmsgPtr(new InputROSmsg("Corridor","Location",1,10));
	testMap3["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap4["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap4["LaboExperimental"] = InputROSmsgPtr(new InputROSmsg("LaboExperimental","Location",1,10));
	testMap4["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",0.7,10));


	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap1));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap2));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap3));
	EXPECT_FALSE( e->processInput() );

	e->setLearningRateMode(LEARNING);
	ASSERT_TRUE(e->activateInputs(testMap4));
	EXPECT_TRUE( e->processInput() );

	EXPECT_EQ( 5 , e->getEventLayer()->getListCategory().size() );
	EXPECT_EQ( 1 , e->getEpisodeLayer()->getListCategory().size() );

	e->setLearningRateMode(RECOGNIZING);
	e->clearActivation();

	ASSERT_TRUE(e->activateInputs(testMap));
	//Without emotion, the vigilance is low, so the recognition is easy
	EXPECT_TRUE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap1));
	EXPECT_TRUE(e->processInput() ) ;

	ASSERT_TRUE(e->activateInputs(testMap2));
	EXPECT_TRUE(e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap3));
	EXPECT_TRUE(e->processInput() );

	e->setLearningRateMode(LEARNING);
	ASSERT_TRUE(e->activateInputs(testMap4));
	EXPECT_TRUE( e->processInput() );

	EXPECT_EQ( 5 , e->getEventLayer()->getListCategory().size() );
	EXPECT_EQ( 1 , e->getEpisodeLayer()->getListCategory().size() );
}

TEST_F(EpisodicMemoryResonance, recognizePartialEpisodeFromBeginning)
{
	std::map<std::string,InputROSmsgPtr> testMap;
	std::map<std::string,InputROSmsgPtr> testMap1;
	std::map<std::string,InputROSmsgPtr> testMap2;
	std::map<std::string,InputROSmsgPtr> testMap3;
	std::map<std::string,InputROSmsgPtr> testMap4;

	testMap["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap1["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap1["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap1["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap1["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap2["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap2["Goto"] = InputROSmsgPtr(new InputROSmsg("Goto","Motivation",1,10));
	testMap2["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap2["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap3["Goto"] = InputROSmsgPtr(new InputROSmsg("Goto","Motivation",1,10));
	testMap3["Corridor"] = InputROSmsgPtr(new InputROSmsg("Corridor","Location",1,10));
	testMap3["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap4["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap4["LaboExperimental"] = InputROSmsgPtr(new InputROSmsg("LaboExperimental","Location",1,10));
	testMap4["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",0.7,10));

	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap1));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap2));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap3));
	EXPECT_FALSE( e->processInput() );

	e->setLearningRateMode(LEARNING);
	ASSERT_TRUE(e->activateInputs(testMap4));
	EXPECT_TRUE( e->processInput() );

	EXPECT_EQ( 5 , e->getEventLayer()->getListCategory().size() );
	EXPECT_EQ( 1 , e->getEpisodeLayer()->getListCategory().size() );

	e->setLearningRateMode(RECOGNIZING);
	e->clearActivation();

	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_TRUE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap1));
	EXPECT_TRUE(e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap2));
	EXPECT_TRUE(e->processInput() ); //AFTER THREE IDENTICAL EVENTS OUT OF 5 WE SHOULD RECOGNIZE THE EPISODE (event are in the same order from beginning)

	EXPECT_EQ( 5 , e->getEventLayer()->getListCategory().size() );
	EXPECT_EQ( 1 , e->getEpisodeLayer()->getListCategory().size() );
}

TEST_F(EpisodicMemoryResonance, recognizePartialEpisodeFromEnd)
{
	std::map<std::string,InputROSmsgPtr> testMap;
	std::map<std::string,InputROSmsgPtr> testMap1;
	std::map<std::string,InputROSmsgPtr> testMap2;
	std::map<std::string,InputROSmsgPtr> testMap3;
	std::map<std::string,InputROSmsgPtr> testMap4;
	std::map<std::string,InputROSmsgPtr> testMap5;

	testMap["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap1["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap1["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap1["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap1["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap2["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap2["Goto"] = InputROSmsgPtr(new InputROSmsg("Goto","Motivation",1,10));
	testMap2["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap2["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap3["Goto"] = InputROSmsgPtr(new InputROSmsg("Goto","Motivation",1,10));
	testMap3["Corridor"] = InputROSmsgPtr(new InputROSmsg("Corridor","Location",1,10));
	testMap3["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap4["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap4["LaboExperimental"] = InputROSmsgPtr(new InputROSmsg("LaboExperimental","Location",1,10));
	testMap4["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",0.7,10));

	testMap5["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap5["LaboExperimental"] = InputROSmsgPtr(new InputROSmsg("LaboExperimental","Location",1,10));
	testMap5["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",0.7,10));
	testMap5["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));

	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap1));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap2));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap3));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap4));
	EXPECT_FALSE( e->processInput() );

	e->setLearningRateMode(LEARNING);
	ASSERT_TRUE(e->activateInputs(testMap5));
	EXPECT_TRUE( e->processInput() );

	EXPECT_EQ( 6 , e->getEventLayer()->getListCategory().size() );
	EXPECT_EQ( 1 , e->getEpisodeLayer()->getListCategory().size() );

	e->setLearningRateMode(RECOGNIZING);
	e->clearActivation();

	ASSERT_TRUE(e->activateInputs(testMap2));
	EXPECT_TRUE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap3));
	EXPECT_TRUE(e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap4));
	EXPECT_TRUE(e->processInput() ); //AFTER THREE IDENTICAL EVENTS OUT OF 5 WE SHOULD RECOGNIZE THE EPISODE (event are the last three)

	ASSERT_TRUE(e->activateInputs(testMap5));
	EXPECT_TRUE(e->processInput() );

	EXPECT_EQ( 6 , e->getEventLayer()->getListCategory().size() );
	EXPECT_EQ( 1 , e->getEpisodeLayer()->getListCategory().size() );
}

TEST_F(EpisodicMemoryResonance, create2Episode)
{
	std::map<std::string,InputROSmsgPtr> testMap;
	std::map<std::string,InputROSmsgPtr> testMap1;
	std::map<std::string,InputROSmsgPtr> testMap2;
	std::map<std::string,InputROSmsgPtr> testMap3;
	std::map<std::string,InputROSmsgPtr> testMap4;
	std::map<std::string,InputROSmsgPtr> testMap5;

	testMap["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap1["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap1["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap1["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap1["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap2["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap2["Goto"] = InputROSmsgPtr(new InputROSmsg("Goto","Motivation",1,10));
	testMap2["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap2["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap3["Goto"] = InputROSmsgPtr(new InputROSmsg("Goto","Motivation",1,10));
	testMap3["Corridor"] = InputROSmsgPtr(new InputROSmsg("Corridor","Location",1,10));
	testMap3["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap4["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap4["LaboExperimental"] = InputROSmsgPtr(new InputROSmsg("LaboExperimental","Location",1,10));
	testMap4["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",0.7,10));

	testMap5["Wander"] = InputROSmsgPtr(new InputROSmsg("Wander","Motivation",1,10));
	testMap5["LaboExperimental"] = InputROSmsgPtr(new InputROSmsg("LaboExperimental","Location",1,10));
	testMap5["Livre"] = InputROSmsgPtr(new InputROSmsg("Livre","Object",1,10));
	testMap5["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",0.7,10));

	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap1));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap2));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap3));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap4));
	EXPECT_FALSE( e->processInput() );

	e->setLearningRateMode(LEARNING);
	ASSERT_TRUE(e->activateInputs(testMap5));
	EXPECT_TRUE( e->processInput() );

	EXPECT_EQ( 6 , e->getEventLayer()->getListCategory().size() );
	EXPECT_EQ( 1 , e->getEpisodeLayer()->getListCategory().size() );

	e->setLearningRateMode(RECOGNIZING);
	e->clearActivation();

	std::map<std::string,InputROSmsgPtr> test2Map;
	std::map<std::string,InputROSmsgPtr> test2Map1;
	std::map<std::string,InputROSmsgPtr> test2Map2;
	std::map<std::string,InputROSmsgPtr> test2Map3;

	test2Map["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	test2Map["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	test2Map["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	test2Map1["Personne A"] = InputROSmsgPtr(new InputROSmsg("Personne A","Persons",1,10));
	test2Map1["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	test2Map1["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	test2Map1["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	test2Map2["Personne A"] = InputROSmsgPtr(new InputROSmsg("Personne A","Persons",1,10));
	test2Map2["Goto"] = InputROSmsgPtr(new InputROSmsg("Goto","Motivation",1,10));
	test2Map2["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	test2Map2["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	test2Map3["Goto"] = InputROSmsgPtr(new InputROSmsg("Goto","Motivation",1,10));
	test2Map3["Livre"] = InputROSmsgPtr(new InputROSmsg("Livre","Object",1,10));
	test2Map3["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	test2Map3["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	ASSERT_TRUE(e->activateInputs(test2Map));
	e->processInput();

	ASSERT_TRUE(e->activateInputs(test2Map1));
	EXPECT_FALSE(e->processInput() );

	ASSERT_TRUE(e->activateInputs(test2Map2));
	EXPECT_FALSE(e->processInput());

	e->setLearningRateMode(LEARNING);
	ASSERT_TRUE(e->activateInputs(test2Map3));
	EXPECT_TRUE( e->processInput() );

	EXPECT_EQ( 9 , e->getEventLayer()->getListCategory().size() );
	ASSERT_EQ( 2 , e->getEpisodeLayer()->getListCategory().size() );

}

TEST_F(EpisodicMemoryResonance, modulateChannelRelevance)
{
	std::map<std::string,InputROSmsgPtr> testMap;
	std::map<std::string,InputROSmsgPtr> testMap1;
	std::map<std::string,InputROSmsgPtr> testMap2;
	std::map<std::string,InputROSmsgPtr> testMap3;

	testMap["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap1["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap1["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap1["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",0.7,10));

	testMap2["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap2["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap2["Colere"] = InputROSmsgPtr(new InputROSmsg("Colere","Emotion",0.7,10));

	//The first three events are different in the emotion channel, eventLayer should create three different category
	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap1));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap2));
	EXPECT_FALSE( e->processInput() );

	EXPECT_EQ( 3 , e->getEventLayer()->getListCategory().size() );
	EXPECT_EQ( 10 , e->getInputLayer()->getListCategory().size() );

	//If we lower the relevance of the emotion channel, we should recognize a category even if the emotion is not the same
	testMap3["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap3["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap3["Surprise"] = InputROSmsgPtr(new InputROSmsg("Surprise","Emotion",1,  5  ));

	ASSERT_TRUE(e->activateInputs(testMap3));
	EXPECT_FALSE( e->processInput() );

	EXPECT_EQ( 3 , e->getEventLayer()->getListCategory().size() );
	EXPECT_EQ( 12 , e->getInputLayer()->getListCategory().size() );
}

TEST_F (EpisodicMemoryResonance, ExtendedTest)
{
	std::map<std::string,InputROSmsgPtr> testMap;
	std::map<std::string,InputROSmsgPtr> testMap2;
	std::map<std::string,InputROSmsgPtr> testMap3;
	std::map<std::string,InputROSmsgPtr> testMap4;
	std::map<std::string,InputROSmsgPtr> testMap5;
	std::map<std::string,InputROSmsgPtr> testMap6;
	std::map<std::string,InputROSmsgPtr> testMap7;

	testMap["Personne A"] = InputROSmsgPtr(new InputROSmsg("Personne A","Persons",1,10));
	testMap["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));
	testMap["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",1,10));

	testMap2["Personne A"] = InputROSmsgPtr(new InputROSmsg("Personne A","Persons",1,10));
	testMap2["Go To"] = InputROSmsgPtr(new InputROSmsg("Go To","Motivation",1,10));
	testMap2["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",1,10));
	testMap2["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));

	testMap3["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",1,10));
	testMap3["Cuisine"] = InputROSmsgPtr(new InputROSmsg("Cuisine","Location",1,10));
	testMap3["Go To"] = InputROSmsgPtr(new InputROSmsg("Go To","Motivation",1,10));

	testMap4["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",1,10));
	testMap4["Cuisine"] = InputROSmsgPtr(new InputROSmsg("Cuisine","Location",1,10));
	testMap4["Sac chips"] = InputROSmsgPtr(new InputROSmsg("Sac chips","Object",1,10));
	testMap4["Prendre"] = InputROSmsgPtr(new InputROSmsg("Prendre","Motivation",1,10));

	testMap5["Colere"] = InputROSmsgPtr(new InputROSmsg("Colere","Emotion",1,10));
	testMap5["Cuisine"] = InputROSmsgPtr(new InputROSmsg("Cuisine","Location",1,10));
	testMap5["Sac chips"] = InputROSmsgPtr(new InputROSmsg("Sac chips","Object",1,10));
	testMap5["Prendre"] = InputROSmsgPtr(new InputROSmsg("Prendre","Motivation",1,10));

	testMap6["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));
	testMap6["Sac chips"] = InputROSmsgPtr(new InputROSmsg("Sac chips","Object",1,10));
	testMap6["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",1,10));
	testMap6["Go To"] = InputROSmsgPtr(new InputROSmsg("Go To","Motivation",1,10));

	testMap7["Personne A"] = InputROSmsgPtr(new InputROSmsg("Personne A","Persons",1,10));
	testMap7["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));
	testMap7["Sac chips"] = InputROSmsgPtr(new InputROSmsg("Sac chips","Object",1,10));
	testMap7["Donner"] = InputROSmsgPtr(new InputROSmsg("Donner","Motivation",1,10));
	testMap7["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",1,10));

	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap2));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap3));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap4));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap5));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap6));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap7));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap2));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap3));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap4));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap5));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap6));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap7));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap2));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap3));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap4));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap5));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap6));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap7));
	EXPECT_FALSE( e->processInput() );

	EXPECT_EQ( 7 , e->getEventLayer()->getListCategory().size() );
	EXPECT_EQ( 22 , e->getInputLayer()->getListCategory().size() );
}

TEST_F (EpisodicMemoryResonance, HighEmotionIntensity)
{
	std::map<std::string,InputROSmsgPtr> testMap;
	std::map<std::string,InputROSmsgPtr> testMap1;
	std::map<std::string,InputROSmsgPtr> testMap2;
	std::map<std::string,InputROSmsgPtr> testMap3;
	std::map<std::string,InputROSmsgPtr> testMap4;

	testMap["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap1["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap1["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap1["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap1["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap2["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap2["Goto"] = InputROSmsgPtr(new InputROSmsg("Goto","Motivation",1,10));
	testMap2["Labo1033"] = InputROSmsgPtr(new InputROSmsg("Labo1033","Location",1,10));
	testMap2["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap3["Goto"] = InputROSmsgPtr(new InputROSmsg("Goto","Motivation",1,10));
	testMap3["Corridor"] = InputROSmsgPtr(new InputROSmsg("Corridor","Location",1,10));
	testMap3["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap4["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap4["LaboExperimental"] = InputROSmsgPtr(new InputROSmsg("LaboExperimental","Location",1,10));
	testMap4["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",0.7,10));

	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_FALSE( e->processInput() );

	e->onEmotionChange(0.2);

	ASSERT_TRUE(e->activateInputs(testMap1));
	EXPECT_FALSE( e->processInput() );

	e->onEmotionChange(0.4);

	ASSERT_TRUE(e->activateInputs(testMap2));
	EXPECT_FALSE( e->processInput() );

	e->onEmotionChange(0.6);

	ASSERT_TRUE(e->activateInputs(testMap3));
	EXPECT_FALSE( e->processInput() );

	e->onEmotionChange(0.9);

	e->setLearningRateMode(LEARNING);
	ASSERT_TRUE(e->activateInputs(testMap4));
	EXPECT_TRUE( e->processInput() );

	EXPECT_EQ( 5 , e->getEventLayer()->getListCategory().size() );
	EXPECT_EQ( 1 , e->getEpisodeLayer()->getListCategory().size() );

	e->clearActivation();

	//we reproduce the same pattern. it will take more events before a recognition occur because the
	//vigilance will be set higher according to the high emotion intensity
	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap1));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap2));
	e->processInput() ;

	ASSERT_TRUE(e->activateInputs(testMap3));
	EXPECT_TRUE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap4));
	EXPECT_TRUE( e->processInput() );
}

TEST_F (EpisodicMemoryResonance, HighEmotionIntensityLongEpisode)
{
	std::map<std::string,InputROSmsgPtr> testMap;
	std::map<std::string,InputROSmsgPtr> testMap2;
	std::map<std::string,InputROSmsgPtr> testMap3;
	std::map<std::string,InputROSmsgPtr> testMap4;
	std::map<std::string,InputROSmsgPtr> testMap5;
	std::map<std::string,InputROSmsgPtr> testMap6;
	std::map<std::string,InputROSmsgPtr> testMap7;
	std::map<std::string,InputROSmsgPtr> testMap8;
	std::map<std::string,InputROSmsgPtr> testMap9;
	std::map<std::string,InputROSmsgPtr> testMap10;

	testMap["Personne A"] = InputROSmsgPtr(new InputROSmsg("Personne A","Persons",1,10));
	testMap["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));
	testMap["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",1,10));

	testMap2["Personne A"] = InputROSmsgPtr(new InputROSmsg("Personne A","Persons",1,10));
	testMap2["Go To"] = InputROSmsgPtr(new InputROSmsg("Go To","Motivation",1,10));
	testMap2["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",1,10));
	testMap2["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));

	testMap3["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",1,10));
	testMap3["Cuisine"] = InputROSmsgPtr(new InputROSmsg("Cuisine","Location",1,10));
	testMap3["Go To"] = InputROSmsgPtr(new InputROSmsg("Go To","Motivation",1,10));

	testMap4["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",1,10));
	testMap4["Cuisine"] = InputROSmsgPtr(new InputROSmsg("Cuisine","Location",1,10));
	testMap4["Sac chips"] = InputROSmsgPtr(new InputROSmsg("Sac chips","Object",1,10));
	testMap4["Prendre"] = InputROSmsgPtr(new InputROSmsg("Prendre","Motivation",1,10));

	testMap5["Colere"] = InputROSmsgPtr(new InputROSmsg("Colere","Emotion",1,10));
	testMap5["Cuisine"] = InputROSmsgPtr(new InputROSmsg("Cuisine","Location",1,10));
	testMap5["Sac chips"] = InputROSmsgPtr(new InputROSmsg("Sac chips","Object",1,10));
	testMap5["Prendre"] = InputROSmsgPtr(new InputROSmsg("Prendre","Motivation",1,10));

	testMap6["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));
	testMap6["Sac chips"] = InputROSmsgPtr(new InputROSmsg("Sac chips","Object",1,10));
	testMap6["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",1,10));
	testMap6["Go To"] = InputROSmsgPtr(new InputROSmsg("Go To","Motivation",1,10));

	testMap7["Personne A"] = InputROSmsgPtr(new InputROSmsg("Personne A","Persons",1,10));
	testMap7["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));
	testMap7["Sac chips"] = InputROSmsgPtr(new InputROSmsg("Sac chips","Object",1,10));
	testMap7["Donner"] = InputROSmsgPtr(new InputROSmsg("Donner","Motivation",1,10));
	testMap7["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",1,10));

	testMap8["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));
	testMap8["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap8["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",1,10));

	testMap9["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap9["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap9["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));
	testMap9["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",1,10));

	testMap10["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap10["Avoid"] = InputROSmsgPtr(new InputROSmsg("Avoid","Motivation",1,10));
	testMap10["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));
	testMap10["Colere"] = InputROSmsgPtr(new InputROSmsg("Colere","Emotion",1,10));

	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap2));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap3));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap4));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap5));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap6));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap7));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap8));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap9));
	EXPECT_FALSE( e->processInput() );

	e->onEmotionChange(0.6);
	e->setLearningRateMode(LEARNING);

	ASSERT_TRUE(e->activateInputs(testMap10));
	EXPECT_TRUE( e->processInput() );

	EXPECT_EQ( 10 , e->getEventLayer()->getListCategory().size() );
	EXPECT_EQ( 1 , e->getEpisodeLayer()->getListCategory().size() );

	e->clearActivation();

	//we reproduce the same pattern. it will take more events before a recognition occur because the
	//vigilance will be set higher according to the high emotion intensity. However, the vigilance is
	//lowered because the episode is long
	ASSERT_TRUE(e->activateInputs(testMap));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap2));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap3));
	EXPECT_FALSE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap4));
	 e->processInput() ;

	ASSERT_TRUE(e->activateInputs(testMap5));
	e->processInput() ;

	ASSERT_TRUE(e->activateInputs(testMap6));
	e->processInput() ;

	ASSERT_TRUE(e->activateInputs(testMap7));
	 e->processInput() ;

	ASSERT_TRUE(e->activateInputs(testMap8));
	EXPECT_TRUE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap9));
	EXPECT_TRUE( e->processInput() );

	ASSERT_TRUE(e->activateInputs(testMap10));
	EXPECT_TRUE( e->processInput() );

}

#endif
