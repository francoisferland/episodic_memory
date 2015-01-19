#include <gtest/gtest.h>
#include "common.hpp"

/*
 * Behavior looking for legs and faces in the egosphere,
 * do its best to place and track people
 *
 * TODO purely evenemential implementation?
 *      ie. face.onUpdate(checkConcistency(face,legs)) when
 *      face and legs are associated
 *      + no need to search for associated legs each time
 *      - disconnection seems complex
 */

// Positon(center of mass (incertainty explained here))

class PeopleDetector
{
public :
	PeopleDetector(Egos * egosphere) :
		egosphere_(egosphere)
	{
		// look if we can identify peoples with informations already
		// in the internal representation.
		for(Info::Itr info = egosphere_->infoItr(); info; info++) {
			onNewInfo(info);
		}

		// Will advert added informations (ie. new informations)
		egosphere_->onNewInfo(
			boost::bind(&PeopleDetector::onNewInfo, this, _1));
	}

	void onNewInfo(const Info::Ptr & info)
	{
		if(info->is("legs")) {
			onLegsUpdate(info);
			info->onUpdate(boost::bind(
				&PeopleDetector::onLegsUpdate, this, _1));
		}
		if(info->is("face")) {
			onFaceUpdate(info);
			info->onUpdate(boost::bind(
				&PeopleDetector::onFaceUpdate, this, _1));
		}
	}

	/*
	 * Called whenever a leg couple is updated in the internal
	 * representation.
	 *
	 * If a person is already associated with this legs, updates his
	 * position.
	 *
	 * Else if a person with no legs is near, add legs to it
	 *
	 * Else, creates a new person associted with this legs.
	 */
	void onLegsUpdate(const Info::Ptr & legs)
	{
		Info::Itr person = egosphere_->infoByAssociation(legs, "person");
		if(person) {
			person->set(Params()
				.value("azimut", legs->get<Angle>("azimut"))
				.value("dist", legs->get<Length>("dist")));
			return;
		}
		// For each person in the internal representation
		for(Info::Itr p = egosphere_->infoByTag("person"); p; p++) {
			// if the person has no associated legs
			// and is close to the legs
			if(!egosphere_->infoByAssociation(p, "legs") && mayBeNear(p, legs))
			{
				// associate legs to this person, update the person's position
				egosphere_->associate(p, legs);
				p->set(Params()
					.value("azimut", legs->get<Angle>("azimut"))
					.value("dist", legs->get<Length>("dist")));
				return;
			}
		}
		// There no existing person with no legs near, adding a new one
		// TODO low accuracy
		egosphere_->associate(legs, egosphere_->newInfo("person", 0, Params()
			.value("azimut", legs->get<Angle>("azimut"))
			.value("dist", legs->get<Length>("dist"))
		));
	}

	/*
	 * Called whenever a face is updated in the internal
	 * representation.
	 *
	 * If a person is already associated with this face, check if
	 * the face position is still consistent with legs position.
	 *
	 * Else if a unidentified person is near, associate the face with
	 * it and identify it
	 *
	 * Else add a person associated with the face.
	 * TODO low acuracy, no legs to confirm
	 */
	void onFaceUpdate(const Info::Ptr & face)
	{
		Info::Itr person = egosphere_->infoByAssociation(face, "person");
		if(person)
		{
			Info::Itr legs = egosphere_->infoByAssociation(person, "legs");
			// check if consistent with legs
			if (legs) {
				if(mayBeNear(face, legs)) {
					return;
				} else {
					egosphere_->dissociate(face, person);
					person->remove("id");
				}
			} else {
				person->set(Params()
					.value("id", face->get<int>("id"))
					.value("azimut", face->get<Angle>("azimut")));
				return;
			}
		}

		// For each person in the internal representation
		for(Info::Itr p = egosphere_->infoByTag("person"); p; p++) {
			if(!egosphere_->infoByAssociation(p, "face") && mayBeNear(p, face))
			{
				// associate face to this person, update the person's id
				egosphere_->associate(p, face);
				p->set(Params()
					.value("id", face->get<int>("id")));
				return;
			}
		}

		egosphere_->associate(face, egosphere_->newInfo("person", 0, Params()
			.value("id", face->get<int>("id"))
			.value("azimut", face->get<Angle>("azimut"))));
	}

	/*
	 * Tells if the informations were close to each other
	 * according to the latest news.
	 */
	static bool mayBeNear(const Info::Ptr & i1, const Info::Ptr & i2)
	{
		Info::Ptr infos[] = {i1, i2};
		double tAzim, tDist;
		try {
			Info::Itr it = Info::Itr(infos + 0, infos + 2);
			tAzim = Info::latestCommonUpdateStamp(it, "azimut");
		} catch (ParameterNotDefined & e) {
			tAzim = 0;
		}
		try {
			Info::Itr it = Info::Itr(infos + 0, infos + 2);
			tDist = Info::latestCommonUpdateStamp(it, "dist");
		} catch (ParameterNotDefined & e) {
			tDist = 0;
		}
		if (
			(!i1->has("azimut", tAzim) || !i2->has("azimut", tAzim)) &&
			(!i1->has("dist",   tDist) || !i2->has("dist",   tDist))
		) {
			return false;
		}

		int dDist = 0;
		int dAzim = 0;


		if (i1->has("azimut", tAzim) && i2->has("azimut", tAzim)) {
			dAzim = i1->get<Angle>("azimut",tAzim) - i2->get<Angle>("azimut", tAzim);
		}

		if (i1->has("dist", tDist) && i2->has("dist", tDist)) {
			dDist = (double) i1->get<Length>("dist", tDist) -
				    (double) i2->get<Length>("dist", tDist);
		}
		// distance +- 0.25m if provided, azimut +- 5° if provided
		return fabs(dDist) < 0.25 && fabs(dAzim) < 0.08;
	}

private:

	Egos * egosphere_;
};

class LegsOnly : public testing::Test
{
protected:
	void SetUp() {
		// Creates the internal representation
		legs = e.newInfo("legs", 0, Params()
			.value("azimut", Angle(0.5))
			.value("dist", Length(2)));
		detector.reset(new PeopleDetector(&e));
	}
	Egos e;
	boost::shared_ptr<PeopleDetector> detector;
	Info::Ptr legs;
};

// Checks if the representation includes 2 infos
// (legs and person)
TEST_F(LegsOnly, TwoInfos)
{
	Info::Itr it = e.infoItr();
	EXPECT_EQ(2, it.count());
}

// Checks that the added information is a person
TEST_F(LegsOnly, PersonAdded)
{
	Info::Itr person = e.infoByTag("person");
	EXPECT_EQ(1, person.count());
}

// Checks that the person is associated with the legs
TEST_F(LegsOnly, PersonAssociated)
{
	Info::Itr person = e.infoByAssociation(legs);
	ASSERT_EQ(1, person.count());
	EXPECT_TRUE(person->is("person"));
}

// Checks that the person's position is the same as it's legs
TEST_F(LegsOnly, SamePosition)
{
	Info::Itr person = e.infoByTag("person");
	ASSERT_EQ(1, person.count());
	EXPECT_EQ(legs->get<Angle>("azimut"), person->get<Angle>("azimut"));
}

// Checks that a person is added when adding legs
TEST_F(LegsOnly, PersonAddedWhenLegsAdded)
{
	e.newInfo("legs", 0, Params()
		.value("azimut", Angle(1))
		.value("dist", Length(2)));
	Info::Itr person = e.infoByTag("person");
	ASSERT_EQ(2, person.count());
}

// Checks if the persons moves when the legs moves
TEST_F(LegsOnly, PersonMovesWhenLegsMoves)
{
	legs->set(Params()
		.value("azimut", Angle(1)));
	EXPECT_EQ(1, (double) legs->get<Angle>("azimut"));
	Info::Itr person = e.infoByTag("person");
	ASSERT_EQ(1, person.count());
	EXPECT_EQ(legs->get<Angle>("azimut"), person->get<Angle>("azimut"));
}

class FaceOnly : public testing::Test
{
protected:
	void SetUp() {
		face = e.newInfo("face", 0, Params()
			.value("azimut", Angle(0.5))
			.value("id", 1));
		detector.reset(new PeopleDetector(&e));
	}
	Egos e;
	boost::shared_ptr<PeopleDetector> detector;
	Info::Ptr face;
};

// Checks that the added information is a person
TEST_F(FaceOnly, PersonAdded)
{
	Info::Itr all = e.infoItr();
	EXPECT_EQ(2, all.count());
	Info::Itr person = e.infoByTag("person");
	EXPECT_EQ(1, person.count());
}

// Checks that the person is associated with the face
TEST_F(FaceOnly, PersonAssociated)
{
	Info::Itr person = e.infoByAssociation(face);
	ASSERT_EQ(1, person.count());
	EXPECT_TRUE(person->is("person"));
}

// Checks that the person's id and azimut are the same as the face
TEST_F(FaceOnly, SamePosition)
{
	Info::Itr person = e.infoByTag("person");
	ASSERT_EQ(1, person.count());
	EXPECT_EQ(face->get<int>("id"), person->get<int>("id"));
	EXPECT_EQ(face->get<Angle>("azimut"), person->get<Angle>("azimut"));
}

// Checks that a person is added when adding a new face
TEST_F(FaceOnly, PersonAddedWhenLegsAdded)
{
	e.newInfo("face", 0, Params()
		.value("azimut", Angle(1))
		.value("id", 13));
	Info::Itr person = e.infoByTag("person");
	ASSERT_EQ(2, person.count());
}

// Checks if the persons moves when the face moves
// (There is no legs associated, so the face azimut should be used
TEST_F(FaceOnly, PersonMovesWhenFaceMoves)
{
	face->set(Params()
		.value("azimut", Angle(1)));
	EXPECT_EQ(1, (double) face->get<Angle>("azimut"));
	Info::Itr person = e.infoByTag("person");
	ASSERT_EQ(1, person.count());
	EXPECT_EQ(face->get<Angle>("azimut"), person->get<Angle>("azimut"));
}

class FaceAndLegs : public testing::Test
{
protected:
	void SetUp() {
		face = e.newInfo("face", 0, Params()
			.value("azimut", Angle(0.5))
			.value("id", 1));
		legs = e.newInfo("legs", 0, Params()
			.value("azimut", Angle(0.5))
			.value("dist", Length(2.0)));
		detector.reset(new PeopleDetector(&e));
	}
	Egos e;
	boost::shared_ptr<PeopleDetector> detector;
	Info::Ptr face;
	Info::Ptr legs;
};

// Checks that the added information is a person
TEST_F(FaceAndLegs, PersonAdded)
{
	Info::Itr all = e.infoItr();
	EXPECT_EQ(3, all.count());
	Info::Itr person = e.infoByTag("person");
	EXPECT_EQ(1, person.count());
}

// Checks that the person is associated with the face and legs
TEST_F(FaceAndLegs, PersonAssociated)
{
	Info::Itr person = e.infoByAssociation(face);
	ASSERT_EQ(1, person.count());
	EXPECT_TRUE(person->is("person"));
	person = e.infoByAssociation(legs);
	ASSERT_EQ(1, person.count());
	EXPECT_TRUE(person->is("person"));
}

// Checks that the person's id is the same as the face,
// position is the same as the legs
TEST_F(FaceAndLegs, SameParams)
{
	Info::Itr person = e.infoByTag("person");
	ASSERT_EQ(1, person.count());
	EXPECT_EQ(face->get<int>("id"), person->get<int>("id"));
	EXPECT_EQ(legs->get<Angle>("azimut"), person->get<Angle>("azimut"));
	EXPECT_EQ(legs->get<Length>("dist"), person->get<Length>("dist"));
}

/*// Checks that a person is added when adding a new face
TEST_F(FaceAndLegs, PersonAddedWhenLegsAdded)
{
	e.newInfo("face", Params()
		.value("azimut", Angle(1))
		.value("id", 13));
	Info::Itr person = e.infoByTag("person");
	ASSERT_EQ(2, person.count());
}

// Checks if the persons moves when the face moves
// (There is no legs associated, so the face azimut should be used
TEST_F(FaceAndLegs, PersonMovesWhenFaceMoves)
{
	face->set(Params()
		.value("azimut", Angle(1)));
	EXPECT_EQ(1, (double) face->get<Angle>("azimut"));
	Info::Itr person = e.infoByTag("person");
	ASSERT_EQ(1, person.count());
	EXPECT_EQ(face->get<Angle>("azimut"), person->get<Angle>("azimut"));
}*/

// TODO (in egosphere test)
// delete the detector and check what appens with callbacks

int main(int argc, char *argv[])
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
