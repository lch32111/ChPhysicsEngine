#pragma once

#include "../../ChanPhysicsPlayGround/chLiteWorld.h"
#include "../../ChanPhysicsPlayGround/chLiteBody.h"
#include "../../ChanPhysicsPlayGround/chLiteJoint.h"

class Physics
{
public:
	Physics();
	~Physics();

	void step();
	Chan::chLiteWorld* getWorld();

	void launchBomb();

	void demo0();
	void demo1();
	void demo2();
	void demo3();
	void demo4();
	void demo5();
	void demo6();

	void setWarmStart(bool w);
	bool isWarmStart();
	void setAccumImpulse(bool a);
	bool isAccumImpulse();
	void setPosCorrection(bool c);
	bool isPosCoorection();

	void increaseIteration();
	void decreaseIteration();
 private:
	int numBodies;
	int numJoints;
	Chan::ChVector2 gravity;
	float timeStep;
	int maxIteration;
	Chan::chLiteWorld* world;


	Chan::chLiteBody bodies[200];
	Chan::chLiteJoint joints[10];
	Chan::chLiteBody* bomb;
};