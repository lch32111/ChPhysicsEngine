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

	void demo1();
	void demo3();
	void demo4();
	void demo5();
private:
	int numBodies;
	int numJoints;
	Chan::ChVector2 gravity;
	float timeStep;
	int iteration;
	Chan::chLiteWorld* world;


	Chan::chLiteBody bodies[200];
	Chan::chLiteJoint joints[10];
	Chan::chLiteBody* bomb;
};