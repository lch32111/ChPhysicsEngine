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
private:
	int numBodies;
	int numJoints;
	Chan::ChVector2 gravity;
	float timeStep;
	int iteration;
	Chan::chLiteWorld* world;


	Chan::chLiteBody bodies[20];
	Chan::chLiteJoint joints[10];
	Chan::chLiteBody* bomb;
};