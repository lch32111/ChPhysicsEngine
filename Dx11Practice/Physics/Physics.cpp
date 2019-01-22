#include "Physics.h"

Physics::Physics()
{
	timeStep = 1.f / 60.f;
	maxIteration = 20;
	gravity.Set(0.f, -10.f);

	numBodies = 0;
	numJoints = 0;

	world = new Chan::chLiteWorld(gravity, maxIteration / 5);
}

Physics::~Physics()
{
	delete world;
}

void Physics::step()
{
	world->Step(timeStep);
}

Chan::chLiteWorld * Physics::getWorld()
{
	return world;
}

void Physics::launchBomb()
{
	//if (!bomb)
	//{
	//	bomb = bodies + numBodies;
	//	bomb->Set(Chan::ChVector2(1.f, 1.f), 50.f);
	//	bomb->friction = 0.2f;
	//	world->Add(bomb);
	//	++numBodies;
	//}

	bomb = bodies + numBodies;
	bomb->Set(Chan::ChVector2(1.f, 1.f), 50.f);
	bomb->friction = 0.2f;
	world->Add(bomb);
	++numBodies;

	bomb->position.Set(Chan::Random(-15.f, 15.f), 15.f);
	bomb->rotation = Chan::Random(-1.5f, 1.5f);
	bomb->velocity = -1.5f * bomb->position;
	bomb->angularVelocity = Chan::Random(-20.f, 20.f);
}

void Physics::demo0()
{
	world->Clear();
	numBodies = 0;
	numJoints = 0;
	bomb = NULL;

	Chan::chLiteBody* b = bodies;

	b->Set(Chan::ChVector2(100.0f, 20.0f), FLT_MAX);
	b->position.Set(0.0f, -0.5f * b->width.y);
	world->Add(b);
	++b; ++numBodies;

	b->Set(Chan::ChVector2(1.0f, 1.0f), 100.0f);
	b->position.Set(0.0f, 4.0f);
	world->Add(b);
	++b; ++numBodies;

	b->Set(Chan::ChVector2(1.0f, 1.0f), 200.0f);
	b->position.Set(0.0f, 5.0f);
	world->Add(b);
	++b; ++numBodies;
}

void Physics::demo1()
{
	world->Clear();
	numBodies = 0;
	numJoints = 0;
	bomb = NULL;

	Chan::chLiteBody* b = bodies;

	b->Set(Chan::ChVector2(100.0f, 20.0f), FLT_MAX);
	b->position.Set(0.0f, -0.5f * b->width.y);
	world->Add(b);
	++b; ++numBodies;

	b->Set(Chan::ChVector2(1.0f, 1.0f), 200.0f);
	b->position.Set(0.0f, 4.0f);
	world->Add(b);
	++b; ++numBodies;
}

void Physics::demo2()
{
	world->Clear();
	numBodies = 0;
	numJoints = 0;
	bomb = NULL;

	Chan::chLiteBody* b = bodies;

	b->Set(Chan::ChVector2(100.f, 20.f), Chreal_max);
	b->position.Set(0.f, -0.5f * b->width.y);
	world->Add(b);
	++b; ++numBodies;

	bool wideandshort = false;

	for (unsigned i = 0; i < 8; ++i)
	{
		b->Set(Chan::ChVector2(wideandshort ? 0.3f : 1.0f, 0.5f), 1.f);
		b->position.Set(0.f, 0.51f + i * 0.65f);
		world->Add(b);
		++b; ++numBodies;

		wideandshort = !wideandshort;
	}
}

void Physics::demo3()
{
	world->Clear();
	numBodies = 0;
	numJoints = 0;
	bomb = NULL;

	Chan::chLiteBody* b = bodies;

	b->Set(Chan::ChVector2(100.0f, 20.0f), FLT_MAX);
	b->position.Set(0.0f, -0.5f * b->width.y);
	world->Add(b);
	++b; ++numBodies;

	b->Set(Chan::ChVector2(13.0f, 0.25f), FLT_MAX);
	b->position.Set(-2.0f, 11.0f);
	b->rotation = -0.25f;
	world->Add(b);
	++b; ++numBodies;

	b->Set(Chan::ChVector2(0.25f, 1.0f), FLT_MAX);
	b->position.Set(5.25f, 9.5f);
	world->Add(b);
	++b; ++numBodies;

	b->Set(Chan::ChVector2(13.0f, 0.25f), FLT_MAX);
	b->position.Set(2.0f, 7.0f);
	b->rotation = 0.25f;
	world->Add(b);
	++b; ++numBodies;

	b->Set(Chan::ChVector2(0.25f, 1.0f), FLT_MAX);
	b->position.Set(-5.25f, 5.5f);
	world->Add(b);
	++b; ++numBodies;

	b->Set(Chan::ChVector2(13.0f, 0.25f), FLT_MAX);
	b->position.Set(-2.0f, 3.0f);
	b->rotation = -0.25f;
	world->Add(b);
	++b; ++numBodies;

	float friction[5] = { 0.75f, 0.5f, 0.35f, 0.1f, 0.0f };
	for (int i = 0; i < 5; ++i)
	{
		b->Set(Chan::ChVector2(0.5f, 0.5f), 25.0f);
		b->friction = friction[i];
		b->position.Set(-7.5f + 2.0f * i, 14.0f);
		world->Add(b);
		++b; ++numBodies;
	}
}

void Physics::demo4()
{
	world->Clear();
	numBodies = 0;
	numJoints = 0;
	bomb = NULL;

	Chan::chLiteBody* b = bodies;
	b->Set(Chan::ChVector2(100.0f, 20.0f), FLT_MAX);
	b->friction = 0.2f;
	b->position.Set(0.0f, -0.5f * b->width.y);
	b->rotation = 0.0f;
	world->Add(b);
	++b; ++numBodies;

	for (int i = 0; i < 15; ++i)
	{
		b->Set(Chan::ChVector2(1.0f, 1.0f), 1.0f);
		b->friction = 0.2f;
		float x = Chan::Random(-0.1f, 0.1f);
		b->position.Set(x, 0.51f + 1.05f * i);
		world->Add(b);
		++b; ++numBodies;
	}
}

void Physics::demo5()
{
	world->Clear();
	numBodies = 0;
	numJoints = 0;
	bomb = NULL;

	Chan::chLiteBody* b = bodies;

	b->Set(Chan::ChVector2(100.0f, 20.0f), FLT_MAX);
	b->friction = 0.2f;
	b->position.Set(0.0f, -0.5f * b->width.y);
	b->rotation = 0.0f;
	world->Add(b);
	++b; ++numBodies;

	Chan::ChVector2 x(-6.0f, 0.75f);
	Chan::ChVector2 y;

	for (int i = 0; i < 7; ++i)
	{
		y = x;

		for (int j = i; j < 7; ++j)
		{
			b->Set(Chan::ChVector2(1.0f, 1.0f), 10.0f);
			b->friction = 0.2f;
			b->position = y;
			world->Add(b);
			++b; ++numBodies;

			y += Chan::ChVector2(1.125f, 0.0f);
		}

		//x += Chan::ChVector2(0.5625f, 1.125f);
		x += Chan::ChVector2(0.5625f, 2.0f);
	}
}

void Physics::setWarmStart(bool w)
{
	world->warmStarting = w;
}

bool Physics::isWarmStart()
{
	return world->warmStarting;
}

void Physics::setAccumImpulse(bool a)
{
	world->accumulateImpulses = a;
}

bool Physics::isAccumImpulse()
{
	return world->accumulateImpulses;
}

void Physics::setPosCorrection(bool c)
{
	world->positionCorrection = c;
}

bool Physics::isPosCoorection()
{
	return world->positionCorrection;
}

void Physics::increaseIteration()
{
	if (world->iterations < maxIteration)
		++world->iterations;
}

void Physics::decreaseIteration()
{
	if (world->iterations > 1)
		--world->iterations;
}


