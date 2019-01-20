#include "Physics.h"

Physics::Physics()
{
	timeStep = 1.f / 60.f;
	iteration = 10;
	gravity.Set(0.f, -10.f);

	numBodies = 0;
	numJoints = 0;

	world = new Chan::chLiteWorld(gravity, iteration);

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
