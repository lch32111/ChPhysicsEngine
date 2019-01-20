//#include <iostream>
//
//#include "chLiteWorld.h"
//#include "chLiteBody.h"
//#include "chLiteJoint.h"
//
//int main()
//{
//	Chan::chLiteBody bodies[3];
//	Chan::chLiteJoint joints[1];
//
//	Chan::chLiteBody* bomb = NULL;
//
//	float timestep = 1.0f / 60.f;
//	int iterations = 10;
//	Chan::ChVector2 gravity(0.f, -10.f);
//
//	int numBodies = 0;
//	int numJoints = 0;
//
//	Chan::chLiteWorld world(gravity, iterations);
//
//	Chan::chLiteBody* b = bodies;
//
//	b->Set(Chan::ChVector2(100.f, 20.f), Chreal_max);
//	b->position.Set(0.f, -0.5f * b->width.y);
//	world.Add(b); 
//	++b; ++numBodies;
//
//	b->Set(Chan::ChVector2(1.f, 1.f), 200.f);
//	b->position.Set(0.f, 5.0f);
//	world.Add(b);
//	++b; ++numBodies;
//
//	while (1)
//	{
//		world.Step(timestep);
//
//		std::cout << bodies[1].position.y << '\n';
//	}
//}