#include "chLiteWorld.h"
#include "chLiteBody.h"
#include "chLiteJoint.h"

using std::vector;
using std::map;
using std::pair;

typedef map<Chan::ArbiterKey, Chan::chLiteArbiter>::iterator ArbIter;
typedef pair<Chan::ArbiterKey, Chan::chLiteArbiter> ArbPair;

bool Chan::chLiteWorld::accumulateImpulses = true;
bool Chan::chLiteWorld::warmStarting = true;
bool Chan::chLiteWorld::positionCorrection = true;

void Chan::chLiteWorld::Add(chLiteBody * body)
{
	bodies.push_back(body);
}

void Chan::chLiteWorld::Add(chLiteJoint * joint)
{
	joints.push_back(joint);
}

void Chan::chLiteWorld::Clear()
{
	bodies.clear();
	joints.clear();
	arbiters.clear();
}

void Chan::chLiteWorld::BroadPhase()
{
	// O(n^2) broad-phase
	for(int i = 0; i < (int)bodies.size(); ++i)
	{
		chLiteBody* bi = bodies[i];

		for (int j = i + 1; j < (int)bodies.size(); ++j)
		{
			chLiteBody* bj = bodies[j];

			if (bi->invMass == ChReal(0.0) && bj->invMass == ChReal(0.0))
				continue;

			chLiteArbiter newArb(bi, bj);
			ArbiterKey key(bi, bj);

			if (newArb.numContacts > 0)
			{
				ArbIter iter = arbiters.find(key);
				if (iter == arbiters.end())
				{
					arbiters.insert(ArbPair(key, newArb));
				}
				else
				{
					iter->second.Update(newArb.contacts, newArb.numContacts);
				}
			}
			else
			{
				arbiters.erase(key);
			}
		}
	}
}


void Chan::chLiteWorld::Step(ChReal dt)
{
	ChReal inv_dt = dt > ChReal(0.0) ? ChReal(1.0) / dt : ChReal(0.0);

	// Determine overlapping bodies and update contact points
	BroadPhase();

	// integrate forces.
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		chLiteBody* b = bodies[i];

		if (b->invMass == ChReal(0.0))
			continue;

		b->velocity += dt * (gravity + b->invMass * b->force);
		b->angularVelocity += dt * b->invI * b->torque;
	}

	// Perform pre-steps
	for (ArbIter arb = arbiters.begin(); arb != arbiters.end(); ++arb)
	{
		arb->second.PreStep(inv_dt);
	}

	for (int i = 0; i < (int)joints.size(); ++i)
	{
		joints[i]->PreStep(inv_dt);
	}

	// Perform iterations
	for (int i = 0; i < iterations; ++i)
	{
		for (ArbIter arb = arbiters.begin(); arb != arbiters.end(); ++arb)
		{
			arb->second.ApplyImpulse();
		}

		for (int j = 0; j < (int)joints.size(); ++j)
		{
			joints[j]->ApplyImpulse();
		}
	}

	// Integrate Velocities
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		chLiteBody* b = bodies[i];
		b->position += dt * b->velocity;
		b->rotation += dt * b->angularVelocity;

		b->force.Set(ChReal(0.0), ChReal(0.0));
		b->torque = ChReal(0.0);
	}
}

