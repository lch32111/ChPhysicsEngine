#pragma once
#ifndef __CH_LITE_WORLD_H__
#define __CH_LITE_WORLD_H__

#include <vector>
#include <map>
#include "chMath.h"
#include "chLiteArbiter.h"

namespace Chan
{
	struct chLiteBody;
	struct chLiteJoint;

	class chLiteWorld
	{
	public:
		chLiteWorld(ChVector2 gravity, int iterations)
			: gravity(gravity), iterations(iterations) { }

		void Add(chLiteBody* body);
		void Add(chLiteJoint* joint);
		void Clear();

		void Step(ChReal dt);

		void BroadPhase();

		std::vector<chLiteBody*> bodies;
		std::vector<chLiteJoint*> joints;
		std::map<ArbiterKey, chLiteArbiter> arbiters;
		ChVector2 gravity;
		int iterations;
		static bool accumulateImpulses;
		static bool warmStarting;
		static bool positionCorrection;
	};
}


#endif




