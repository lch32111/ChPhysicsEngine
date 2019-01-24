#pragma once
#ifndef __CH_LITE_BODY_H__
#define __CH_LITE_BODY_H__

#include "chMath.h"

namespace Chan
{
	struct chLiteBody
	{
	public:
		chLiteBody();

		void Set(const ChVector2& w, ChReal m);

		void AddForce(const ChVector2& f)
		{
			force += f;
		}

		ChVector2 position;
		ChReal rotation;

		ChVector2 velocity;
		ChReal angularVelocity;

		ChVector2 force;
		ChReal torque;

		ChVector2 width;

		ChReal friction;
		ChReal mass, invMass;
		ChReal I, invI;

		int proxyId;
	};
}



#endif