#pragma once
#ifndef __CH_LITE_JOINT_H__
#define __CH_LITE_JOINT_H__

#include "chMath.h"

namespace Chan
{
	struct chLiteBody;

	struct chLiteJoint
	{
		chLiteJoint() 
			: body1(0), body2(0),
			P(0.0, 0.0), 
			biasFactor((ChReal)0.2),
			softness((ChReal)0.0)
		{}

		void Set(chLiteBody* body1, chLiteBody* body2, const ChVector2& anchor);

		void PreStep(ChReal inv_dt);
		void ApplyImpulse();

		ChMat22 M;
		ChVector2 localAnchor1, localAnchor2;
		ChVector2 r1, r2;
		ChVector2 bias;
		ChVector2 P; // accumulated impulse
		chLiteBody* body1;
		chLiteBody* body2;
		ChReal biasFactor;
		ChReal softness;
	};
}

#endif