#pragma once
#ifndef __CH_LITE_ARBITER_H__
#define __CH_LITE_ARBITER_H__

#include "chMath.h"

namespace Chan
{
	struct chLiteBody;

	union FeaturePair
	{
		struct Edges
		{
			char inEdge1;
			char outEdge1;
			char inEdge2;
			char outEdge2;
		} e;
		int value;
	};

	struct Contact
	{
		Contact() : Pn(ChReal(0.0)), Pt(ChReal(0.0)), Pnb(ChReal(0.0))
		{ }

		ChVector2 position;
		ChVector2 normal;
		ChVector2 r1, r2;
		ChReal separation;
		ChReal Pn;	// accumulated normal impulse
		ChReal Pt;	// accumulated tangent impulse
		ChReal Pnb;	// accumulated normal impulse for position bias
		ChReal massNormal, massTangent;
		ChReal bias;
		FeaturePair feature;
	};

	struct ArbiterKey
	{
		ArbiterKey(chLiteBody* b1, chLiteBody* b2)
		{
			if (b1 < b2)
			{
				body1 = b1; body2 = b2;
			}
			else
			{
				body1 = b2; body2 = b1;
			}

		}

		chLiteBody* body1;
		chLiteBody* body2;
	};

	struct chLiteArbiter
	{
		enum { MAX_POINTS = 2 };

		chLiteArbiter(chLiteBody* b1, chLiteBody* b2);

		void Update(Contact* contacts, int numContacts);

		void PreStep(ChReal inv_dt);
		void ApplyImpulse();

		Contact contacts[MAX_POINTS];
		int numContacts;

		chLiteBody* body1;
		chLiteBody* body2;

		// combined friction
		ChReal friction;
	};

	// This is used by std::set
	inline bool operator < (const ArbiterKey& a1, const ArbiterKey& a2)
	{
		if (a1.body1 < a2.body1) return true;

		if (a1.body1 == a2.body1 && a1.body2 < a2.body2)
			return true;

		return false;
	}

	int Collide(Contact* contacts, chLiteBody* body1, chLiteBody* body2);
}

#endif
