#include "chLiteArbiter.h"
#include "chLiteBody.h"
#include "chLiteWorld.h"

Chan::chLiteArbiter::chLiteArbiter(chLiteBody * b1, chLiteBody * b2)
{
	if (b1 < b2)
	{
		body1 = b1;
		body2 = b2;
	}
	else
	{
		body1 = b2;
		body2 = b1;
	}

	numContacts = Collide(contacts, body1, body2);

	friction = ChReal_sqrt(body1->friction * body2->friction);
}

void Chan::chLiteArbiter::Update(Contact * newContacts, int numContacts)
{
	Contact mergedContacts[2];
	for (int i = 0; i < numContacts; ++i)
	{
		Contact* cNew = newContacts + i;
		int k = -1;
		for (int j = 0; j < numContacts; ++j)
		{
			Contact* cOld = contacts + j;
			if (cNew->feature.value == cOld->feature.value)
			{
				k = j;
				break;
			}
		}

		if (k > -1)
		{
			Contact* c = mergedContacts + i;
			Contact* cOld = contacts + k;
			*c = *cNew;
			
		}
		else
		{
			mergedContacts[i] = newContacts[i];
		}
	}
}

void Chan::chLiteArbiter::PreStep(ChReal inv_dt)
{
	const ChReal k_allowedPenetration = (ChReal)0.01;
	ChReal k_biasFactor = chLiteWorld::positionCorrection ? ChReal(0.2) : ChReal(0.0);

	for (int i = 0; i < numContacts; ++i)
	{
		Contact* c = contacts + i;

		ChVector2 r1 = c->position - body1->position;
		ChVector2 r2 = c->position - body2->position;

		// Precompute normal mass, tagent mass, and bias.
		ChReal rn1 = dot(r1, c->normal);
		ChReal rn2 = dot(r2, c->normal);
		ChReal kNormal = body1->invMass + body2->invMass;
		kNormal += body1->invI * (dot(r1, r1) - rn1 * rn1) + body2->invI * (dot(r2, r2) - rn2 * rn2);
		c->massNormal = ChReal(1.0) / kNormal;

		ChVector2 tangent = Cross(c->normal, ChReal(1.0));
		ChReal rt1 = dot(r1, tangent);
		ChReal rt2 = dot(r2, tangent);
		ChReal kTangent = body1->invMass + body2->invMass;
		kTangent = body1->invI * (dot(r1, r1) - rt1 * rt1) + body2->invI * (dot(r2, r2) - rt2 * rt2);
		c->massTangent = ChReal(1.0) / kTangent;

		c->bias = -k_biasFactor * inv_dt * Min(ChReal(0.0), c->separation + k_allowedPenetration);

		if (chLiteWorld::accumulateImpulses)
		{
			// Apply normal + friction impulse
			ChVector2 P = c->Pn * c->normal + c->Pt * tangent;

			body1->velocity -= body1->invMass * P;
			body1->angularVelocity -= body1->invI * Cross(r1, P);

			body2->velocity += body2->invMass * P;
			body2->angularVelocity += body2->invI * Cross(r2, P);
		}
	}
}

void Chan::chLiteArbiter::ApplyImpulse()
{
	chLiteBody* b1 = body1;
	chLiteBody* b2 = body2;

	for (int i = 0; i < numContacts; ++i)
	{
		Contact* c = contacts + i;
		c->r1 = c->position - b1->position;
		c->r2 = c->position - b2->position;

		// Relative velocity at contact
		ChVector2 dv = b2->velocity + Cross(b2->angularVelocity, c->r2) 
					- b1->velocity - Cross(b1->angularVelocity, c->r1);

		// Compute normal impulse
		ChReal vn = dot(dv, c->normal);

		ChReal dPn = c->massNormal * (-vn + c->bias);

		if (chLiteWorld::accumulateImpulses)
		{
			// Clamp the accmulated impulse
			ChReal Pn0 = c->Pn;
			c->Pn = Max(Pn0 + dPn, ChReal(0.0));
			dPn = c->Pn - Pn0;
		}
		else
		{
			dPn = Max(dPn, ChReal(0.0));
		}

		// Apply contact impulse
		ChVector2 Pn = dPn * c->normal;

		b1->velocity -= b1->invMass * Pn;
		b1->angularVelocity -= b1->invI * Cross(c->r1, Pn);

		b2->velocity += b2->invMass * Pn;
		b2->angularVelocity += b2->invI * Cross(c->r2, Pn);

		ChVector2 tangent = Cross(c->normal, ChReal(1.0));
		ChReal vt = dot(dv, tangent);
		ChReal dPt = c->massTangent * (-vt);

		if (chLiteWorld::accumulateImpulses)
		{
			// Compute friction impulse
			ChReal maxPt = friction * c->Pn;

			// Clamp friction
			ChReal oldTangentImpulse = c->Pt;
			c->Pt = Clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
			dPt = c->Pt - oldTangentImpulse;
		}
		else
		{
			ChReal maxPt = friction * dPn;
			dPt = Clamp(dPt, -maxPt, maxPt);
		}

		// Apply contact impulse
		ChVector2 Pt = dPt * tangent;
		
		b1->velocity -= b1->invMass * Pt;
		b1->angularVelocity -= b1->invI * Cross(c->r1, Pt);

		b2->velocity += b2->invMass * Pt;
		b2->angularVelocity += b2->invI * Cross(c->r2, Pt);
	}
}