#include "chLiteArbiter.h"
#include "chLiteBody.h"
#include "chLiteWorld.h"

#include <iostream>
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

void Chan::chLiteArbiter::Update(Contact * newContacts, int numNewContacts)
{
	Contact mergedContacts[2];
	for (int i = 0; i < numNewContacts; ++i)
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
			if (chLiteWorld::warmStarting)
			{
				c->Pn = cOld->Pn;
				c->Pt = cOld->Pt;
				c->Pnb = cOld->Pnb;
			}
			else
			{
				c->Pn = ChReal(0.0);
				c->Pt = ChReal(0.0);
				c->Pnb = ChReal(0.0);
			}
			
		}
		else
		{
			mergedContacts[i] = newContacts[i];
		}
	}

	for (int i = 0; i < numNewContacts; ++i)
		contacts[i] = mergedContacts[i];

	numContacts = numNewContacts;
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
		kTangent += body1->invI * (dot(r1, r1) - rt1 * rt1) + body2->invI * (dot(r2, r2) - rt2 * rt2);
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

// Box vertex and edge numbering:
//
//        ^ y
//        |
//        e1
//   v2 ------ v1
//    |        |
// e2 |        | e4  --> x
//    |        |
//   v3 ------ v4
//        e3

enum Axis
{
	FACE_A_X,
	FACE_A_Y,
	FACE_B_X,
	FACE_B_Y
};

enum EdgeNumbers
{
	NO_EDGE = 0,
	EDGE1,
	EDGE2,
	EDGE3,
	EDGE4
};

struct ClipVertex
{
	ClipVertex() { fp.value = 0; }

	Chan::ChVector2 v;
	Chan::FeaturePair fp;
};

inline void Flip(Chan::FeaturePair& fp)
{
	Chan::Swap(fp.e.inEdge1, fp.e.inEdge2);
	Chan::Swap(fp.e.outEdge1, fp.e.outEdge2);
}

int ClipSegmentToLine(ClipVertex vOut[2], ClipVertex vIn[2],
	const Chan::ChVector2& normal, Chan::ChReal offset, char clipEdge)
{
	// Start with no output pints
	int numOut = 0;

	// Calculate the distance of end points to the line
	Chan::ChReal distance0 = dot(normal, vIn[0].v) - offset;
	Chan::ChReal distance1 = dot(normal, vIn[1].v) - offset;

	// If the points are behind the plane
	if (distance0 <= Chan::ChReal(0.0)) vOut[numOut++] = vIn[0];
	if (distance1 <= Chan::ChReal(0.0)) vOut[numOut++] = vIn[1];

	// If the points are on different sides of the plane
	if (distance0 * distance1 < Chan::ChReal(0.0))
	{
		// Find intersection point of edge and plane
		Chan::ChReal interp = distance0 / (distance0 - distance1);
		vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
		if (distance0 > Chan::ChReal(0.0))
		{
			vOut[numOut].fp = vIn[0].fp;
			vOut[numOut].fp.e.inEdge1 = clipEdge;
			vOut[numOut].fp.e.inEdge2 = NO_EDGE;
		}
		else
		{
			vOut[numOut].fp = vIn[1].fp;
			vOut[numOut].fp.e.outEdge1 = clipEdge;
			vOut[numOut].fp.e.outEdge2 = NO_EDGE;

		}

		++numOut;
	}

	return numOut;
}

static void ComputeIncidentEdge(ClipVertex c[2], const Chan::ChVector2& h, const Chan::ChVector2& pos,
	const Chan::ChMat22& Rot, const Chan::ChVector2& normal)
{
	// The normal is from the reference box. Convert it.
	// to the incident boxe's frame and flip sign.
	Chan::ChMat22 RotT = Rot.Transpose();
	Chan::ChVector2 n = -(RotT * normal);
	Chan::ChVector2 nAbs = Abs(n);

	if (nAbs.x > nAbs.y)
	{
		if (Chan::Sign(n.x) > Chan::ChReal(0.0))
		{
			c[0].v.Set(h.x, -h.y);
			c[0].fp.e.inEdge2 = EDGE3;
			c[0].fp.e.outEdge2 = EDGE4;

			c[1].v.Set(h.x, h.y);
			c[1].fp.e.inEdge2 = EDGE4;
			c[1].fp.e.outEdge2 = EDGE1;
		}
		else
		{
			c[0].v.Set(-h.x, h.y);
			c[0].fp.e.inEdge2 = EDGE1;
			c[0].fp.e.outEdge2 = EDGE2;

			c[1].v.Set(-h.x, -h.y);
			c[1].fp.e.inEdge2 = EDGE2;
			c[1].fp.e.outEdge2 = EDGE3;
		}
	}
	else
	{
		if (Chan::Sign(n.y) > Chan::ChReal(0.0))
		{
			c[0].v.Set(h.x, h.y);
			c[0].fp.e.inEdge2 = EDGE4;
			c[0].fp.e.outEdge2 = EDGE1;

			c[1].v.Set(-h.x, h.y);
			c[1].fp.e.inEdge2 = EDGE1;
			c[1].fp.e.outEdge2 = EDGE2;
		}
		else
		{
			c[0].v.Set(-h.x, -h.y);
			c[0].fp.e.inEdge2 = EDGE2;
			c[0].fp.e.outEdge2 = EDGE3;

			c[1].v.Set(h.x, -h.y);
			c[1].fp.e.inEdge2 = EDGE3;
			c[1].fp.e.outEdge2 = EDGE4;
		}
	}

	c[0].v = pos + Rot * c[0].v;
	c[1].v = pos + Rot * c[1].v;
}

// The normal points from A to B
int Chan::Collide(Contact* contacts, chLiteBody* bodyA, chLiteBody* bodyB)
{
	// Setup
	Chan::ChVector2 hA = Chan::ChReal(0.5) * bodyA->width;
	Chan::ChVector2 hB = Chan::ChReal(0.5) * bodyB->width;

	ChVector2 posA = bodyA->position;
	ChVector2 posB = bodyB->position;

	// Local to World Rotation Transform
	ChMat22 RotA(bodyA->rotation), RotB(bodyB->rotation);

	// World to Local Transform
	ChMat22 RotAT = RotA.Transpose();
	ChMat22 RotBT = RotB.Transpose();

	ChVector2 dp = posB - posA;
	ChVector2 dA = RotAT * dp;
	ChVector2 dB = RotBT * dp;

	ChMat22 C = RotAT * RotB;

	// RotAT(World to Local A) * RotB (Local to World B)
	ChMat22 absC = Abs(C);

	// Transpose(absC) = RotB^t * RotAT^t
	// = RotBT(World to LocalB) * RotA (Local to World)
	ChMat22 absCT = absC.Transpose();

	// Box A faces (SAT)
	ChVector2 faceA = Abs(dA) - hA - absC * hB;
	if (faceA.x > ChReal(0.0) || faceA.y > ChReal(0.0))
		return 0;

	// Box B faces (SAT)
	ChVector2 faceB = Abs(dB) - absCT * hA - hB;
	if (faceB.x > ChReal(0.0) || faceB.y > ChReal(0.0))
		return 0;

	// Find best axis
	Axis axis;
	ChReal t_separation;
	ChVector2 normal;

	// Box A faces
	axis = FACE_A_X;
	t_separation = faceA.x;
	normal = dA.x > ChReal(0.0) ? RotA.col1 : -RotA.col1;

	const ChReal relativeTol = ChReal(0.95);
	const ChReal absoluteTol = ChReal(0.01);

	// favor previous axis, taking out more value of it.
	if (faceA.y > relativeTol * t_separation + absoluteTol * hA.y)
	{
		axis = FACE_A_Y;
		t_separation = faceA.y;
		normal = dA.y > ChReal(0.0) ? RotA.col2 : -RotA.col2;
	}

	// Box B faces
	if (faceB.x > relativeTol * t_separation + absoluteTol * hB.x)
	{
		axis = FACE_B_X;
		t_separation = faceB.x;
		normal = dB.x > ChReal(0.0) ? RotB.col1 : -RotB.col1;
	}

	if (faceB.y > relativeTol * t_separation + absoluteTol * hB.y)
	{
		axis = FACE_B_Y;
		t_separation = faceB.y;
		normal = dB.y > ChReal(0.0) ? RotB.col2 : -RotB.col2;
	}

	// Setup clipping plane data based on the separating axis
	ChVector2 frontNormal, sideNormal;
	ClipVertex incidentEdge[2];
	ChReal front, negSide, posSide;
	char negEdge, posEdge;

	// compute the clipping lines and the line segment to be clipped.
	switch (axis)
	{
	case FACE_A_X:
		{
			frontNormal = normal;
			front = dot(posA, frontNormal) + hA.x;
			sideNormal = RotA.col2;
			ChReal side = dot(posA, sideNormal);
			negSide = -side + hA.y;
			posSide = side + hA.y;
			negEdge = EDGE3;
			posEdge = EDGE1;
			ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
		}
		break;
	case FACE_A_Y:
		{
			frontNormal = normal;
			front = dot(posA, frontNormal) + hA.y;
			sideNormal = RotA.col1;
			ChReal side = dot(posA, sideNormal);
			negSide = -side + hA.x;
			posSide = side + hA.x;
			negEdge = EDGE2;
			posEdge = EDGE4;
			ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
		}
		break;
	case FACE_B_X:
		{
			frontNormal = -normal;
			front = dot(posB, frontNormal) + hB.x;
			sideNormal = RotB.col2;
			ChReal side = dot(posB, sideNormal);
			negSide = -side + hB.y;
			posSide = side + hB.y;
			negEdge = EDGE3;
			posEdge = EDGE1;
			ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
		}
		break;
	case FACE_B_Y:
		{
			frontNormal = -normal;
			front = dot(posB, frontNormal) + hB.y;
			sideNormal = RotB.col1;
			ChReal side = dot(posB, sideNormal);
			negSide = -side + hB.x;
			posSide = side + hB.x;
			negEdge = EDGE2;
			posEdge = EDGE4;
			ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
		}
		break;
	}

	// clip other face with 5 box planes (1 face plane, 4 edge plane)
	ClipVertex clipPoints1[2];
	ClipVertex clipPoints2[2];
	int np;

	// Clip to box side 1
	np = ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, negSide, negEdge);

	if (np < 2) return 0;

	// Clip to negative box side 1
	np = ClipSegmentToLine(clipPoints2, clipPoints1, sideNormal, posSide, posEdge);

	if (np < 2) return 0;

	// Now clipoints2 contains the clipping points.
	// Due to roundoff, it is possible that clipping removes all points.

	int numContacts = 0;
	for (int i = 0; i < 2; ++i)
	{
		ChReal separation = dot(frontNormal, clipPoints2[i].v) - front;

		if (separation <= 0)
		{
			contacts[numContacts].separation = separation;
			contacts[numContacts].normal = normal;
			// slide contact point onto reference face (easy to cull)
			contacts[numContacts].position = clipPoints2[i].v - separation * frontNormal;
			contacts[numContacts].feature = clipPoints2[i].fp;
			if (axis == FACE_B_X || axis == FACE_B_Y)
				Flip(contacts[numContacts].feature);
			++numContacts;
		}
	}

	return numContacts;
}