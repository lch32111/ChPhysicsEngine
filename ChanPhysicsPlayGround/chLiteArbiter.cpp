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

		const float restitution = -0.0;
		ChReal dPn = c->massNormal * (-(1 + restitution) * vn + c->bias);

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
		
		dv = b2->velocity + Cross(b2->angularVelocity, c->r2) - b1->velocity - Cross(b1->angularVelocity, c->r1);

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
	// It's equivalent to the distance from vertex to plane in 3D.
	// If distance0 > 0 , the vertex in on the side of plane normal.
	Chan::ChReal distance0 = dot(normal, vIn[0].v) - offset;
	Chan::ChReal distance1 = dot(normal, vIn[1].v) - offset;

	// If the points are behind the plane, meaning on the side oppostite the normal,
	// So, we have to keep it. we don't need to clip it.
	if (distance0 <= Chan::ChReal(0.0)) vOut[numOut++] = vIn[0];
	if (distance1 <= Chan::ChReal(0.0)) vOut[numOut++] = vIn[1];

	// If the points are on different sides of the plane
	// We have to clip it.
	if (distance0 * distance1 < Chan::ChReal(0.0))
	{
		// Find intersection point of edge and plane
		// You will get to know why this line should be like this
		// If you draw graph on your paper.
		Chan::ChReal interp = distance0 / (distance0 - distance1);
		vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);

		// If vIn[0] is on the side of normal
		// That is, you should put the clipped information on the Out ClipVertex
		if (distance0 > Chan::ChReal(0.0))
		{
			vOut[numOut].fp = vIn[0].fp;
			vOut[numOut].fp.e.inEdge1 = clipEdge; // Reference Face's clipEdge is the inEdge of this clipped vertex
			vOut[numOut].fp.e.inEdge2 = NO_EDGE;  // Delete previous inEdge
			// So now this clip vertex consists of inEdge1(from reference facae) and outEdge2(from incident face) 
		}
		// in this case, vIn[0] is on the opposite side of normal.
		// So vIn[1] is on the side of normal. you should put the clipped information on the Out ClipVertex
		else
		{
			vOut[numOut].fp = vIn[1].fp;
			vOut[numOut].fp.e.outEdge1 = clipEdge; // Reference Face's clipedge is outedge of this cliped vertex
			vOut[numOut].fp.e.outEdge2 = NO_EDGE; // Delete previous outEdge
			// So now this clip vertex consists of inEdge2(from incident face) and outEdge1(from reference face)
		}

		++numOut;
	}

	return numOut;
}

void ComputeIncidentEdge(ClipVertex c[2], 
	const Chan::ChVector2& h, // half extents
	const Chan::ChVector2& pos,
	const Chan::ChMat22& Rot, 
	const Chan::ChVector2& normal)
{
	// The normal is from the reference box. Convert it.
	// to the incident boxe's frame and flip sign.
	Chan::ChMat22 RotT = Rot.Transpose(); // World to Local
	Chan::ChVector2 n = -(RotT * normal); // From World Normal to Local
	Chan::ChVector2 nAbs = Abs(n);

	// Box vertex and edge numbering:
	//        ^ y
	//        |
	//        e1
	//   v2 ------ v1
	//    |        |
	// e2 |        | e4  --> x
	//    |        |
	//   v3 ------ v4
	//        e3
	// nAbs.x > nAbs.y means that we have to check left or right edges on the box
	// the incident edges of incident face will be e1, e4, e3 with positive n.x
	// with negative n.x, the edges will be e1, e2, e3.
	// else if(nAbs.x < nAbs.y) means that we have to check up or down edges on the box
	// the incident edges of incident face will be e2, e1, e4 with positinve n.y.
	// with negative n.y, the edges will be e2, e3, e4.
	// 
	// You will get to know why we use Absolute Vector to check which axis to use between x-axis and y-axis
	// if you draw and write your own box on the paper.
	// The notion of in/out Edge is related to the winding of the edges.
	// The box above is specified in Counter-Clock Wise(CCW) winding.
	// So, inEdge means the first edge to one vertex, and outEdge means the second edge out of one vertex.
	// And in/outEdge1 means edges of reference face.
	// in/outEdge2 means edges of Incident face.
	
	if (nAbs.x > nAbs.y) // X-axis is incident area
	{
		// Right X-axis
		if (Chan::Sign(n.x) > Chan::ChReal(0.0))
		{
			c[0].v.Set(h.x, -h.y); // Vertex 4
			c[0].fp.e.inEdge2 = EDGE3; 
			c[0].fp.e.outEdge2 = EDGE4;

			c[1].v.Set(h.x, h.y); // Vertex 1
			c[1].fp.e.inEdge2 = EDGE4;
			c[1].fp.e.outEdge2 = EDGE1;
		}
		else // Left X-axis
		{
			c[0].v.Set(-h.x, h.y); // Vertex 2
			c[0].fp.e.inEdge2 = EDGE1;
			c[0].fp.e.outEdge2 = EDGE2;

			c[1].v.Set(-h.x, -h.y); // Vertex 3
			c[1].fp.e.inEdge2 = EDGE2;
			c[1].fp.e.outEdge2 = EDGE3;
		}
	}
	else // Y-axis is incident area
	{
		// Upper Y-axis
		if (Chan::Sign(n.y) > Chan::ChReal(0.0))
		{
			c[0].v.Set(h.x, h.y); // Vertex 1
			c[0].fp.e.inEdge2 = EDGE4;
			c[0].fp.e.outEdge2 = EDGE1;

			c[1].v.Set(-h.x, h.y); // Vertex 2
			c[1].fp.e.inEdge2 = EDGE1;
			c[1].fp.e.outEdge2 = EDGE2;
		}
		else // Lower Y-axis
		{
			c[0].v.Set(-h.x, -h.y); // Vertex 3
			c[0].fp.e.inEdge2 = EDGE2;
			c[0].fp.e.outEdge2 = EDGE3;

			c[1].v.Set(h.x, -h.y); // Vertex 4
			c[1].fp.e.inEdge2 = EDGE3;
			c[1].fp.e.outEdge2 = EDGE4;
		}
	}

	// Rotate and Translate the clip vertex to the original vertex position. (from local to world)
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
	
	/*
	 * The way of determining the normal is that, if you assume the FACE_A_X should be a least penetration axis
	 * dA is A's Local Space from A center to B center, and if its x component is over 0,
	 * The colliding direction of A with B is Right X axis.
	 * And each column of rotatio matrix, specifically the local to world matrix, means
	 * [  Vec1(Local Axis X in terms of World)   Vec2(Local Axis Y in terms of World) ].
	 * So you can use just col1 or -col1 if the normal is pointing toward left.
	 *
	 * To simplify the explanation according to the Erin Catto's presentation,
	 * In 2D, the separating axis is a face normal.
	 */
	normal = dA.x > ChReal(0.0) ? RotA.col1 : -RotA.col1;

	const ChReal relativeTol = ChReal(0.95);
	const ChReal absoluteTol = ChReal(0.01);

	// To find the least penetration axis in the consistent way,
	// favor original axis to get better float-point comparison
	// The reason why we have to find the least penetration axis, you will get to know
	// if you imagine the sitatuon, where two boxes overlaps and you have to calculate each penetration axis value manually.
	// Erin's explanation on this : https://pybullet.org/Bullet/phpBB3/viewtopic.php?f=4&t=398
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

	/*
	 * Clipping Setup : 
	 * 1. Identify Reference Face -> We already know what is reference face with least seperating axis
	 * 2. Identify Incident Face -> We will calculate the incident face with ComputeIncidentEdge() function
	 * 3. And then clip the face to the edge, using Sutherland-Hodgman clipping, using ClipSegmentToLine() function.
	 * refer necessarily to the Erin Catto's presentation for Box-Box Clipping
	 * You Should remember this diagram again
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
	 * 
	 */

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
			/*
			 * ClipVertex has the next structure
			 * struct ClipVertex
				{
					ClipVertex() { fp.value = 0; }

					Chan::ChVector2 v;
					Chan::FeaturePair fp;
				};
			 * FeaturePair has the next union structure
			 * union FeaturePair
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
			 *
			 * FeaturePair will be used to specify which edges are related to the collision area.
			 * And the value will be used to identify the old contact to accumulate the impulses.
			 * So ComputeIncidentEdge will not use "int value" member.
			 * And, ComputeIncidentEdge will fill inEdge2, outEdge2.
			 * I guess so far the 2 means the second box(B). So, We will calculate B's incident edge.
			 * In case of the least penetration axis from Body B, A will be assumed as Body B, opposite one.
			 * It will be easire for you to think of 2 as a opposite one.
			 * And 1 means the standard one.
			 */

			// Informations needed to get clip the incident edge in ClipSegmentToLine function
			sideNormal = RotA.col2;
			ChReal side = dot(posA, sideNormal);
			negSide = -side + hA.y; 
			posSide = side + hA.y;
			negEdge = EDGE3;
			posEdge = EDGE1;
			// Informations needed to get clip the incident edge in ClipSegmentToLine function

			/*
			 * ClipSegmentToLine uses Sutherland-Hodgman clipping.
			 * It means extending the edges to the plane and clipping incident edges
			 * so, the informations above for the ClipSegmentToLine function are related to edges and planes
			 * especially, negSide and posSide means offset of plane. However, in 2D, There is no plane.
			 * So, it is the line offset. But, the reason why negside = -side + hA.y is that,
			 * the negSide has the opposite normal. So, If it has the same normal with posSide,
			 * negSide offset will be side - hA.y. Since it has opposite direction, we have to negate the negSide.
			 * ClipSegmentToLine will use these offset to distinguish whether the clip vertex is on the plane(line) normal or not.
			 * If clip vertex is on the plane(line) normal, we have to clip it. Otherwise, we have to keep it, because it's begine the 
			 * plane.
			 */
			
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

	/*
	 * Now you get the incident edge from the ComputeIncidentEdge()
	 * And you also hold the information of side edges of the reference edge
	 * The reason why we get the side information is for cliipping the incident edge with side edge.
	 * It's Sutherland-Hodgman algorithm for clipping polygons.
	 */

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

		// Check if the clipped veritces are behind the reference face
		// If it is not, the vertex is not the colliding point
		if (separation <= 0)
		{
			contacts[numContacts].separation = separation;
			contacts[numContacts].normal = normal;
			// slide contact point onto reference face (easy to cull)
			// imagine the contact manifold. if the contact manifold will not be modifed to the reference face,
			// the contact resolution will work in the wrong way a little.
			// https://www.reddit.com/r/box2d/comments/aim8rm/box2d_lite_collision/
			// I did self-question and self-answer on this reddit post.
			contacts[numContacts].position = clipPoints2[i].v - separation * frontNormal;
			contacts[numContacts].feature = clipPoints2[i].fp;

			// this collision assumes that reference face is boxA. 
			// Because the incident edge is always put on in/outEdge2, boxB.
			// It means the edges will be calculated in terms of A.
			// So If the axis will be from B, we need to flip the feature edge specifying where the edge is from.
			if (axis == FACE_B_X || axis == FACE_B_Y)
				Flip(contacts[numContacts].feature);
			++numContacts;
		}
	}

	return numContacts;
}
