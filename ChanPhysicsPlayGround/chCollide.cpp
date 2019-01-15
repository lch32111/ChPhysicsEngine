#include "chLiteArbiter.h"
#include "chLiteBody.h"

using namespace Chan;

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

	ChVector2 v;
	FeaturePair fp;
};

void Flip(FeaturePair& fp)
{
	Swap(fp.e.inEdge1, fp.e.inEdge2);
	Swap(fp.e.outEdge1, fp.e.outEdge2);
}

int ClipSegmentToLine(ClipVertex vOut[2], ClipVertex vIn[2],
	const ChVector2& normal, ChReal offset, char clipEdge)
{
	// Start with no output pints
	int numOut = 0;

	// Calculate the distance of end points to the line
	ChReal distance0 = dot(normal, vIn[0].v) - offset;
	ChReal distance1 = dot(normal, vIn[1].v) - offset;

	// If the points are behind the plane
	if (distance0 <= ChReal(0.0)) vOut[numOut++] = vIn[0];
	if (distance1 <= ChReal(0.0)) vOut[numOut++] = vIn[1];

	// If the points are on different sides of the plane
	if (distance0 * distance1 < ChReal(0.0))
	{
		// Find intersection point of edge and plane
		ChReal interp = distance0 / (distance0 - distance1);
		vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
		if (distance0 > ChReal(0.0))
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

static void ComputeIncidentEdge(ClipVertex c[2], const ChVector2& h, const ChVector2& pos,
	const ChMat22& Rot, const ChVector2& normal)
{
	// The normal is from the reference box. Convert it.
	// to the incident boxe's frame and flip sign.
	ChMat22 RotT = Rot.Transpose();
	ChVector2 n = -(RotT * normal);
	ChVector2 nAbs = Abs(n);

	if (nAbs.x > nAbs.y)
	{
		if (Sign(n.x) > ChReal(0.0))
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
		if (Sign(n.y) > ChReal(0.0))
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
int Collide(Contact* contacts, chLiteBody* bodyA, chLiteBody* bodyB)
{
	// Setup
	ChVector2 hA = ChReal(0.5) * bodyA->width;
	ChVector2 hB = ChReal(0.5) * bodyB->width;

	ChVector2 posA = bodyA->position;
	ChVector2 posB = bodyB->position;

	ChMat22 RotA(bodyA->rotation), RotB(bodyB->rotation);

	ChMat22 RotAT = RotA.Transpose();
	ChMat22 RotBT = RotB.Transpose();

	ChVector2 A1 = RotA.col1, a2 = RotA.col2;
	ChVector2 b1 = RotB.col1, b2 = RotB.col2;

	ChVector2 dp = posB - posA;
	ChVector2 dA = RotAT * dp;
	ChVector2 dB = RotBT * dp;

	ChMat22 C = RotAT * RotB;
	ChMat22 absC = Abs(C);
	ChMat22 absCT = absC.Transpose();

	// Box A faces (SAT)
	ChVector2 faceA = Abs(dA) - hA - absC * hB;
	if (faceA.x > ChReal(0.0) || faceA.y > ChReal(0.0))
		return 0;

	// Box B faces (SAT)
	ChVector2 faceB = Abs(dB) - absCT * hA - hB;
	if (faceB.x > 0.0f || faceB.y > 0.0f)
		return 0;

	// Find best axis
	Axis axis;
	ChReal separation;
	ChVector2 normal;

	// Box A faces
	axis = FACE_A_X;
	separation = faceA.x;
	normal = dA.x > ChReal(0.0) ? RotA.col1 : -RotA.col1;

	const ChReal relativeTol = ChReal(0.95);
	const ChReal absoluteTol = ChReal(0.01);

	if (faceA.y > relativeTol * separation + absoluteTol * hA.y)
	{
		axis = FACE_A_Y;
		separation = faceA.y;
		normal = dA.y > ChReal(0.0) ? RotA.col2 : -RotA.col2;
	}

	// Box B faces
	if (faceB.x > relativeTol * separation + absoluteTol * hB.x)
	{
		axis = FACE_B_X;
		separation = faceB.x;
		normal = dB.x > ChReal(0.0) ? RotB.col1 : -RotB.col1;
	}

	if (faceB.y > relativeTol * separation + absoluteTol * hB.y)
	{
		axis = FACE_B_Y;
		separation = faceB.y;
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