#include "chCollision.h"

bool Chan::AABBOverlap(const chAABB2D & a, const chAABB2D & b)
{
	if (a.max.x < b.min.x || a.min.x > b.max.x) return false;
	if (a.max.y < b.min.y || a.min.y > b.max.y) return false;
	return true;
}

bool Chan::AABBOverlap(const chAABB3D & a, const chAABB3D & b)
{
	if (a.max.x < b.min.x || a.min.x > b.max.x) return false;
	if (a.max.y < b.min.y || a.min.y > b.max.y) return false;
	if (a.max.z < b.min.z || a.min.z > b.max.z) return false;
	return true;
}

bool Chan::CircleOverlap(const chCircle2D& a, const chCircle2D & b)
{
	ChReal r = a.radius + b.radius;
	r *= r;

	ChVector2 v = a.position - b.position;
	return r < dot(v, v);
}

bool Chan::CircleOverlap(const chCircle3D & a, const chCircle3D & b)
{
	ChReal r = a.radius + b.radius;
	r *= r;

	ChVector3 v = a.position - b.position;
	return r < dot(v, v);
}
