#pragma once
#ifndef __CH_COLLISION_H__
#define __CH_COLLISION_H__

#include "chAABB.h"
#include "chCircle.h"

namespace Chan
{
	bool AABBOverlap(const chAABB2D& a, const chAABB2D& b);
	bool AABBOverlap(const chAABB3D& a, const chAABB3D& b);
	bool CircleOverlap(const chCircle2D& a, const chCircle2D& b);
	bool CircleOverlap(const chCircle3D& a, const chCircle3D& b);
}

#endif