#pragma once
#ifndef __CH_AABB_H__
#define __CH_AABB_H__

#include "chMath.h"

namespace Chan
{
	class chAABB2D
	{
	public:
		ChVector2 min;
		ChVector2 max;
	};

	class chAABB3D
	{
	public:
		ChVector3 min;
		ChVector3 max;
	};
}


#endif