#pragma once
#ifndef __CH_CIRCLE_H__
#define __CH_CIRCLE_H__

#include "chMath.h"

namespace Chan
{
	class chCircle2D
	{
	public:
		ChVector2 position;
		ChReal radius;
	};

	class chCircle3D
	{
	public:
		ChVector3 position;
		ChReal radius;
	};
}

#endif
