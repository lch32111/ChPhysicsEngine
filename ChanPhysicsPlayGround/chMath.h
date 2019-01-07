#pragma once
#ifndef __CH_MATH_H__
#define __CH_MATH_H__

#include <cmath>

namespace Chan
{
	

#define CH_MATH_PRECISION 1

#if CH_MATH_PRECISION == 1
	typedef float ChReal;
#define ChReal_sqrt std::sqrtf
#elif
	typedef double ChReal;
#define ChReal_sqrt std::sqrt
#endif

	class ChVector2
	{
	public:
		ChVector2() { }
		ChVector2(ChReal _x, ChReal _y) 
			: x(_x), y(_y) { }

		ChReal x;
		ChReal y;
	};

	class ChVector3
	{
	public:
		ChVector3() { }
		ChVector3(ChReal _x, ChReal _y, ChReal _z) 
			: x(_x), y(_y), z(_z) { }

		ChReal x;
		ChReal y;
		ChReal z;
	};

	inline ChVector2 operator-(const ChVector2& v1, const ChVector2& v2)
	{
		return ChVector2(v1.x - v2.x, v1.y - v2.y);
	}

	inline ChVector3 operator-(const ChVector3& v1, const ChVector3& v2)
	{
		return ChVector3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
	}

	inline ChReal dot(const ChVector2& a, const ChVector2& b)
	{
		return a.x * b.x + a.y * b.y ;
	}

	inline ChReal dot(const ChVector3& a, const ChVector3& b)
	{
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	inline ChReal distance(const ChVector2& a, const ChVector2& b)
	{
		return ChReal_sqrt((a.x - b.x) * (a.x - b.x) +
			(a.y - b.y) * (a.y - b.y));
	}
	
	inline ChReal distance(const ChVector3& a, const ChVector3& b)
	{
		return ChReal_sqrt((a.x - b.x) * (a.x - b.x) + 
							(a.y - b.y) * (a.y - b.y) + 
							(a.z - b.z) * (a.z - b.z));
	}
}

#endif
