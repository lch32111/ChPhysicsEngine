#pragma once
#ifndef __CH_MATH_H__
#define __CH_MATH_H__

#include <math.h>
#include <assert.h>
#include <stdlib.h>
#include <float.h>

namespace Chan
{
	

#define CH_MATH_PRECISION 1

#if CH_MATH_PRECISION == 1

	typedef float ChReal;
#define ChReal_sqrt sqrtf
#define ChReal_cos cosf
#define ChReal_sin sinf
#define Chreal_max FLT_MAX
#elif
	typedef double ChReal;
#define ChReal_sqrt std::sqrt
#define ChReal_cos std::cos
#define ChReal_sin std::sin
#define Chreal_max DBL_MAX
#endif

	class ChVector2
	{
	public:
		ChVector2() { }
		ChVector2(ChReal _x, ChReal _y) 
			: x(_x), y(_y) { }

		void Set(ChReal x_, ChReal y_) { x = x_; y = y_; }

		ChVector2 operator -() { return ChVector2(-x, -y); }
		
		void operator += (const ChVector2& v)
		{
			x += v.x; y += v.y;
		}

		void operator -= (const ChVector2& v)
		{
			x -= v.x; y -= v.y;
		}

		void operator *= (const ChVector2& v)
		{
			x *= v.x; y *= v.y;
		}

		float Length() const
		{
			return ChReal_sqrt(x * x + y * y);
		}


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

	class ChMat22
	{
	public:
		ChMat22() { }
		ChMat22(ChReal angle)
		{
			ChReal c = ChReal_cos(angle), s = ChReal_sin(angle);
			col1.x = c; col2.x = -s;
			col1.y = s; col2.y = c;
		}

		ChMat22(const ChVector2& col1, const ChVector2& col2) : col1(col1), col2(col2) { }

		ChMat22 Transpose() const
		{
			return ChMat22(ChVector2(col1.x, col2.x), ChVector2(col1.y, col2.y));
		}

		ChMat22 Invert() const
		{
			ChReal a = col1.x, b = col2.x, c = col1.y, d = col2.y;
			ChMat22 B;
			ChReal det = a * d - b * c;
			assert(det != 0.0f);
			det = ChReal(1.0) / det;
			B.col1.x = det * d;		B.col2.x = -det * b;
			B.col1.y = -det * c;	B.col2.y = det * a;
			return B;
		}


		ChVector2 col1, col2;
	};

	inline ChReal dot(const ChVector2& a, const ChVector2& b)
	{
		return a.x * b.x + a.y * b.y;
	}

	inline ChReal Cross(const ChVector2& a, const ChVector2& b)
	{
		return a.x * b.y - a.y * b.x;
	}

	inline ChVector2 Cross(const ChVector2& a, ChReal s)
	{
		return ChVector2(s * a.y, -s * a.x);
	}

	inline ChVector2 Cross(ChReal s, const ChVector2& a)
	{
		return ChVector2(-s * a.y, s * a.x);
	}

	inline ChReal dot(const ChVector3& a, const ChVector3& b)
	{
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	inline ChVector2 operator * (const ChMat22& A, const ChVector2& v)
	{
		return ChVector2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
	}

	inline ChVector2 operator + (const ChVector2& a, const ChVector2& b)
	{
		return ChVector2(a.x + b.x, a.y + b.y);
	}

	inline ChVector2 operator - (const ChVector2& a, const ChVector2& b)
	{
		return ChVector2(a.x - b.x, a.y - b.y);
	}

	inline ChVector2 operator * (const ChVector2& a, const ChVector2& b)
	{
		return ChVector2(a.x * b.x, a.y * b.y);
	}

	inline ChVector2 operator * (ChReal s, const ChVector2& v)
	{
		return ChVector2(s * v.x, s * v.y);
	}

	inline ChMat22 operator + (const ChMat22& A, const ChMat22& B)
	{
		return ChMat22(A.col1 + B.col1, A.col2 + B.col2);
	}

	inline ChMat22 operator * (const ChMat22& A, const ChMat22& B)
	{
		return ChMat22(A * B.col1, A * B.col2);
	}

	inline ChVector3 operator - (const ChVector3& a, const ChVector3& b)
	{
		return ChVector3(a.x - b.x, a.y - b.y, a.z - b.z);
	}

	inline ChReal Abs(ChReal a)
	{
		return a > ChReal(0.0) ? a : -a;
	}

	inline ChVector2 Abs(const ChVector2& a)
	{
		return ChVector2(Abs(a.x), Abs(a.y));
	}

	inline ChMat22 Abs(const ChMat22& A)
	{
		return ChMat22(Abs(A.col1), Abs(A.col2));
	}

	inline ChReal Sign(ChReal x)
	{
		return x < ChReal(0.0) ? ChReal(-1.0) : ChReal(1.0);
	}

	inline ChReal Min(ChReal a, ChReal b)
	{
		return a < b ? a : b;
	}

	inline ChReal Max(ChReal a, ChReal b)
	{
		return a > b ? a : b;
	}

	inline ChReal Clamp(ChReal a, ChReal low, ChReal high)
	{
		return Max(low, Min(a, high));
	}

	template<typename T> inline void Swap(T& a, T& b)
	{
		T tmp = a;
		a = b;
		b = tmp;
	}

	// Random number in range [-1, 1]
	inline ChReal Random()
	{
		ChReal r = (ChReal)rand();
		r /= RAND_MAX;
		r = ChReal(2.0) * r - ChReal(1.0);
		return r;
	}

	inline ChReal Random(ChReal lo, ChReal hi)
	{
		ChReal r = (ChReal)rand();
		r /= RAND_MAX;
		r = (hi - lo) * r + lo;
		return r;
	}
}

#endif
