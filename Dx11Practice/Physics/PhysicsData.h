#pragma once

#include <DirectXMath.h>
struct PhysicsData
{
	struct GameActor
	{
		DirectX::XMMATRIX modelMatrix;
	};

	GameActor actors[200];
	unsigned numActors;
	unsigned iterations;
	bool useWarmStart;
	bool accumulateImpulse;
	bool positionCorrection;
};