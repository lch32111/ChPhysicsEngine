#pragma once

#include <DirectXMath.h>
struct PhysicsData
{
	struct GameActor
	{
		DirectX::XMMATRIX modelMatrix;
	};

	struct Joint
	{
		DirectX::XMFLOAT3 Line1[2];
		DirectX::XMFLOAT3 Line2[2];
	};

	GameActor actors[200];
	Joint joints[50];


	unsigned numActors;
	unsigned numJoints;
	unsigned iterations;
	bool useWarmStart;
	bool accumulateImpulse;
	bool positionCorrection;
};