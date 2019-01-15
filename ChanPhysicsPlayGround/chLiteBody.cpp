#include "chLiteBody.h"

Chan::chLiteBody::chLiteBody()
{
	position.Set(0.0, 0.0);
	rotation = 0.0;
	velocity.Set(0.0, 0.0);
	angularVelocity = 0.0;
	force.Set(0.0, 0.0);
	torque = 0.0;
	friction = ChReal(0.2);

	width.Set(1.0, 1.0);
	mass = Chreal_max;
	invMass = 0.0;
	I = Chreal_max;
	invI = 0.0;
}

void Chan::chLiteBody::Set(const ChVector2 & w, ChReal m)
{
	position.Set(0.0, 0.0);
	rotation = 0.0;
	velocity.Set(0.0, 0.0);
	angularVelocity = 0.0;
	force.Set(0.0, 0.0);
	torque = 0.0;
	friction = (ChReal)0.2;
	
	width = w;
	mass = m;

	if (mass < Chreal_max)
	{
		invMass = (ChReal)1.0 / mass;
		I = mass * (width.x * width.x + width.y * width.y) / (ChReal)12.0;
		invI = (ChReal)1.0 / I;
	}
	else
	{
		invMass = (ChReal)0.0;
		I = Chreal_max;
		invI = (ChReal)0.0;
	}
}


