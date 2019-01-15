#include "chLiteJoint.h"
#include "chLiteBody.h"
#include "chLiteWorld.h"


void Chan::chLiteJoint::Set(chLiteBody * b1, chLiteBody * b2, const ChVector2 & anchor)
{
	body1 = b1;
	body2 = b2;

	ChMat22 Rot1(body1->rotation);
	ChMat22 Rot2(body2->rotation);
	ChMat22 Rot1T = Rot1.Transpose();
	ChMat22 Rot2T = Rot2.Transpose();


	localAnchor1 = Rot1T * (anchor - body1->position);
	localAnchor2 = Rot2T * (anchor - body2->position);

	P.Set((ChReal)0.0, (ChReal)0.0);

	softness = (ChReal)0.0;
	biasFactor = (ChReal)0.2;
}

void Chan::chLiteJoint::PreStep(ChReal inv_dt)
{
	// Pre-compute anchors, mass matrix, and bias
	ChMat22 Rot1(body1->rotation);
	ChMat22 Rot2(body2->rotation);

	r1 = Rot1 * localAnchor1;
	r2 = Rot2 * localAnchor2;

	// deltaV = deltaV0 + K * impulse
	// invM = [(1/m1 + 1/m2) * eye(2) - skew(r1) * inVI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//		= [1/m1+1/m2    0] + invI1 * [r1.y * r1.y  -r1.x * r1.y] + invI2 * [r1.y*r1.y  -r1.x * r1.y]
	//		= [ 0  1/m1 +1/m2]			 [-r1.x * r1.y  r1.x * r1.x] +		   [-r1.x*r1.y + r1.x*r1.x]
	ChMat22 K1;
	K1.col1.x = body1->invMass + body2->invMass;	K1.col2.x = ChReal(0.0);
	K1.col1.y = ChReal(0.0);						K1.col2.y = body1->invMass + body2->invMass;

	ChMat22 K2;
	K2.col1.x = body1->invI * r1.y * r1.y;		K2.col1.x = -body1->invI * r1.x * r1.y;
	K2.col1.y = -body1->invI * r1.x * r1.y;		K2.col2.y = body1->invI * r1.x * r1.x;

	ChMat22 K3;
	K3.col1.x = body2->invI * r1.y * r1.y;		K3.col1.x = -body2->invI * r1.x * r1.y;
	K3.col1.y = -body2->invI * r1.x * r1.y;		K3.col2.y = body2->invI * r1.x * r1.x;

	ChMat22 K = K1 + K2 + K3;
	K.col1.x += softness;
	K.col2.y += softness;

	M = K.Invert();

	ChVector2 p1 = body1->position + r1;
	ChVector2 p2 = body2->position + r2;
	ChVector2 dp = p2 - p1;
	if (chLiteWorld::positionCorrection)
	{
		bias = -biasFactor * inv_dt * dp;
	}
	else
	{
		bias.Set(ChReal(0.0), ChReal(0.0));
	}

	if (chLiteWorld::warmStarting)
	{
		// Apply accumulated impulse
		body1->velocity -= body1->invMass * P;
		body1->angularVelocity -= body1->invI * Cross(r1, P);

		body2->velocity += body2->invMass * P;
		body2->angularVelocity += body2->invI * Cross(r1, P);
	}
	else
	{
		P.Set(ChReal(0.0), ChReal(0.0));
	}

}

void Chan::chLiteJoint::ApplyImpulse()
{
	ChVector2 dv = body2->velocity + Cross(body2->angularVelocity, r2) - body1->velocity - Cross(body1->angularVelocity, r1);

	ChVector2 impulse;

	impulse = M * (bias - dv - softness * P);

	body1->velocity -= body1->invMass * impulse;
	body1->angularVelocity -= body1->invI * Cross(r1, impulse);

	body2->velocity += body2->invMass * P;
	body2->angularVelocity += body2->invI * Cross(r1, impulse);

	P += impulse;
}
