#include "FRTStretchCondition.h"


void FRTStretchCondition::ComputeForces(
	TArray<FVector> const& X, TArray<FVector> const& V, TArray<FVector2D> const& UV, float K, float D,
	TArray<FVector>& Forces, FRTBBSSMatrix<float>& dfdx,
	TArray<FVector>& DampingF, FRTBBSSMatrix<float>& dddx, FRTBBSSMatrix<float>& dddv)
{
	// Update triangle properties
	Update(X[V_Inx[0]], X[V_Inx[1]], X[V_Inx[2]], V[V_Inx[0]], V[V_Inx[1]], V[V_Inx[2]]);

	// update Forces
	// E = 0.5 * ( C0*C0 + C1*C1 )
	// f = -dE/dx = - (C0 dC0/dx + C1 dC1/dx)
	
	Forces[V_Inx[0]] -= K * (C0 * dC0dP0 + C1 * dC1dP0);
	Forces[V_Inx[1]] -= K * (C0 * dC0dP1 + C1 * dC1dP1);
	Forces[V_Inx[2]] -= K * (C0 * dC0dP2 + C1 * dC1dP2);

	const FRTMatrix3 CPMat_0_0 = FRTMatrix3::CrossVec(dC0dP0, dC0dP0) + FRTMatrix3::CrossVec(dC1dP0, dC1dP0);
	const FRTMatrix3 CPMat_0_1 = FRTMatrix3::CrossVec(dC0dP0, dC0dP1) + FRTMatrix3::CrossVec(dC1dP0, dC1dP1);
	const FRTMatrix3 CPMat_0_2 = FRTMatrix3::CrossVec(dC0dP0, dC0dP2) + FRTMatrix3::CrossVec(dC1dP0, dC1dP2);
	
	const FRTMatrix3 CPMat_1_0 = FRTMatrix3::CrossVec(dC0dP1, dC0dP0) + FRTMatrix3::CrossVec(dC1dP1, dC1dP0);
	const FRTMatrix3 CPMat_1_1 = FRTMatrix3::CrossVec(dC0dP1, dC0dP1) + FRTMatrix3::CrossVec(dC1dP1, dC1dP1);
	const FRTMatrix3 CPMat_1_2 = FRTMatrix3::CrossVec(dC0dP1, dC0dP2) + FRTMatrix3::CrossVec(dC1dP1, dC1dP2);
	
	const FRTMatrix3 CPMat_2_0 = FRTMatrix3::CrossVec(dC0dP2, dC0dP0) + FRTMatrix3::CrossVec(dC1dP2, dC1dP0);
	const FRTMatrix3 CPMat_2_1 = FRTMatrix3::CrossVec(dC0dP2, dC0dP1) + FRTMatrix3::CrossVec(dC1dP2, dC1dP1);
	const FRTMatrix3 CPMat_2_2 = FRTMatrix3::CrossVec(dC0dP2, dC0dP2) + FRTMatrix3::CrossVec(dC1dP2, dC1dP2);

	// First Derivative of Force, dfdx
	const FRTMatrix3 df0dP0 = -K * (C0 * d2C0dP0dP0 + C1 * d2C1dP0dP0 + CPMat_0_0);
	const FRTMatrix3 df0dP1 = -K * (C0 * d2C0dP0dP1 + C1 * d2C1dP0dP1 + CPMat_0_1);
	const FRTMatrix3 df0dP2 = -K * (C0 * d2C0dP0dP2 + C1 * d2C1dP0dP2 + CPMat_0_2);

	const FRTMatrix3 df1dP0 = -K * (C0 * d2C0dP1dP0 + C1 * d2C1dP1dP0 + CPMat_1_0);
	const FRTMatrix3 df1dP1 = -K * (C0 * d2C0dP1dP1 + C1 * d2C1dP1dP1 + CPMat_1_1);
	const FRTMatrix3 df1dP2 = -K * (C0 * d2C0dP1dP2 + C1 * d2C1dP1dP2 + CPMat_1_2);

	const FRTMatrix3 df2dP0 = -K * (C0 * d2C0dP2dP0 + C1 * d2C1dP2dP0 + CPMat_2_0);
	const FRTMatrix3 df2dP1 = -K * (C0 * d2C0dP2dP1 + C1 * d2C1dP2dP1 + CPMat_2_1);
	const FRTMatrix3 df2dP2 = -K * (C0 * d2C0dP2dP2 + C1 * d2C1dP2dP2 + CPMat_2_2);

	// fill in dfdx
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			dfdx[3 * V_Inx[0] + i][3 * V_Inx[0] + j] += df0dP0[i][j];
			dfdx[3 * V_Inx[0] + i][3 * V_Inx[1] + j] += df0dP1[i][j];
			dfdx[3 * V_Inx[0] + i][3 * V_Inx[2] + j] += df0dP2[i][j];

			dfdx[3 * V_Inx[1] + i][3 * V_Inx[0] + j] += df1dP0[i][j];
			dfdx[3 * V_Inx[1] + i][3 * V_Inx[1] + j] += df1dP1[i][j];
			dfdx[3 * V_Inx[1] + i][3 * V_Inx[2] + j] += df1dP2[i][j];

			dfdx[3 * V_Inx[2] + i][3 * V_Inx[0] + j] += df2dP0[i][j];
			dfdx[3 * V_Inx[2] + i][3 * V_Inx[1] + j] += df2dP1[i][j];
			dfdx[3 * V_Inx[2] + i][3 * V_Inx[2] + j] += df2dP2[i][j];
		}
	}

	// Compute Damping Forces
	// fd = -d * ( dC0/dt * dC0/dx + dC1/dt * dC1/dx ):
	DampingF[V_Inx[0]] = - D * (dC0dt * dC0dP0 + dC1dt * dC1dP0);
	DampingF[V_Inx[1]] = - D * (dC0dt * dC0dP1 + dC1dt * dC1dP1);
	DampingF[V_Inx[1]] = - D * (dC0dt * dC0dP2 + dC1dt * dC1dP2);

	// Compute And Setup dddv
	const FRTMatrix3 dD0dV0 = - D * CPMat_0_0;
	const FRTMatrix3 dD0dV1 = - D * CPMat_0_1;
	const FRTMatrix3 dD0dV2 = - D * CPMat_0_2;

	const FRTMatrix3 dD1dV0 = - D * CPMat_1_0;
	const FRTMatrix3 dD1dV1 = - D * CPMat_1_1;
	const FRTMatrix3 dD1dV2 = - D * CPMat_1_2;

	const FRTMatrix3 dD2dV0 = - D * CPMat_2_0;
	const FRTMatrix3 dD2dV1 = - D * CPMat_2_1;
	const FRTMatrix3 dD2dV2 = - D * CPMat_2_2;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			dddv[3 * V_Inx[0] + i][3 * V_Inx[0] + j] += dD0dV0[i][j];
			dddv[3 * V_Inx[0] + i][3 * V_Inx[1] + j] += dD0dV1[i][j];
			dddv[3 * V_Inx[0] + i][3 * V_Inx[2] + j] += dD0dV2[i][j];

			dddv[3 * V_Inx[1] + i][3 * V_Inx[0] + j] += dD1dV0[i][j];
			dddv[3 * V_Inx[1] + i][3 * V_Inx[1] + j] += dD1dV1[i][j];
			dddv[3 * V_Inx[1] + i][3 * V_Inx[2] + j] += dD1dV2[i][j];

			dddv[3 * V_Inx[2] + i][3 * V_Inx[0] + j] += dD2dV0[i][j];
			dddv[3 * V_Inx[2] + i][3 * V_Inx[1] + j] += dD2dV1[i][j];
			dddv[3 * V_Inx[2] + i][3 * V_Inx[2] + j] += dD2dV2[i][j];
		}
	}

	// Compute And Setup dddx
	const FRTMatrix3 dD0dP0 = -D * (d2C0dP0dP0 * dC0dt + d2C1dP0dP0 * dC1dt);
	const FRTMatrix3 dD1dP0 = -D * (d2C0dP1dP0 * dC0dt + d2C1dP1dP0 * dC1dt);
	const FRTMatrix3 dD2dP0 = -D * (d2C0dP2dP0 * dC0dt + d2C1dP2dP0 * dC1dt);

	const FRTMatrix3 dD0dP1 = -D * (d2C0dP0dP1 * dC0dt + d2C1dP0dP1 * dC1dt);
	const FRTMatrix3 dD1dP1 = -D * (d2C0dP1dP1 * dC0dt + d2C1dP1dP1 * dC1dt);
	const FRTMatrix3 dD2dP1 = -D * (d2C0dP2dP1 * dC0dt + d2C1dP2dP1 * dC1dt);

	const FRTMatrix3 dD0dP2 = -D * (d2C0dP0dP2 * dC0dt + d2C1dP0dP2 * dC1dt);
	const FRTMatrix3 dD1dP2 = -D * (d2C0dP1dP2 * dC0dt + d2C1dP1dP2 * dC1dt);
	const FRTMatrix3 dD2dP2 = -D * (d2C0dP2dP2 * dC0dt + d2C1dP2dP2 * dC1dt);

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			dddx[3 * V_Inx[0] + i][3 * V_Inx[0] + j] += dD0dP0[i][j];
			dddx[3 * V_Inx[0] + i][3 * V_Inx[1] + j] += dD0dP1[i][j];
			dddx[3 * V_Inx[0] + i][3 * V_Inx[2] + j] += dD0dP2[i][j];

			dddx[3 * V_Inx[1] + i][3 * V_Inx[0] + j] += dD1dP0[i][j];
			dddx[3 * V_Inx[1] + i][3 * V_Inx[1] + j] += dD1dP1[i][j];
			dddx[3 * V_Inx[1] + i][3 * V_Inx[2] + j] += dD1dP2[i][j];

			dddx[3 * V_Inx[2] + i][3 * V_Inx[0] + j] += dD2dP0[i][j];
			dddx[3 * V_Inx[2] + i][3 * V_Inx[1] + j] += dD2dP1[i][j];
			dddx[3 * V_Inx[2] + i][3 * V_Inx[2] + j] += dD2dP2[i][j];
		}
	}
}


void FRTStretchCondition::Update(const FVector& P0, const FVector& P1, const FVector& P2, const FVector& V0, const FVector& V1, const FVector& V2)
{
	FClothTriangleProperties::Update(P0, P1, P2, V0, V1, V2);
	// first derivatives of C:
	float a_wuNorm = a / wuNorm;
	float a_wvNorm = a / wvNorm;
	dC0dP0 = dwudP0 * wu * a_wuNorm;
	dC0dP1 = dwudP1 * wu * a_wuNorm;
	dC0dP2 = dwudP2 * wu * a_wuNorm;

	dC1dP0 = dwvdP0 * wv * a_wvNorm;
	dC1dP1 = dwvdP1 * wv * a_wvNorm;
	dC1dP2 = dwvdP2 * wv * a_wvNorm;

	// condition value:
	C0 = a * (wuNorm - RestU);
	C1 = a * (wvNorm - RestV);

	// time derivative of C:
	dC0dt = (dC0dP0|V0) + (dC0dP1|V1) + (dC0dP2|V2);
	dC1dt = (dC1dP0|V0) + (dC1dP1|V1) + (dC1dP2|V2);

	// second derivatives of C:
	const FRTMatrix3 wuMatrix = (a_wuNorm / (wuNorm * wuNorm)) * (FRTMatrix3::Identity() - FRTMatrix3::CrossVec(wu, wu) );
	d2C0dP0dP0 = dwudP0 * dwudP0 * wuMatrix;
	d2C0dP0dP1 = dwudP0 * dwudP1 * wuMatrix;
	d2C0dP0dP2 = dwudP0 * dwudP2 * wuMatrix;

	d2C0dP1dP0 = dwudP1 * dwudP0 * wuMatrix;
	d2C0dP1dP1 = dwudP1 * dwudP1 * wuMatrix;
	d2C0dP1dP2 = dwudP1 * dwudP2 * wuMatrix;

	d2C0dP2dP0 = dwudP2 * dwudP0 * wuMatrix;
	d2C0dP2dP1 = dwudP2 * dwudP1 * wuMatrix;
	d2C0dP2dP2 = dwudP2 * dwudP2 * wuMatrix;

	const FRTMatrix3 wvMatrix = (a_wvNorm / (wvNorm * wvNorm)) * (FRTMatrix3::Identity() - FRTMatrix3::CrossVec(wv, wv));
	d2C1dP0dP0 = dwvdP0 * dwvdP0 * wvMatrix;
	d2C1dP0dP1 = dwvdP0 * dwvdP1 * wvMatrix;
	d2C1dP0dP2 = dwvdP0 * dwvdP2 * wvMatrix;

	d2C1dP1dP0 = dwvdP1 * dwvdP0 * wvMatrix;
	d2C1dP1dP1 = dwvdP1 * dwvdP1 * wvMatrix;
	d2C1dP1dP2 = dwvdP1 * dwvdP2 * wvMatrix;

	d2C1dP2dP0 = dwvdP2 * dwvdP0 * wvMatrix;
	d2C1dP2dP1 = dwvdP2 * dwvdP1 * wvMatrix;
	d2C1dP2dP2 = dwvdP2 * dwvdP2 * wvMatrix;
}