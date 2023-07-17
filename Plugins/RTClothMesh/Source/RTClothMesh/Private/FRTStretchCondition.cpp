#include "FRTStretchCondition.h"

void FRTStretchCondition::UpdateCondition(
	TArray<FVector> const& X,
	TArray<FVector> const& V,
	TArray<FVector2D> const& UV
	)
{
	// Update triangle properties
	Update(X[V_Inx[0]], X[V_Inx[1]], X[V_Inx[2]], V[V_Inx[0]], V[V_Inx[1]], V[V_Inx[2]]);
}
void FRTStretchCondition::ComputeForces(
	float K, float D,
	TArray<FVector> &Forces, TArray<FVector> &DampingF
	)
{
	// update Forces
	// E = 0.5 * ( C0*C0 + C1*C1 )
	// f = -dE/dx = - (C0 dC0/dx + C1 dC1/dx)
	for (uint32 i = 0; i < 3; i ++)
	{
		Forces[V_Inx[i]] -= K * (C0 * dC0dX[i] + C1 * dC1dX[i]);
	}

	// Compute Damping Forces
	// fd = -d * ( dC0/dt * dC0/dx + dC1/dt * dC1/dx ):
	for (uint32 i = 0; i < 3; i ++)
	{
		DampingF[V_Inx[i]] -= D * (dC0dt * dC0dX[i] + dC1dt * dC1dX[i]);
	}
}
void FRTStretchCondition::ComputeDerivatives(
	float K, float D,
	FRTBBSSMatrix<float> &dfdx,
	FRTBBSSMatrix<float> &dddx,
	FRTBBSSMatrix<float> &dddv
)
{
	const float a_wuNorm = a / wuNorm;
	const float a_wvNorm = a / wvNorm;

	// second derivatives of the energy conditions:
	FRTMatrix3 d2C0dXX[3][3];
	FRTMatrix3 d2C1dXX[3][3];
	
	// second derivatives of C:
	const FRTMatrix3 wuMatrix = (a_wuNorm / (wuNorm * wuNorm)) * (FRTMatrix3::Identity() - FRTMatrix3::CrossVec(wu, wu) );
	const FRTMatrix3 wvMatrix = (a_wvNorm / (wvNorm * wvNorm)) * (FRTMatrix3::Identity() - FRTMatrix3::CrossVec(wv, wv));
	for (uint32 i = 0; i < 3; i ++)
	{
		for (uint32 j = 0; j < 3; j ++)
		{
			d2C0dXX[i][j] = dwudX[i] * dwudX[j] * wuMatrix;
			d2C1dXX[i][j] = dwvdX[i] * dwvdX[j] * wvMatrix;
		}
	}
	// First Derivative of Force, dfdx
	FRTMatrix3 CPMat[3][3], dfdX[3][3];
	for (uint32 i = 0; i < 3; i ++)
	{
		for (uint32 j = 0; j < 3; j ++)
		{
			CPMat[i][j] = FRTMatrix3::CrossVec(dC0dX[i], dC0dX[j]) + FRTMatrix3::CrossVec(dC1dX[i], dC1dX[j]);
			dfdX[i][j] = -K * (C0 * d2C0dXX[i][j] + C1 * d2C1dXX[i][j] + CPMat[i][j]);
		}
	}

	// fill in dfdx
	for (uint32 i = 0; i < 3; ++i)
	{
		for (uint32 j = 0; j < 3; ++j)
		{
			for (uint32 m = 0; m < 3; m ++)
			{
				for (uint32 n = 0; n < 3; n ++)
				{
					dfdx[3 * V_Inx[m] + i][3 * V_Inx[n] + j] += dfdX[m][n][i][j];
				}
			}
		}
	}

	// Compute And Setup dddv
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			for (uint32 m = 0; m < 3; m ++)
			{
				for (uint32 n = 0; n < 3; n ++)
				{
					dddv[3 * V_Inx[m] + i][3 * V_Inx[n] + j] += -D * CPMat[m][n][i][j];
				}
			}
		}
	}

	// Compute And Setup dddx
	FRTMatrix3 dDdX[3][3];
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			dDdX[i][j] = -D * (d2C0dXX[i][j] * dC0dt + d2C1dXX[i][j] * dC1dt);
		}
	}
	
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			for (uint32 m = 0; m < 3; m ++)
			{
				for (uint32 n = 0; n < 3; n ++)
				{
					dddx[3 * V_Inx[m] + i][3 * V_Inx[n] + j] += dDdX[n][m][i][j];
				}
			}
		}
	}
}

void FRTStretchCondition::Update(const FVector& P0, const FVector& P1, const FVector& P2, const FVector& V0, const FVector& V1, const FVector& V2)
{
	FClothTriangleProperties::Update(P0, P1, P2, V0, V1, V2);
	// first derivatives of C:
	const float a_wuNorm = a / wuNorm;
	const float a_wvNorm = a / wvNorm;
	const FVector wu_a_wuNorm = wu * a_wuNorm;
	const FVector wv_a_wvNorm = wv * a_wvNorm;
	for (uint32 i = 0; i < 3; i ++)
	{
		dC0dX[i] = dwudX[i] * wu_a_wuNorm;
		dC1dX[i] = dwvdX[i] * wv_a_wvNorm;
	}

	// condition value:
	C0 = a * (wuNorm - RestU);
	C1 = a * (wvNorm - RestV);

	// time derivative of C:
	dC0dt = (dC0dX[0]|V0) + (dC0dX[1]|V1) + (dC0dX[2]|V2);
	dC1dt = (dC1dX[0]|V0) + (dC1dX[1]|V1) + (dC1dX[2]|V2);
}