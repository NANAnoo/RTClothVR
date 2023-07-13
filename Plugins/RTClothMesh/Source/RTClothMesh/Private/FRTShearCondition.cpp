#include "FRTShearCondition.h"

void FRTShearCondition::ComputeForces(
		TArray<FVector> const& X, TArray<FVector> const& V,TArray<FVector2D> const& UV, float K, float D,
		TArray<FVector> &Forces, FRTBBSSMatrix<float> &dfdx,
		TArray<FVector> &DampingF,  FRTBBSSMatrix<float> &dddx,  FRTBBSSMatrix<float> &dddv
	)
{
	// Update triangle properties
	Update(X[V_Inx[0]], X[V_Inx[1]], X[V_Inx[2]], V[V_Inx[0]], V[V_Inx[1]], V[V_Inx[2]]);

	// Calculate Forces
	// E = 0.5 * C * C
	// f = -dE/dx = - C dC/dx
	for (uint32 i = 0; i < 3; i ++)
	{
		Forces[V_Inx[i]] -= (K * C) * dCdX[i];
	}

	// compute dfdx
	FRTMatrix3 dC2dX[3][3], dFdX[3][3];
	for (uint32 i = 0; i < 3; i ++)
	{
		for (uint32 j = 0; j < 3; j ++)
		{
			dC2dX[i][j] = FRTMatrix3::CrossVec(dCdX[i], dCdX[j]);
			dFdX[i][j] = dC2dX[i][j] + C * d2CdXX[i][j];
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
					dfdx[3 * V_Inx[m] + i][3 * V_Inx[n] + j] += - K * dFdX[m][n][i][j];
				}
			}
		}
	}

	// fd = -d * dC/dt * dC/dx:
	for (uint32 i = 0; i < 3; i ++)
	{
		Forces[V_Inx[i]] -= (D * dCdt) * dCdX[i];
	}

	// dddv_ij = - d * dCdx_i * dCdx_j
	for (uint32 i = 0; i < 3; ++i)
	{
		for (uint32 j = 0; j < 3; ++j)
		{
			for (uint32 m = 0; m < 3; m ++)
			{
				for (uint32 n = 0; n < 3; n ++)
				{
					dddv[3 * V_Inx[m] + i][3 * V_Inx[n] + j] -= D * dC2dX[m][n][i][j];
				}
			}
		}
	}

	// dddx = kd * d2c/dXi_dXj * dc/dt:
	for (uint32 i = 0; i < 3; ++i)
	{
		for (uint32 j = 0; j < 3; ++j)
		{
			for (uint32 m = 0; m < 3; m ++)
			{
				for (uint32 n = 0; n < 3; n ++)
				{
					dddx[3 * V_Inx[m] + i][3 * V_Inx[n] + j] -= D * dCdt * d2CdXX[n][m][i][j];
				}
			}
		}
	}
}


void FRTShearCondition::Update(
	const FVector &P0, const FVector &P1, const FVector &P2,
	const FVector& V0, const FVector& V1, const FVector& V2)
{
	FClothTriangleProperties::Update(P0, P1, P2, V0, V1, V2);
	C = a * (wu | wv);

	for (uint32 i = 0; i < 3; i ++)
	{
		dCdX[i] = a * (dwudXScalar[i] * wv + dwvdXScalar[i] * wu);
	}

	dCdt = (dCdX[0]|V0) + (dCdX[1]|V1) + (dCdX[2]|V2);
	
	for (uint32 i = 0; i < 3; i ++)
	{
		for (uint32 j = 0; j < 3; j ++)
		{
			d2CdXX[i][j] = FRTMatrix3::Diag(a * (dwudXScalar[i] * dwvdXScalar[j] + dwvdXScalar[i] * dwudXScalar[j]));
		}
	}
}
