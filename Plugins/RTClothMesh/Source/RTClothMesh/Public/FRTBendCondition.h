#pragma once

#include "FRTEnergyCondition.h"
#include "RTClothStructures.h"

class FRTBendCondition : public FRTEnergyCondition
{
public:
	FRTBendCondition(uint32 P0, uint32 P1, uint32 P2, uint32 P3) :
	V_idx{P0, P1, P2, P3}
	{
		
	}

	virtual void UpdateCondition(
		TArray<FVector> const& X, TArray<FVector> const& V,TArray<FVector2D> const& UV
	) override;
	
	virtual void ComputeForces(
		 float K, float D,
		TArray<FVector> &Forces, TArray<FVector> &DampingF
	) override;
	
	virtual void ComputeDerivatives(
		float K, float D,
		FRTBBSSMatrix<float> &dfdx,
		FRTBBSSMatrix<float> &dddx,
		FRTBBSSMatrix<float> &dddv
	) override;
	
private:
	uint32 V_idx[4];
	float L = 0;
	float Theta = 0;
	float dTheta_dt = 0;
	FVector dTheta_dX0, dTheta_dX1, dTheta_dX2, dTheta_dX3;

	FVector E01, E02, E32, E31, E21;

	float c01 = 0, c02 = 0, c11 = 0, c12 = 0;

	FVector n0, n1;

	FVector b00, b01, b02, b11, b12, b13;

	float d00 = 0, d01 = 0, d02 = 0, d11 = 0, d12 = 0, d13 = 0;
};