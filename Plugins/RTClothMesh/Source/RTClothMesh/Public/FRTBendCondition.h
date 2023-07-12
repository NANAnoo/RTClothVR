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

	virtual void ComputeForces(
		TArray<FVector> const& X, TArray<FVector> const& V,TArray<FVector2D> const& UV, float K, float D,
		TArray<FVector> &Forces, FRTBBSSMatrix<float> &dfdx,
		TArray<FVector> &DampingF,  FRTBBSSMatrix<float> &dddx,  FRTBBSSMatrix<float> &dddv
	) override;
private:
	uint32 V_idx[4];
};