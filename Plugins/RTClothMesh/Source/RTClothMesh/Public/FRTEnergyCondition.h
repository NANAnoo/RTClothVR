#pragma once

#include "RTClothStructures.h"
#include "Math/FRTSparseMatrix.h"

class FRTEnergyCondition
{
public:
	virtual ~FRTEnergyCondition() {}

	// virtual void ComputeForces(
	// 	TArray<FVector> const& X, TArray<FVector> const& V,TArray<FVector2D> const& UV, float K, float D,
	// 	TArray<FVector> &Forces, FRTBBSSMatrix<float> &dfdx,
	// 	TArray<FVector> &DampingF,  FRTBBSSMatrix<float> &dddx,  FRTBBSSMatrix<float> &dddv
	// ) = 0;
	virtual void UpdateCondition(TArray<FVector> const& X, TArray<FVector> const& V,TArray<FVector2D> const& UV) = 0;
	virtual void ComputeForces(
		float K, float D,
		TArray<FVector> &Forces, TArray<FVector> &DampingF
	) = 0;
	virtual void ComputeDerivatives(
		float K, float D,
		FRTBBSSMatrix<float> &dfdx,
		FRTBBSSMatrix<float> &dddx,
		FRTBBSSMatrix<float> &dddv
	) = 0;
};