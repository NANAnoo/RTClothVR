#pragma once

#include "FRTEnergyCondition.h"
#include "RTClothStructures.h"

class FRTShearCondition : FClothTriangleProperties, public FRTEnergyCondition
{
public:
	FRTShearCondition(FClothRawMesh *const mesh, uint32 P0, uint32 P1, uint32 P2)
		: FClothTriangleProperties(mesh->TexCoords[P0], mesh->TexCoords[P1], mesh->TexCoords[P2]),
			V_Inx{P0, P1, P2}
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
	virtual void Update(const FVector &P0, const FVector &P1, const FVector &P2, const FVector& V0, const FVector& V1, const FVector& V2) override;
private:
	// C condition:
	float C = 0;

	// time derivative of C:
	float dCdt = 0;

	// dcdx
	FVector dCdX[3];
	
	const uint32 V_Inx[3];
};