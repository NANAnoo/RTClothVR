#pragma once

#include "FRTEnergyCondition.h"
#include "RTClothStructures.h"

class FRTStretchCondition : FClothTriangleProperties, public FRTEnergyCondition
{
public:
	FRTStretchCondition(FClothRawMesh *const mesh, int P0, int P1, int P2, float Rest_U = 1.f, float Rest_V = 1.f)
		: FClothTriangleProperties(P0, P1, P2,mesh->TexCoords[P0], mesh->TexCoords[P1], mesh->TexCoords[P2]),
			RestU(Rest_U), RestV(Rest_V)
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
	
	virtual ~FRTStretchCondition() override
	{
	}
private:
	virtual void Update(const FVector &P0, const FVector &P1, const FVector &P2, const FVector& V0, const FVector& V1, const FVector& V2) override;
	
	// energy condition:
	float C0 = 0, C1 = 0;

	// resting U V
	float RestU, RestV;

	// time derivatives of energy condition:
	float dC0dt = 0, dC1dt = 0;

	// derivatives of the energy conditions:
	FVector dC0dX[3];
	FVector dC1dX[3];
};