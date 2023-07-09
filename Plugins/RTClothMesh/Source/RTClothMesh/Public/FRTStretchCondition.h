#pragma once

#include "FRTEnergyCondition.h"
#include "RTClothStructures.h"

class FRTStretchCondition : FClothTriangleProperties, public FRTEnergyCondition
{
public:
	FRTStretchCondition(FClothRawMesh *const mesh, uint32 P0, uint32 P1, uint32 P2, float Rest_U = 1.f, float Rest_V = 1.f)
		: FClothTriangleProperties(mesh->TexCoords[P0], mesh->TexCoords[P1], mesh->TexCoords[P2]),
			RestU(Rest_U), RestV(Rest_V),
			V_Inx{P0, P1, P2}
	{
	}
	virtual void ComputeForces(
		TArray<FVector> const& X, TArray<FVector> const& V,TArray<FVector2D> const& UV, float K, float D,
		TArray<FVector> &Forces, FRTBBSSMatrix<float> &dfdx,
		TArray<FVector> &DampingF,  FRTBBSSMatrix<float> &dddx,  FRTBBSSMatrix<float> &dddv
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
		FVector dC0dP0, dC0dP1, dC0dP2;
		FVector dC1dP0, dC1dP1, dC1dP2;
	
		// second derivatives of the energy conditions:
		FRTMatrix3 d2C0dP0dP0, d2C0dP0dP1, d2C0dP0dP2;
		FRTMatrix3 d2C0dP1dP0, d2C0dP1dP1, d2C0dP1dP2;
		FRTMatrix3 d2C0dP2dP0, d2C0dP2dP1, d2C0dP2dP2;
	
		FRTMatrix3 d2C1dP0dP0, d2C1dP0dP1, d2C1dP0dP2;
		FRTMatrix3 d2C1dP1dP0, d2C1dP1dP1, d2C1dP1dP2;
		FRTMatrix3 d2C1dP2dP0, d2C1dP2dP1, d2C1dP2dP2;
	//FTriangleProperties TriProp;
	// vertex indices
	uint32 V_Inx[3];
};