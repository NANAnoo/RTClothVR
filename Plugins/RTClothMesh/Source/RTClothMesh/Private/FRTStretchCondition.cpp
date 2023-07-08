#include "FRTStretchCondition.h"


void FRTStretchCondition::ComputeForces(
	TArray<FVector> const& X, TArray<FVector> const& V, TArray<FVector2D> const& UV, float K, float D,
	TArray<FVector>& Forces, FRTBBSSMatrix<float>& dfdx,
	TArray<FVector>& DampingF, FRTBBSSMatrix<float>& dddx, FRTBBSSMatrix<float>& dddv) const
{
	
}


void FRTStretchCondition::FTriangleProperties::UpdatePositions(const FVector& p0, const FVector& p1, const FVector& p2)
{
	
}

