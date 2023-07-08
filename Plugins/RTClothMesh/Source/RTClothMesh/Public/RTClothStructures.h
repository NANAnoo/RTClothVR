#pragma once

#include "Math/FRTMatrix.h"

struct FClothRawMesh
{
	FClothRawMesh(){}
	// disable copy
	FClothRawMesh(FClothRawMesh &) = delete;
	FClothRawMesh operator=(FClothRawMesh &) = delete;
	
	// enable move
	FClothRawMesh (FClothRawMesh &&Other) noexcept
	{
		*this = std::move(Other);
	}
	
	FClothRawMesh& operator=(FClothRawMesh &&Other) noexcept
	{
		Positions = std::move(Other.Positions);
		TexCoords = std::move(Other.TexCoords);
		TangentXArray = std::move(Other.TangentXArray);
		TangentYArray = std::move(Other.TangentYArray);
		TangentZArray = std::move(Other.TangentZArray);
		Colors = std::move(Other.Colors);
		Indices = std::move(Other.Indices);
		LocalToWorld = Other.LocalToWorld;
		return *this;
	}
	TArray<FVector> Positions;
	TArray<FVector2D> TexCoords;
	TArray<FVector> TangentXArray;
	TArray<FVector> TangentYArray;
	TArray<FVector> TangentZArray;
	TArray<FColor> Colors;
	TArray<uint32> Indices;
	FTransform LocalToWorld;
};

 
struct FClothTriangleStaticQuantities
{
	explicit FClothTriangleStaticQuantities(
		const FVector2D &uv0, const FVector2D &uv1, const FVector2D &uv2
	)
	{
		FVector2D duv1 = uv1 - uv0;
		FVector2D duv2 = uv2 - uv0;

		du1 = duv1[0];
		dv1 = duv1[1];
		du2 = duv2[0];
		dv2 = duv2[1];
		// triangle area in reference pose:
		a = 0.5f * (du1 * dv2 - du2 * dv1);

		// first derivatives of uv tangents:
		dwudP0Scalar = (dv1 - dv2) / (2 * a);
		dwudP1Scalar = dv2 / (2 * a);
		dwudP2Scalar = -dv1 / (2 * a);

		dwvdP0Scalar = (du2 - du1) / (2 * a);
		dwvdP1Scalar = -du2 / (2 * a);
		dwvdP2Scalar = du1 / (2 * a);
		
		dwudP0 = FRTMatrix3::Diag(dwudP0Scalar);
		dwudP1 = FRTMatrix3::Diag(dwudP1Scalar);
		dwudP2 = FRTMatrix3::Diag(dwudP2Scalar);

		dwvdP0 = FRTMatrix3::Diag(dwvdP0Scalar);
		dwvdP1 = FRTMatrix3::Diag(dwvdP1Scalar);
		dwvdP2 = FRTMatrix3::Diag(dwvdP2Scalar);
	}
	float du1, dv1, du2, dv2, a;
	
	float dwudP0Scalar, dwudP1Scalar, dwudP2Scalar;
	float dwvdP0Scalar, dwvdP1Scalar, dwvdP2Scalar;
	
	// partial derivatives of Wu Wv on Three Positions:
	FRTMatrix3 dwudP0, dwudP1, dwudP2;
	FRTMatrix3 dwvdP0, dwvdP1, dwvdP2;
};

// computed only once
struct FClothTriangleQuantities : public FClothTriangleStaticQuantities
{
	// tangent vectors:
	FVector wu, wv;

	FClothTriangleQuantities (
		const FVector2D &uv0, const FVector2D &uv1, const FVector2D &uv2
	) : FClothTriangleStaticQuantities(uv0, uv1, uv2)
	{
	}

	virtual ~FClothTriangleQuantities() {}

	virtual void UpdatePositions(const FVector &p0, const FVector &p1, const FVector &p2)
	{
		// trangle tangents in reference directions:
		wu = ((p1 - p0) * dv2 - (p2 - p0) * dv1) / (2 * a);
		wv = (-(p1 - p0) * du2 + (p2 - p0) * du1) / (2 * a);
	}
};
