﻿#pragma once

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

 
struct FClothTriangleStaticProperties
{
	explicit FClothTriangleStaticProperties(
		const FVector2D &uv0, const FVector2D &uv1, const FVector2D &uv2
	)
	{
		FVector2D duv1 = uv1 - uv0;
		FVector2D duv2 = uv2 - uv0;

		du1 = duv1[0];
		dv1 = duv1[1];
		du2 = duv2[0];
		dv2 = duv2[1];
		d = du1 * dv2 - du2 * dv1;
		// triangle area in reference pose:
		a = 0.5f * abs(d);
		d = 2 * a;
		// first derivatives of uv tangents:
		dwudXScalar[0] = (dv1 - dv2) / d;
		dwudXScalar[1] = dv2 / d;
		dwudXScalar[2] = -dv1 / d;

		dwvdXScalar[0] = (du2 - du1) / d;
		dwvdXScalar[1] = -du2 / d;
		dwvdXScalar[2] = du1 / d;

		for (uint32 i = 0; i < 3; i ++)
		{
			dwudX[i] = FRTMatrix3::Diag(dwudXScalar[i]);
			dwvdX[i] = FRTMatrix3::Diag(dwvdXScalar[i]);
		}
	}
	float du1, dv1, du2, dv2, a, d;
	
	float dwudXScalar[3];
	float dwvdXScalar[3];
	
	// partial derivatives of Wu Wv on Three Positions:
	FRTMatrix3 dwudX[3];
	FRTMatrix3 dwvdX[3];
};

// computed only once
struct FClothTriangleProperties : public FClothTriangleStaticProperties
{
public:
	// tangent vectors:
	FVector wu, wv;
	float wuNorm = 0, wvNorm = 0;

	FClothTriangleProperties (
		const FVector2D &uv0, const FVector2D &uv1, const FVector2D &uv2
	) : FClothTriangleStaticProperties(uv0, uv1, uv2)
	{
	}

	virtual ~FClothTriangleProperties() {}

	virtual void Update(const FVector &P0, const FVector &P1, const FVector &P2, const FVector& V0, const FVector& V1, const FVector& V2)
	{
		// trangle tangents in reference directions:
		wu = ((P1 - P0) * dv2 - (P2 - P0) * dv1) / d;
		wv = (-(P1 - P0) * du2 + (P2 - P0) * du1) / d;
		wuNorm = wu.Size();
		wvNorm = wv.Size();
	}
};

struct FClothConstraint
{
	enum ELockingType
	{
		ConstraintOnPlane, // Locking P
		ConstraintOnLine, // Locking P and Q
		Fixed
	};
	// Locked Degree
	ELockingType LockingDegree = Fixed;
	FVector P;
	FVector Q;
	
	FRTMatrix3 ConstraintsMat() const
	{
		switch (LockingDegree)
		{
		case ConstraintOnPlane:
			return FRTMatrix3::Identity() - FRTMatrix3::CrossVec(P, P);
		case ConstraintOnLine:
			return FRTMatrix3::Identity() - FRTMatrix3::CrossVec(P, P) - FRTMatrix3::CrossVec(Q, Q);
		case Fixed:
		default:
			return FRTMatrix3::Zero();
		}
	}
};


