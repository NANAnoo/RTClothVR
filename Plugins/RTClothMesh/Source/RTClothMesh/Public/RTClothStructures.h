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

 
struct FClothTriangleStaticProperties
{
	explicit FClothTriangleStaticProperties(
		const int P0, const int P1, const int P2, 
		const FVector2D &uv0, const FVector2D uv1, const FVector2D &uv2
	) : V_Inx{P0, P1, P2}
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
		// first derivatives of uv tangents:
		dwudXScalar[0] = (dv1 - dv2) / d;
		dwudXScalar[1] = dv2 / d;
		dwudXScalar[2] = -dv1 / d;

		dwvdXScalar[0] = (du2 - du1) / d;
		dwvdXScalar[1] = -du2 / d;
		dwvdXScalar[2] = du1 / d;
	}
	
	int ID;
	int V_Inx[3]; // indices of triangles
	float du1, dv1, du2, dv2;

	float a; // area of triangle in UV space
	float d; // du1 * dv2 - du2 * dv1
	
	float dwudXScalar[3];
	float dwvdXScalar[3];
};

static const float Q_rsqrt( float number )
{
	uint32_t i;
	float x2, y;
	const float threehalfs = 1.5F;

	x2 = number * 0.5F;
	y  = number;
	memcpy(&i, &y, sizeof(float));// evil floating point bit level hacking
	i  = 0x5f3759df - ( i >> 1 );               // what the fuck?
	memcpy(&y, &i, sizeof(float));
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

	return y;
}

// computed only once
struct FClothTriangleProperties : public FClothTriangleStaticProperties
{
public:
	// tangent vectors:
	FVector wu, wv;
	float wuNorm = 0, wvNorm = 0;

	// partial derivatives of Wu Wv on Three Positions:
	FRTMatrix3 dwudX[3];
	FRTMatrix3 dwvdX[3];

	FClothTriangleProperties (
		const int P0, const int P1, const int P2, 
		const FVector2D &uv0, const FVector2D &uv1, const FVector2D &uv2
	) : FClothTriangleStaticProperties(P0, P1, P2, uv0, uv1, uv2)
	{
		for (uint32 i = 0; i < 3; i ++)
		{
			dwudX[i] = FRTMatrix3::Diag(dwudXScalar[i]);
			dwvdX[i] = FRTMatrix3::Diag(dwvdXScalar[i]);
		}
	}

	virtual ~FClothTriangleProperties() {}

	virtual void Update(const FVector &P0, const FVector &P1, const FVector &P2, const FVector& V0, const FVector& V1, const FVector& V2)
	{
		// trangle tangents in reference directions:
		wu = ((P1 - P0) * dv2 - (P2 - P0) * dv1) / d;
		wv = (-(P1 - P0) * du2 + (P2 - P0) * du1) / d;
		wuNorm = 1.0f / Q_rsqrt(wu[0] * wu[0] + wu[1] * wu[1] +wu[2] * wu[2]);
		wvNorm = 1.0f / Q_rsqrt(wv[0] * wv[0] + wv[1] * wv[1] +wv[2] * wv[2]);
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
			return FRTMatrix3::Identity() - FRTMatrix3::CrossVec(P / P.Size(), P / P.Size());
		case ConstraintOnLine:
			return FRTMatrix3::Identity() - FRTMatrix3::CrossVec(P / P.Size(), P / P.Size()) - FRTMatrix3::CrossVec(Q / Q.Size(), Q / Q.Size());
		case Fixed:
		default:
			return FRTMatrix3::Zero();
		}
	}
};

template<unsigned int MaxNum>
class FRTDebugLogger : public TThreadSingleton<FRTDebugLogger<MaxNum>>
{
public:
	void Record(FVector const&Vec)
	{
		if (MaxNum > Data.Num())
			Data.Add(Vec);
		else if (!Logged)
		{
			Print();
			Logged = true;
		}
	}
	void Print() const
	{
		UE_LOG(LogTemp, Warning, TEXT("Recorded Data:"));
		for (auto &v : Data)
		{
			UE_LOG(LogTemp, Warning, TEXT("%f, %f, %f"), v[0], v[1], v[2]);
		}
	}
private:
	TArray<FVector> Data;
	bool Logged = false;
};

// Cloth Collider
struct FRTClothCollider {
	constexpr static int FRTClothColliderSphere = 0;
	constexpr static int FRTClothColliderCapsule = 1;
	constexpr static int FRTClothColliderBox = 2;

	FMatrix LocalToWorld = FMatrix::Identity;
	FVector4 Velocity;
	union
	{
		float Radius;      // Sphere, Capsule
		float HalfExtentX; // Box
	};
	union
	{
		float HalfHeight;  // Capsule
		float HalfExtentY; // Box
	};
	float HalfExtentZ; // Box
	int Type = FRTClothColliderSphere;
};

