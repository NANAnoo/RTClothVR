#pragma once
#include "CoreMinimal.h"

template <typename Real, uint32 Raw, uint32 Col>
class FRTMatrix
{
public:
	
	explicit FRTMatrix() = default;
	
	Real *operator[](const uint32 I) const
	{
		check(I < Raw);
		return const_cast<float*>(&Data[I * Col]);
	}

	// calculation
	FORCEINLINE FRTMatrix<Real, Raw, Raw> operator*(FRTMatrix<Real, Col, Raw> const&Mat) const
	{
		FRTMatrix<Real, Raw, Raw> Res;
		for (uint32 i = 0; i < Raw; i ++)
		{
			for (uint32 j = 0; j < Raw; j ++)
			{
				for (uint32 k = 0; k < Col; k ++)
				{
					Res[i][j] = (*this)[i][k] * Mat[k][j];
				}
			}
		}
		return Res;
	}

	FORCEINLINE FRTMatrix Dot(FRTMatrix const&Other)
	{
		FRTMatrix Res;
		for (uint32 i = 0; i < Raw * Col; i ++)
		{
			Res.Data[i] = Data[i] * Other.Data[i];
		}
		return Res;
	}

	FORCEINLINE FRTMatrix& operator*=(FRTMatrix const&Other)
	{
		for (uint32 i = 0; i < Raw * Col; i ++)
		{
			Data[i] *= Other.Data[i];
		}
		return *this;
	}

	FORCEINLINE FRTMatrix<Real, Raw, Raw> Mul(FRTMatrix<Real, Col, Raw> const&Other)
	{
		return (*this) * Other;
	}

	FORCEINLINE FRTMatrix operator+(FRTMatrix const& Other)
	{
		FRTMatrix Res;
		for (uint32 i = 0; i < Raw * Col; i ++)
		{
			Res.Data[i] = Data[i] - Other.Data[i];
		}
		return Res;
	}

	FORCEINLINE FRTMatrix operator-(FRTMatrix const& Other)
	{
		FRTMatrix Res;
		for (uint32 i = 0; i < Raw * Col; i ++)
		{
			Res.Data[i] = Data[i] - Other.Data[i];
		}
		return Res;
	}

	FORCEINLINE FRTMatrix& operator+=(FRTMatrix const& Other)
	{
		for (uint32 i = 0; i < Raw * Col; i ++)
		{
			Data[i] += Other.Data[i];
		}
		return *this;
	}

	FORCEINLINE FRTMatrix& operator-=(FRTMatrix const& Other)
	{
		for (uint32 i = 0; i < Raw * Col; i ++)
		{
			Data[i] -= Other.Data[i];
		}
		return *this;
	}
	
	template<uint32 R>
	FORCEINLINE friend FRTMatrix<float, R, 1> operator*(FRTMatrix<float, R, 3> const& Mat, FVector const& Vec)
	{
		FRTMatrix<float, R, 1> Res;
		for (uint32 i = 0; i < R; i ++)
		{
			Res[i][0] = Mat[i][0] * Vec[0] + Mat[i][1] * Vec[1] + Mat[i][2] * Vec[2];
		}
		return Res;
	}
	
	FORCEINLINE friend FVector operator*(FRTMatrix<float, 3, 3> const& Mat, FVector const& Vec)
	{
		return {
			Mat[0][0] * Vec[0] + Mat[0][1] * Vec[1] + Mat[0][2] * Vec[2],
			Mat[1][0] * Vec[0] + Mat[1][1] * Vec[1] + Mat[1][2] * Vec[2],
			Mat[2][0] * Vec[0] + Mat[2][1] * Vec[1] + Mat[2][2] * Vec[2]
		};
	}
	
	FORCEINLINE static FRTMatrix<Real, Raw, Raw> Identity()
	{
		FRTMatrix<Real, Raw, Raw> Res;
		for (uint32 i = 0; i < Raw; i ++)
		{
			Res[i][i] = Real(1.0);
		}
		return Res;
	}

	FORCEINLINE static FRTMatrix<Real, 3, 3> CrossVec(FVector const& V1, FVector const& V2)
	{
		FRTMatrix<Real, 3, 3> Res;
		Res[0][0] = V1[0] * V2[0];
		Res[0][1] = V1[0] * V2[1];
		Res[0][2] = V1[0] * V2[2];
		
		Res[1][0] = V1[1] * V2[0];
		Res[1][1] = V1[1] * V2[1];
		Res[1][2] = V1[1] * V2[2];
		
		Res[2][0] = V1[2] * V2[0];
		Res[2][1] = V1[2] * V2[1];
		Res[2][2] = V1[2] * V2[2];
		
		return Res;
	}

	FORCEINLINE static FRTMatrix<Real, Raw, Raw> Diag(Real const Value)
	{
		FRTMatrix<Real, Raw, Raw> Res;
		for (uint32 i = 0; i < Raw; i ++)
		{
			Res[i][i] = Value;
		}
		return Res;
	}

	FORCEINLINE const static FRTMatrix& Zero()
	{
		static FRTMatrix Res;
		return Res;
	}
	
	FORCEINLINE FRTMatrix operator*(Real Scale) const
	{
		FRTMatrix Res;
		for (uint32 i = 0; i < Raw * Col; i ++)
		{
			Res.Data[i] = Data[i] * Scale;
		}
		return Res;
	}

	FORCEINLINE friend FRTMatrix operator*(Real Scale, FRTMatrix const& Mat)
	{
		FRTMatrix Res;
		for (uint32 i = 0; i < Raw * Col; i ++)
		{
			Res.Data[i] = Mat.Data[i] * Scale;
		}
		return Res;
	}

	FORCEINLINE FRTMatrix operator/(Real Scale) const
	{
		return *this * (Real(1) / Scale);
	}
	
	FORCEINLINE FRTMatrix operator/=(Real Scale) const
	{
		*this *= Real(1) / Scale;
		return *this;
	}

	FORCEINLINE FRTMatrix& operator*=(Real Scale) const
	{
		for (uint32 i = 0; i < Raw * Col; i ++)
		{
			Data[i] *= Scale;
		}
		return *this;
	}
	
	FORCEINLINE static constexpr uint32 Raws() {return Raw;}
	FORCEINLINE static constexpr uint32 Cols() {return Col;}
protected:
	Real Data[Raw * Col] = {Real(0)};
};


typedef FRTMatrix<float, 3, 3> FRTMatrix3;

typedef FRTMatrix<float, 2, 2> FRTMatrix2;
