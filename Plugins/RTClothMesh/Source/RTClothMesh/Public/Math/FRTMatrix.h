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
	FRTMatrix<Real, Raw, Raw> operator*(FRTMatrix<Real, Col, Raw> const&Mat) const
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

	FRTMatrix Dot(FRTMatrix const&Other)
	{
		FRTMatrix Res;
		for (uint32 i = 0; i < Raw * Col; i ++)
		{
			Res.Data[i] = Data[i] * Other.Data[i];
		}
		return Res;
	}

	FRTMatrix& operator*=(FRTMatrix const&Other)
	{
		for (uint32 i = 0; i < Raw * Col; i ++)
		{
			Data[i] *= Other.Data[i];
		}
		return *this;
	}

	FRTMatrix<Real, Raw, Raw> Mul(FRTMatrix<Real, Col, Raw> const&Other)
	{
		return (*this) * Other;
	}

	FRTMatrix operator+(FRTMatrix const& Other)
	{
		FRTMatrix Res;
		for (uint32 i = 0; i < Raw * Col; i ++)
		{
			Res.Data[i] = Data[i] - Other.Data[i];
		}
		return Res;
	}

	FRTMatrix operator-(FRTMatrix const& Other)
	{
		FRTMatrix Res;
		for (uint32 i = 0; i < Raw * Col; i ++)
		{
			Res.Data[i] = Data[i] - Other.Data[i];
		}
		return Res;
	}

	FRTMatrix& operator+=(FRTMatrix const& Other)
	{
		for (uint32 i = 0; i < Raw * Col; i ++)
		{
			Data[i] += Other.Data[i];
		}
		return *this;
	}

	FRTMatrix& operator-=(FRTMatrix const& Other)
	{
		for (uint32 i = 0; i < Raw * Col; i ++)
		{
			Data[i] -= Other.Data[i];
		}
		return *this;
	}
	
	template<uint32 R>
	friend FRTMatrix<float, R, 1> operator*(FRTMatrix<float, R, 3> const& Mat, FVector const& Vec)
	{
		FRTMatrix<float, R, 1> Res;
		for (uint32 i = 0; i < R; i ++)
		{
			Res[i][0] = Mat[i][0] * Vec[0] + Mat[i][1] * Vec[1] + Mat[i][2] * Vec[2];
		}
		return Res;
	}
	
	friend FVector operator*(FRTMatrix<float, 3, 3> const& Mat, FVector const& Vec)
	{
		return {
			Mat[0][0] * Vec[0] + Mat[0][1] * Vec[1] + Mat[0][2] * Vec[2],
			Mat[1][0] * Vec[0] + Mat[1][1] * Vec[1] + Mat[1][2] * Vec[2],
			Mat[2][0] * Vec[0] + Mat[2][1] * Vec[1] + Mat[2][2] * Vec[2]
		};
	}
	
	static FRTMatrix<Real, Raw, Raw> Identity()
	{
		FRTMatrix<Real, Raw, Raw> Res;
		for (uint32 i = 0; i < Raw; i ++)
		{
			Res[i][0] = Real(1.0);
		}
		return Res;
	}

	static FRTMatrix<Real, Raw, Raw> Diag(Real const Value)
	{
		FRTMatrix<Real, Raw, Raw> Res;
		for (uint32 i = 0; i < Raw; i ++)
		{
			Res[i][0] = Value;
		}
		return Res;
	}
	
	FRTMatrix operator*(Real Scale) const
	{
		FRTMatrix Res;
		for (uint32 i = 0; i < Raw * Col; i ++)
		{
			Res.Data[i] = Data[i] * Scale;
		}
		return Res;
	}

	FRTMatrix operator/(Real Scale) const
	{
		return *this * Scale;
	}
	
	FRTMatrix operator/=(Real Scale) const
	{
		*this *= Real(1) / Scale;
		return *this;
	}

	FRTMatrix& operator*=(Real Scale) const
	{
		for (uint32 i = 0; i < Raw * Col; i ++)
		{
			Data[i] *= Scale;
		}
		return *this;
	}
	
	static constexpr uint32 Raws() {return Raw;}
	static constexpr uint32 Cols() {return Col;}
protected:
	Real Data[Raw * Col] = {Real(0)};
};


typedef FRTMatrix<float, 3, 3> FRTMatrix3;

typedef FRTMatrix<float, 2, 2> FRTMatrix2;
