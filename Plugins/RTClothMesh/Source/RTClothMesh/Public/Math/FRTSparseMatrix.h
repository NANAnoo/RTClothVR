#pragma once

#include "CoreMinimal.h"

// sparse pattern of a matrix
struct FrtSparsePattern
{
	bool operator==(FrtSparsePattern const&Other)
	{
		if (Size != Other.Size) return false;
		if (PreSumNumEntriesOfRaw.Num() != Other.PreSumNumEntriesOfRaw.Num()) return false;
		if (ColIndexAtEntrance.Num() != Other.ColIndexAtEntrance.Num()) return false;
		for (int32 i = 0; i < PreSumNumEntriesOfRaw.Num(); i ++)
		{
			if (PreSumNumEntriesOfRaw[i] != Other.PreSumNumEntriesOfRaw[i]) return false;
		}
		for (int32 i = 0; i < ColIndexAtEntrance.Num(); i ++)
		{
			if (ColIndexAtEntrance[i] != Other.ColIndexAtEntrance[i]) return false;
		}
		return true;
	}
	uint32 Size = 0;
	// StartIndex[i] = 0 if i == 0 else PreSumNumEntriesOfRaw[i-1]
	TArray<uint32> PreSumNumEntriesOfRaw;
	// Same size of OffDiagData, |uint32[NumOfEntriesOfRaw[0]]|uint32[NumOfEntriesOfRaw[1]]
	// Value is the Col Index
	TArray<uint32> ColIndexAtEntrance;
};

// Block Based Square Sparse Matrix
template <typename BlockType>
class FRTBBSSMatrix
{
public:
	struct FRTBBSSMatrixRaw
	{
		explicit FRTBBSSMatrixRaw(FRTBBSSMatrix *M, uint32 R) : Mat(M), Raw(R) {}
		
		BlockType& operator[](uint32 const J)
		{
			if (J == Raw)
			{
				return Mat->DiagData[J];
			}
			if (Mat->IsLockPattern)
			{
				auto const Eid = Raw * Mat->Pattern.Size + J;
				check(Mat->IDToCompressedID.Contains(Eid));
				return Mat->OffDiagData[Mat->IDToCompressedID[Eid]];
			}
			Mat->ReleaseCompressedData();
			if (!Mat->TempData[Raw].Contains(J))
			{
				Mat->TempData[Raw].Add(J, BlockType());
			}
			return Mat->TempData[Raw][J];
		}

		const BlockType& operator[](uint32 const J) const
		{
			if (J == Raw)
			{
				return Mat->DiagData[J];
			}
			check(Mat->TempData[Raw].Contains(J));
			return Mat->TempData[Raw][J];
		}
	private:
		FRTBBSSMatrix *Mat;
		uint32 Raw;
	};
	FRTBBSSMatrix() = default;

	FRTBBSSMatrixRaw operator[](uint32 I)
	{
		check(I < Pattern.Size);
		return FRTBBSSMatrixRaw(this, I);
	}

	void UpdateSize(uint32 N)
	{
		IsLockPattern = false;
		ReleaseCompressedData();
		Pattern.Size = N;
		DiagData.SetNumZeroed(Pattern.Size);
		TempData.SetNumZeroed(Pattern.Size);
	}

	FORCEINLINE void ReleaseCompressedData()
	{
		if (Compressed)
		{
			OffDiagData.Empty();
			Pattern.ColIndexAtEntrance.Empty();
			Pattern.PreSumNumEntriesOfRaw.Empty();
			IDToCompressedID.Reset();
			Compressed = false;
		}
	}
	
	virtual ~FRTBBSSMatrix()
	{
	}

	void MakeCompressed()
	{
		ReleaseCompressedData();
		int NumOfOffDiagEntries = 0;
		for (auto &Raw : TempData)
		{
			NumOfOffDiagEntries += Raw.Num();
		}
		Pattern.PreSumNumEntriesOfRaw.SetNumZeroed(Pattern.Size);
		Pattern.ColIndexAtEntrance.SetNumZeroed(NumOfOffDiagEntries);
		OffDiagData.SetNumZeroed(NumOfOffDiagEntries);

		uint32 CurrentStartIndex = 0;
		for (uint32 i = 0; i < Pattern.Size; i ++)
		{
			TMap<uint32, BlockType> &Raw = TempData[i];
			uint32 NumAtCurRaw = 0;
			// copy data into OffDiagData
			for (uint32 j = 0; j < Pattern.Size; j ++)
			{
				if (Raw.Contains(j))
				{
					auto const Eid = CurrentStartIndex + NumAtCurRaw;
					IDToCompressedID.Add(i * Pattern.Size + j, Eid);
					Pattern.ColIndexAtEntrance[Eid] = j;
					OffDiagData[Eid] = Raw[j];
					NumAtCurRaw ++;
				}
			}
			check(NumAtCurRaw == Raw.Num());
			CurrentStartIndex += Raw.Num();
			Pattern.PreSumNumEntriesOfRaw[i] = CurrentStartIndex;
		}
		TempData.SetNumZeroed(Pattern.Size);
		Compressed = true;
	}

	// Check if two sparse matrix has same pattern
	bool HasSamePattern(FRTBBSSMatrix const&Other)
	{
		return Pattern == Other.Pattern;
	}

	// Lock the sparse pattern,
	// Then the element update will happen in place
	void LockPattern()
	{
		if (!IsLockPattern)
		{
			IsLockPattern = true;
			MakeCompressed();
		}
	}

	uint32 Size() const
	{
		return Pattern.Size;
	}

	// Calculation under Compressed State
	void MulVector(BlockType *OutData, BlockType const* InData, uint32 const Len) const
	{
		check(Compressed && Len == Pattern.Size && OutData != nullptr);
		
		// Sparse x Vector
		for (uint32 I = 0; I < Pattern.Size; I ++)
		{
			uint32 const StartIndex = I == 0 ? 0 : Pattern.PreSumNumEntriesOfRaw[I - 1];
			uint32 const EndIndex = Pattern.PreSumNumEntriesOfRaw[I];
			OutData[I] = DiagData[I] * InData[I];
			for (uint32 E = StartIndex; E < EndIndex; E ++)
			{
				uint32 const J = Pattern.ColIndexAtEntrance[E];
				if (J != I)
					OutData[I] += OffDiagData[E] * InData[J];
			}
		}
	}
	
	// Apply Operation with another matrix that has same pattern
	bool Execute(FRTBBSSMatrix & OutMat, FRTBBSSMatrix const& InMat,
		TFunction<float(int32, float, float)> const& Diag,
		TFunction<float(float, float)> const& OffDiag)
	{
		if (!HasSamePattern(InMat) || !HasSamePattern(OutMat)) return false;
		for (uint32 i = 0; i < Pattern.Size; i++)
		{
			OutMat.DiagData[i] = Diag(i, DiagData[i], InMat.DiagData[i]);
		}
		for (uint32 i = 0; i < Pattern.PreSumNumEntriesOfRaw.Last(); i ++)
		{
			OutMat.OffDiagData[i] = OffDiag(OffDiagData[i], InMat.OffDiagData[i]);
		}
		return true;
	}
	
	static FRTBBSSMatrix MatrixFromOtherPattern(FRTBBSSMatrix const& Other)
	{
		FRTBBSSMatrix Res;
		Res.Compressed = true;
		Res.IsLockPattern = true;
		Res.IDToCompressedID = Other.IDToCompressedID;
		Res.Pattern = Other.Pattern;
		Res.DiagData.SetNumZeroed(Res.Pattern.Size);
		Res.OffDiagData.SetNumZeroed(Res.Pattern.PreSumNumEntriesOfRaw.Last());
		return Res;
	}
	
private:
	bool Compressed = false;
	bool IsLockPattern = false;
	TArray<TMap<uint32, BlockType>> TempData;
	TMap<uint32, uint32> IDToCompressedID;
	FrtSparsePattern Pattern;
	TArray<BlockType> OffDiagData;
	TArray<BlockType> DiagData;
};

