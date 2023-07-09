#pragma once

#include "CoreMinimal.h"

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
			if (!Mat->TempData[Raw].Contains(J))
			{
				Mat->TempData[Raw].Add(J, BlockType());
			}
			return Mat->TempData[Raw][J];
		}
	private:
		FRTBBSSMatrix *Mat;
		int Raw;
	};
	explicit FRTBBSSMatrix(uint32 const Size) : Size(Size)
	{
		DiagData = new BlockType[Size]();
		TempData.SetNumZeroed(Size);
	}

	FRTBBSSMatrixRaw operator[](uint32 I)
	{
		check(I < Size);
		if (Compressed)
		{
			Compressed = false;
			delete[] OffDiagData;
			delete[] NumEntriesOfRaw;
			delete[] ColIndexAtRaw;
			OffDiagData = nullptr;
			NumEntriesOfRaw = nullptr;
			ColIndexAtRaw = nullptr;
		}
		return FRTBBSSMatrixRaw(this, I);
	}
	
	virtual ~FRTBBSSMatrix()
	{
		delete[] DiagData;
		if (Compressed)
		{
			delete[] OffDiagData;
			delete[] NumEntriesOfRaw;
			delete[] ColIndexAtRaw;
			OffDiagData = nullptr;
			NumEntriesOfRaw = nullptr;
			ColIndexAtRaw = nullptr;
		}
	}

	void MakeCompressed()
	{
		int NumOfOffDiagEntries = 0;
		for (auto &Raw : TempData)
		{
			NumOfOffDiagEntries += Raw.Num();
		}
		NumEntriesOfRaw = new uint32[Size];
		ColIndexAtRaw = new uint32[NumOfOffDiagEntries]();
		OffDiagData = new BlockType[NumOfOffDiagEntries]();

		int CurrentStartIndex = 0;
		for (uint32 i = 0; i < Size; i ++)
		{
			TMap<uint32, BlockType> &Raw = TempData[i];
			int NumAtCurRaw = 0;
			// copy data into OffDiagData
			for (uint32 j = 0; j < Size; j ++)
			{
				if (Raw.Contains(j))
				{
					ColIndexAtRaw[CurrentStartIndex + NumAtCurRaw] = j;
					OffDiagData[CurrentStartIndex + NumAtCurRaw] = Raw[j];
					NumAtCurRaw ++;
				}
			}
			check(NumAtCurRaw == Raw.Num());
			CurrentStartIndex = Raw.Num();
			NumEntriesOfRaw[i] = CurrentStartIndex;
		}
		Compressed = true;
	}

	// TODO, Lock Pattern
	
	void MulVector(BlockType const* InData, BlockType *OutData, uint32 const Len)
	{
		check(Compressed && Len == Size);
		if (OutData == nullptr)
		{
			OutData = new BlockType[Len]();
		}

		// Sparse x Vector
		for (uint32 I = 0; I < Size; I ++)
		{
			uint32 const StartIndex = I == 0 ? 0 : NumEntriesOfRaw[I - 1];
			uint32 const NumOfEntries = NumEntriesOfRaw[I];
			OutData[I] = DiagData[I] * InData[I];
			for (uint32 E = 0; E < NumOfEntries; E ++)
			{
				uint32 const J = ColIndexAtRaw[StartIndex + E];
				if (J != I)
					OutData[I] += OffDiagData[StartIndex + E] * InData[J];
			}
		}
	}
	
private:
	bool Compressed = false;
	TArray<TMap<uint32, BlockType>> TempData;
	uint32 Size;
	// StartIndex[i] = 0 if i == 0 else NumEntriesOfRaw[i-1]
	uint32 *NumEntriesOfRaw = nullptr;
	// Same size of OffDiagData, |uint32[NumOfEntriesOfRaw[0]]|uint32[NumOfEntriesOfRaw[1]]
	// Value is the Col Index
	uint32 *ColIndexAtRaw = nullptr;
	BlockType *OffDiagData = nullptr;
	BlockType *DiagData = nullptr;
};

