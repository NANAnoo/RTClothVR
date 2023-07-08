#include "Math/FRTSparseMatrix.h"

// template<typename BlockType>
// void FRTBBSSMatrix<BlockType>::MulVector(BlockType const* InData, BlockType *OutData, uint32 const Len)
// {
// 	check(Compressed && Len == Size);
// 	if (OutData == nullptr)
// 	{
// 		OutData = new BlockType[Len]();
// 	}
//
// 	// Sparse x Vector
// 	for (uint32 I = 0; I < Size; I ++)
// 	{
// 		uint32 const StartIndex = I == 0 ? 0 : NumEntriesOfRaw[I - 1];
// 		uint32 const NumOfEntries = NumEntriesOfRaw[I];
// 		OutData[I] = DiagData[I] * InData[I];
// 		for (uint32 E = 0; E < NumOfEntries; E ++)
// 		{
// 			uint32 const J = ColIndexAtRaw[StartIndex + E];
// 			if (J != I)
// 				OutData[I] += OffDiagData[StartIndex + E] * InData[J];
// 		}
// 	}
// }
//
// template <typename BlockType>
// void FRTBBSSMatrix<BlockType>::MakeCompressed()
// {
// 	int NumOfOffDiagEntries = 0;
// 	for (auto &Raw : TempData)
// 	{
// 		NumOfOffDiagEntries += Raw.Num();
// 	}
// 	NumEntriesOfRaw = new uint32[Size];
// 	ColIndexAtRaw = new uint32[NumOfOffDiagEntries]();
// 	OffDiagData = new BlockType[NumOfOffDiagEntries]();
//
// 	int CurrentStartIndex = 0;
// 	for (uint32 i = 0; i < Size; i ++)
// 	{
// 		TMap<uint32, BlockType> &Raw = TempData[i];
// 		int NumAtCurRaw = 0;
// 		// copy data into OffDiagData
// 		for (uint32 j = 0; j < Size; j ++)
// 		{
// 			if (Raw.Contains(j))
// 			{
// 				ColIndexAtRaw[CurrentStartIndex + NumAtCurRaw] = j;
// 				OffDiagData[CurrentStartIndex + NumAtCurRaw] = Raw[j];
// 				NumAtCurRaw ++;
// 			}
// 		}
// 		check(NumAtCurRaw == Raw.Num());
// 		CurrentStartIndex = Raw.Num();
// 		NumEntriesOfRaw[i] = CurrentStartIndex;
// 	}
// 	Compressed = true;
// }

// template<typename BlockType>
// FRTBBSSMatrix<BlockType>::~FRTBBSSMatrix()
// {
// 	delete[] DiagData;
// 	if (Compressed)
// 	{
// 		delete[] OffDiagData;
// 		delete[] NumEntriesOfRaw;
// 		delete[] ColIndexAtRaw;
// 		OffDiagData = nullptr;
// 		NumEntriesOfRaw = nullptr;
// 		ColIndexAtRaw = nullptr;
// 	}
// }