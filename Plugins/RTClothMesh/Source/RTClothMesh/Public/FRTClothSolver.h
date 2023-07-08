#pragma once

#include "Math/FRTMatrix.h"
#include "Math/FRTSparseMatrix.h"

// interface for cloth solver
template<typename Real>
class IRTClothSolver
{
public:
	virtual void Solve(FRTBBSSMatrix<Real> const& A, TArray<Real> const& B, TArray<Real> &X) = 0;
	virtual ~IRTClothSolver() = 0;
};