#pragma once

#include "Math/FRTMatrix.h"
#include "Math/FRTSparseMatrix.h"

// interface for cloth solver
template<typename Real>
class IRTClothSolver
{
public:
	IRTClothSolver(Real Tol, uint32 MaxItNums) : Tolerance(Tol), MaxIterations(MaxItNums) {}
	virtual void Init(FRTBBSSMatrix<Real> const&) = 0;
	virtual void Solve(FRTBBSSMatrix<Real> const& A, TArray<Real> const& B, TArray<Real> &X) = 0;
	virtual ~IRTClothSolver() = 0;
protected:
	Real Tolerance;
	uint32 MaxIterations;
};