﻿#pragma once

#include "Math/FRTMatrix.h"
#include "Math/FRTSparseMatrix.h"

// interface for cloth solver
template<typename Real>
class IRTLinearSolver
{
public:
	IRTLinearSolver(Real Tol, uint32 MaxItNums) : Tolerance(Tol), MaxIterations(MaxItNums) {}
	virtual void Init(FRTBBSSMatrix<Real> const&) = 0;
	virtual void Solve(FRTBBSSMatrix<Real> & A, TArray<Real> const& B, TArray<Real> &X) = 0;
	virtual void UpdateConstraints(TArray<uint32> const&Ids, TArray<FRTMatrix<Real, 3,3>> const& Mats) = 0;
	virtual void UpdateVelocityConstraints(FVector const&Vel) = 0;
	virtual ~IRTLinearSolver() {}
protected:
	Real Tolerance;
	uint32 MaxIterations;
};