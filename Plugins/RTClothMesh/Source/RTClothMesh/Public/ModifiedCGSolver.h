#pragma once
#include "FRTClothSolver.h"

class FModifiedCGSolver : public IRTClothSolver<float>
{
public:
	FModifiedCGSolver(float Tol = 1e-6, uint32 MaxItNums = 50)
		: IRTClothSolver(Tol, MaxItNums) {}
	virtual void Init(FRTBBSSMatrix<float> const&) override;
	virtual void Solve(FRTBBSSMatrix<float> const& A, TArray<float> const& B, TArray<float> &X) override;
	virtual ~FModifiedCGSolver() override;

	void UpdateConstraints(TArray<uint32> const&Ids, TArray<FRTMatrix3> const& Mats)
	{
		ConstraintIds = Ids;
		ConstraintMats = Mats;
	}

	void UpdateVelocityConstraints(TArray<float> const& Cons)
	{
		VelConstrains = Cons;
	}
	
private:

	void Precondition(TArray<float> const&In, bool Inverse, TArray<float> &Out);
	void Filter(TArray<float> const&In, TArray<float> &Out);
	
	TArray<uint32> ConstraintIds;
	TArray<FRTMatrix3> ConstraintMats;
	TArray<float> VelConstrains;

	// Precondition
	TArray<float> P;
	
	// residual
	TArray<float> R;
	TArray<float> S;
	TArray<float> C;
	TArray<float> Q;
};
