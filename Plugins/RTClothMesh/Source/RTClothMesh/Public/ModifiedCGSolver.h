#pragma once
#include "FRTClothSolver.h"

class FModifiedCGSolver : public IRTClothSolver<float>
{
public:
	FModifiedCGSolver(float Tol = 1e-9, uint32 MaxItNums = 100)
		: IRTClothSolver(Tol, MaxItNums) {}
	virtual void Init(FRTBBSSMatrix<float> const&) override;
	virtual void Solve(FRTBBSSMatrix<float> & A, TArray<float> const& B, TArray<float> &X) override;
	virtual ~FModifiedCGSolver() override;

	virtual void UpdateConstraints(TArray<uint32> const&Ids, TArray<FRTMatrix3> const& Mats) override
	{
		ConstraintIds = Ids;
		ConstraintMats = Mats;
	}

	virtual void UpdateVelocityConstraints(TArray<float> const& Cons) override
	{
		VelConstrains = Cons;
	}
	
private:

	void Precondition(TArray<float> const&In, bool Inverse, TArray<float> &Out);
	void Filter(TArray<float> &Out);
	
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
