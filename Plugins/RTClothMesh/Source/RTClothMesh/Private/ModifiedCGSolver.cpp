#include "ModifiedCGSolver.h"

void FModifiedCGSolver::Init(FRTBBSSMatrix<float> const&Mat)
{
	P.SetNumUninitialized(Mat.Size());
	R.SetNumUninitialized(Mat.Size());
	C.SetNumUninitialized(Mat.Size());
	S.SetNumUninitialized(Mat.Size());
	Q.SetNumUninitialized(Mat.Size());
}

void FModifiedCGSolver::Solve(FRTBBSSMatrix<float> const& A, TArray<float> const& B, TArray<float> &X)
{
	// setup precondition
	for (int32 i = 0; i < P.Num(); i ++)
	{
		P[i] = A[i][i];
	}
	uint32 const Size = X.Num();
	// set up velocity constraints
	if (VelConstrains.Num() == Size)
	{
		for (uint32 i = 0; i < Size; i ++)
		{
			X[i] = VelConstrains[i];
		}
	}

	// calculate delta0
	// δ0 = filter(b)TP filter(b)
	// C = filter(b)
	Filter(B, C);
	// S = P filter(b)
	Precondition(C, false, S);
	// delta0 = C.Dot(S)
	float Delta_0 = 0;
	for (uint32 i = 0; i < Size; i ++)
		Delta_0 += C[i] * S[i];

	// r = filter(b − AX)
	A.MulVector(R.GetData(), X.GetData(), X.Num());
	for (uint32 i = 0; i < Size; i ++)
		R[i] = B[i] - R[i];
	Filter(R, R);

	// c = filter(P−1r)
	Precondition(R, true, C);
	Filter(C, C);

	// δ_new = rT c
	float Delta_new = 0;
	for (uint32 i = 0; i < Size; i ++)
		Delta_new += C[i] * R[i];

	for (uint32 I = 0;Delta_new > Tolerance * Tolerance * Delta_0 && I < MaxIterations; I ++)
	{
		// q = filter(Ac)
		A.MulVector(Q.GetData(), C.GetData(), C.Num());
		Filter(Q, Q);

		// α = δnew/(cT q)
		float Alpha = 0;
		for (uint32 i = 0; i < Size; i ++)
			Alpha += C[i] * Q[i];
		Alpha = Delta_new / Alpha;

		// X = X + αc
		for (uint32 i = 0; i < Size; i ++)
			X[i] += Alpha * C[i];

		// r = r − αq
		for (uint32 i = 0; i < Size; i ++)
			R[i] -= Alpha * Q[i];

		// s = P−1r
		Precondition(R, true, S);

		float Delta_old = Delta_new;
		Delta_new = 0;
		// δnew = rT s
		for (uint32 i = 0; i < Size; i ++)
			Delta_new += C[i] * R[i];

		// c = filter(s + δnew/δold * c)
		Delta_old = Delta_new / Delta_old;
		for (uint32 i = 0; i < Size; i ++)
			C[i] = S[i] + Delta_old * C[i];

		Filter(C, C);
	}
}

FModifiedCGSolver::~FModifiedCGSolver()
{
	
}

void FModifiedCGSolver::Precondition(TArray<float> const&In, bool Inverse, TArray<float> &Out)
{
	for (int32 i = 0; i < In.Num(); i ++) 
	{
		Out[i] = Inverse ? In[i] / P[i] : In[i] * P[i];
	}
}

void FModifiedCGSolver::Filter(TArray<float> const&In, TArray<float> &Out)
{
	for (int32 i = 0; i < ConstraintIds.Num(); i ++)
	{
		uint32 const idx  = ConstraintIds[i];
		auto res = ConstraintMats[i] * FVector(In[3 * idx], In[3 * idx + 1], In[3 * idx + 2]);
		Out[3 * idx] = res[0];
		Out[3 * idx + 1] = res[1];
		Out[3 * idx + 2] = res[2];
	}
}