#include "ModifiedCGSolver.h"

void FModifiedCGSolver::Init(FRTBBSSMatrix<float> const&Mat)
{
	P.SetNumZeroed(Mat.Size());
	R.SetNumZeroed(Mat.Size());
	C.SetNumZeroed(Mat.Size());
	S.SetNumZeroed(Mat.Size());
	Q.SetNumZeroed(Mat.Size());
}

void FModifiedCGSolver::Solve(FRTBBSSMatrix<float> & A, TArray<float> const& B, TArray<float> &X)
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
	for (uint32 i = 0; i < Size; i ++)
		C[i] = B[i];
	Filter(C);
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
	Filter(R);

	// c = filter(P−1r)
	Precondition(R, true, C);
	Filter(C);

	// δ_new = rT c
	float Delta_new = 0;
	for (uint32 i = 0; i < Size; i ++)
		Delta_new += C[i] * R[i];
	uint32 I = 0;
	UE_LOG(LogTemp, Warning, TEXT("---------------- New Frame -----------------"));
	double Timer = FPlatformTime::Seconds();
	double MatVecMul = 0, Other = 0;
	for (;Delta_new > Tolerance * Tolerance * Delta_0 && I < MaxIterations; I ++)
	{
		Timer = FPlatformTime::Seconds();
		// q = filter(Ac)
		A.MulVector(Q.GetData(), C.GetData(), C.Num());
		Filter(Q);
		MatVecMul += FPlatformTime::Seconds() - Timer;
		Timer = FPlatformTime::Seconds();

		// α = δnew/(cT q)
		float Alpha = 0;
		for (uint32 i = 0; i < Size; i ++)
			Alpha += C[i] * Q[i];
		check(!isnan(Alpha) && !isinf(Alpha))
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
			Delta_new += R[i] * S[i];
		
		check(!isnan(Delta_new) && !isinf(Delta_new))
		// c = filter(s + δnew/δold * c)
		Delta_old = Delta_new / Delta_old;
		for (uint32 i = 0; i < Size; i ++)
			C[i] = S[i] + Delta_old * C[i];

		Filter(C);
		Other += FPlatformTime::Seconds() - Timer;
		UE_LOG(LogTemp, Warning, TEXT("delta %f"), Delta_new);
	}
	Timer = (FPlatformTime::Seconds() - Timer) * 1000.0;
	UE_LOG(LogTemp, Warning, TEXT("Iterations %d, Final Delta Radio %f, Cost %f (/It), MulVec %f, VecVec %f"), I, Delta_new / (Tolerance * Tolerance * Delta_0), Timer / (I + 1), MatVecMul, Other);
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

void FModifiedCGSolver::Filter(TArray<float> &Out)
{
	for (int32 i = 0; i < ConstraintIds.Num(); i ++)
	{
		uint32 const idx  = ConstraintIds[i];
		auto res = ConstraintMats[i] * FVector(Out[3 * idx], Out[3 * idx + 1], Out[3 * idx + 2]);
		Out[3 * idx] = res[0];
		Out[3 * idx + 1] = res[1];
		Out[3 * idx + 2] = res[2];
	}
}