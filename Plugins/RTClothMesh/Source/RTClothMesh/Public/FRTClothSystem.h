﻿#pragma once

#include "FRTClothSystemBase.h"

#include "Math/FRTSparseMatrix.h"
#include "FRTShearCondition.h"
#include "FRTStretchCondition.h"
#include "FRTBendCondition.h"

#include <memory>
#include "FRTClothSolver.h"

class FRTClothSystem : public FRTClothSystemBase
{
public:
	explicit FRTClothSystem(std::shared_ptr<IRTLinearSolver<float>> const &ASolver)
		: Solver(ASolver) {}

	virtual void TickOnce(float Duration) override;
	
private:
	// calculate forces and derivatives
	void ForcesAndDerivatives();

	// setup runtime variables
	virtual void PrepareSimulation() override;

	// TODO : solve inner collision

	// pre computed conditions cache
	TArray<FRTStretchCondition> StretchConditions;
	TArray<FRTShearCondition> ShearConditions;
	TArray<FRTBendCondition> BendConditions;

	// forces and derivatives
	TArray<FVector> Forces;
	TArray<FVector> DampingForces;
	FRTBBSSMatrix<float> Df_Dx;
	FRTBBSSMatrix<float> Df_Dv;

	// For solvers x = A/B
	FRTBBSSMatrix<float> A;
	TArray<float> B;
	std::shared_ptr<IRTLinearSolver<float>> Solver;
	
	// physics properties at each particle
	TArray<FVector> Velocity;

	// check if it's fist frame of the incoming mesh
	bool IsFirstFrame = false;
};