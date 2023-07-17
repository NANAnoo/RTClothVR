#pragma once

#include "FRTClothSystemBase.h"

#include "FRTShearCondition.h"
#include "FRTStretchCondition.h"
#include "FRTBendCondition.h"

// Verlet Integration
// X_{i + 1} = 2 * X_{i} - X_{i-1} + Acc(X_{i}) * h ^ 2

class FRTClothSystem_Verlet_CPU : public FRTClothSystemBase
{
public:
	explicit FRTClothSystem_Verlet_CPU() = default;

	virtual void TickOnce(float Duration) override;
	
private:
	// calculate forces and derivatives
	void Acceleration();

	// setup runtime variables
	virtual void PrepareSimulation() override;

	// TODO : solve inner collision

	// pre computed conditions cache. use forces only
	TArray<FRTStretchCondition> StretchConditions;
	TArray<FRTShearCondition> ShearConditions;
	TArray<FRTBendCondition> BendConditions;

	// Forces
	TArray<FVector> Forces;
	
	// X_{i-1}
	TArray<FVector> Pre_Positions;
	TArray<FVector> Velocities;

};