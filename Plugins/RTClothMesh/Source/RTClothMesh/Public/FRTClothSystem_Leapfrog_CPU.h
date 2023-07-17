#pragma once

#include "FRTClothSystemBase.h"

#include "FRTShearCondition.h"
#include "FRTStretchCondition.h"
#include "FRTBendCondition.h"

// LeafFrog Integration
// V_{i + 1/2} = V[i] + Acc[i] * h / 2
// X_{i + 1} = X_{i} + V_{i + 1/2} * h
// V_{i + 1} = V_{i + 1/2} + Acc[i + 1] * h / 2

class FRTClothSystem_Leapfrog_CPU : public FRTClothSystemBase
{
public:
	explicit FRTClothSystem_Leapfrog_CPU() = default;

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
	
	// A_{i-1}
	TArray<FVector> Pre_As;

	TArray<FVector> Velocities;
	
	// V_{i-0.5}
	TArray<FVector> Velocities_Half;

};