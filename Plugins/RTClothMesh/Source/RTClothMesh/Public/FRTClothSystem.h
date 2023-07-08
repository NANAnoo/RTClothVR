#pragma once

#include "RTClothStructures.h"
#include "Math/FRTSparseMatrix.h"

#include <memory>

// the simulation parameters
template <typename  Real>
struct FRTClothPhysicalMaterial
{
	// for bend condition
	Real K_Bend;
	Real D_Bend;

	// for stretch condition
	Real K_Stretch;
	Real D_Stretch;

	// for shear condition
	Real D_Shear;
	Real K_Shear;

	// mass per area for each triangle
	Real Density;
};

class FRTClothSystem
{
public:
	FRTClothSystem() : M_Material({0, 0, 0, 0, 0, 0, 0}){};
	// build up condition W
	void InitWithClothMesh(std::shared_ptr<FClothRawMesh> const& Mesh, FRTClothPhysicalMaterial<float> const& Material);

	// TODO: set up constraints
	// void UpdateConstrains(std::vector<IConstrains *> const& Constrains);

	// TODO: set up Collision bodies
	// void AddCollider();

	// update material
	void UpdateMaterial(FRTClothPhysicalMaterial<float> const&);

	// simulate for one tick
	void TickOnce(float Duration)
	{
	}

	const TArray<FVector> &CurrentPositions();
	
private:
	// void InitConditions();
	// TODO : solve inner collision
	
	FRTClothPhysicalMaterial<float> M_Material;
	std::shared_ptr<FClothRawMesh> Mesh;

	// instant positions
	TArray<FVector> Current_Positions;

	// simulation states
	TArray<FVector> M_Positions;
	TArray<FVector> M_Velocity;
	TArray<FVector> M_UV;

	// pre computed conditions cache
};