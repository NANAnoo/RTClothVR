﻿#include "FRTClothSystem_Verlet_CPU.h"

DECLARE_STATS_GROUP(TEXT("RTCloth(Verlet)"), STATGROUP_RTCloth_Verlet, STATCAT_Advanced);
DECLARE_CYCLE_STAT(TEXT("One Frame Cost"), TIME_COST_Verlet, STATGROUP_RTCloth_Verlet);
DECLARE_CYCLE_STAT(TEXT("StretchConditions"), StretchConditions_Verlet,STATGROUP_RTCloth_Verlet);
DECLARE_CYCLE_STAT(TEXT("ShearConditions"), ShearConditions_Verlet,STATGROUP_RTCloth_Verlet);
DECLARE_CYCLE_STAT(TEXT("BendConditions"), BendConditions_Verlet,STATGROUP_RTCloth_Verlet);
DECLARE_CYCLE_STAT(TEXT("Integration"), Integration_Verlet,STATGROUP_RTCloth_Verlet);

void FRTClothSystem_Verlet_CPU::Acceleration()
{
	// calculate forces
	{
		SCOPE_CYCLE_COUNTER(StretchConditions_Verlet)
		for (auto &Con : StretchConditions)
		{
			Con.UpdateCondition(Mesh->Positions, Velocities, Mesh->TexCoords);
			Con.ComputeForces(M_Material.K_Stretch, M_Material.D_Stretch, Forces, Forces);
		}
	}
	{
		SCOPE_CYCLE_COUNTER(ShearConditions_Verlet)
		for (auto &Con : ShearConditions)
		{
			Con.UpdateCondition(Mesh->Positions, Velocities, Mesh->TexCoords);
			// Serious numerical un-stability meet while using original Shader Damping
			Con.ComputeForces(M_Material.K_Shear, M_Material.D_Shear, Forces, Forces);
		}
	}
	{
		SCOPE_CYCLE_COUNTER(BendConditions_Verlet)
		for (auto &Con : BendConditions)
		{
			Con.UpdateCondition(Mesh->Positions, Velocities, Mesh->TexCoords);
			Con.ComputeForces(M_Material.K_Bend, M_Material.D_Bend, Forces, Forces);
		}
	}
	for (int i = 0; i < Masses.Num(); i ++)
	{
		Forces[i] += - M_Material.AirFriction * (Velocities[i] - WindVelocity) * Masses[i];
	}
}

void FRTClothSystem_Verlet_CPU::PrepareSimulation()
{
	// setup shear and stretch conditions
	ShearConditions.Reserve(this->Mesh->Indices.Num() / 3);
	StretchConditions.Reserve(this->Mesh->Indices.Num() / 3);
	for (int32 i = 0; i < this->Mesh->Indices.Num() / 3; i ++)
	{
		auto &F = getFaceAt(i);
		ShearConditions.Add({Mesh.get(), F.vertex_index[0], F.vertex_index[1], F.vertex_index[2]});
		StretchConditions.Add({Mesh.get(), F.vertex_index[0], F.vertex_index[1], F.vertex_index[2], M_Material.Rest_U, M_Material.Rest_V});
	}
	// set up bend conditions
	uint32 PairNum = 0;
	for (auto const E : other_half_of_edge)
	{
		PairNum += (E != UNKNOWN_HALF_EDGE);
	}
	BendConditions.Reserve(PairNum / 2);

	TSet<uint32> VisitedEdges;
	VisitedEdges.Reserve(other_half_of_edge.Num());
	for (auto const Edge : other_half_of_edge)
	{
        
		if (Edge != UNKNOWN_HALF_EDGE && !VisitedEdges.Contains(Edge))
		{
			HalfEdgeRef const OtherE = otherHalfEdge(Edge);
			if (!VisitedEdges.Contains(OtherE))
			{
				uint32 const V0 = toVertexIndexOfHalfEdge(nextHalfEdge(Edge));
				uint32 const V1 = fromVertexIndexOfHalfEdge(Edge);
				uint32 const V2 = toVertexIndexOfHalfEdge(Edge);
				uint32 const V3 = toVertexIndexOfHalfEdge(nextHalfEdge(OtherE));
				BendConditions.Add({V0, V1, V2, V3, M_Material.InitTheta});
				VisitedEdges.Add(Edge);
				VisitedEdges.Add(OtherE);
			}
		}
	}

	// set up runtime simulation variables
	Pre_Positions = Mesh->Positions;
	Forces.SetNumZeroed(Mesh->Positions.Num());
	Velocities.SetNumZeroed(Mesh->Positions.Num());
}

void FRTClothSystem_Verlet_CPU::TickOnce(float Duration)
{
	SCOPE_CYCLE_COUNTER(TIME_COST_Verlet);
	FRTClothSystemBase::TickOnce(Duration);
	// set up gravity
	for (int32 i = 0; i < Masses.Num(); i ++)
	{
		Forces[i] = Masses[i] * Gravity;
	}
	if (M_Material.EnableInnerCollision)
	{
		UpdateTriangleProperties(Mesh->Positions, Velocities);
		AddCollisionSpringForces(Forces, Mesh->Positions, Velocities);
	}
	Acceleration();
	// integration
	{
		SCOPE_CYCLE_COUNTER(Integration_Verlet)
		for (int32 i = 0; i < Masses.Num(); i ++)
		{
			auto Acc =  Forces[i] / Masses[i] + DeltaClothAttachedVelocity / Duration;
			if (Constraints.Contains(i))
			{
				Acc = Constraints[i] * Acc;
			}
			auto const Pos_Old = Mesh->Positions[i];
			Mesh->Positions[i] = 2 * Mesh->Positions[i] - Pre_Positions[i] + Acc * (Duration * Duration);
			Velocities[i] = (Mesh->Positions[i] - Pos_Old) / (2 * Duration);
			Pre_Positions[i] = Pos_Old;
		}
	}
	if (M_Material.EnableCollision)
		SolveCollision(Pre_Positions, Mesh->Positions, Velocities, Duration);
}


