#include "FRTClothSystem_Leapfrog_CPU.h"


void FRTClothSystem_Leapfrog_CPU::Acceleration()
{
	// set up gravity
	for (int32 i = 0; i < Masses.Num(); i ++)
	{
		Forces[i] = Masses[i] * Gravity;
	}

	// calculate forces
	for (auto &Con : StretchConditions)
	{
		Con.UpdateCondition(Mesh->Positions, Velocities, Mesh->TexCoords);
		Con.ComputeForces(M_Material.K_Stretch, M_Material.D_Stretch, Forces, Forces);
	}
	for (auto &Con : ShearConditions)
	{
		Con.UpdateCondition(Mesh->Positions, Velocities, Mesh->TexCoords);
		// Serious numerical un-stability meet while using original Shader Damping
		Con.ComputeForces(M_Material.K_Shear, 0, Forces, Forces);
	}
	for (auto &Con : BendConditions)
	{
		Con.UpdateCondition(Mesh->Positions, Velocities, Mesh->TexCoords);
		Con.ComputeForces(M_Material.K_Bend, M_Material.D_Bend, Forces, Forces);
	}
}

void FRTClothSystem_Leapfrog_CPU::PrepareSimulation()
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
	Forces.SetNumZeroed(Mesh->Positions.Num());
	Pre_As.SetNumZeroed(Mesh->Positions.Num());
	Velocities.SetNumZeroed(Mesh->Positions.Num());
	Velocities_Half.SetNumZeroed(Mesh->Positions.Num());

	// get initial Acceleration
	Acceleration();
}

void FRTClothSystem_Leapfrog_CPU::TickOnce(float Duration)
{
	FAutoTimer TI("FRTClothSystem_Leapfrog_CPU Tick");
	// Update Velocity_Half
	for (int32 i = 0; i < Masses.Num(); i ++)
	{
		if (Constraints.Contains(i))
		{
			Pre_As[i] = Constraints[i] * Pre_As[i];
		}
		Velocities_Half[i] = Velocities[i] + (Duration / 2) * Pre_As[i];
	}
	
	// Update Positions
	for (int32 i = 0; i < Masses.Num(); i ++)
	{
		Mesh->Positions[i] += Velocities_Half[i] * Duration;
		Velocities[i] += Duration * Pre_As[i];
	}

	// Get new acc and update V_{i+1}
	Acceleration();
	for (int32 i = 0; i < Masses.Num(); i ++)
	{
		auto const Acc = Forces[i] / Masses[i];
		Pre_As[i] = Acc;
		if (Constraints.Contains(i))
		{
			Pre_As[i] = Constraints[i] * Pre_As[i];
		}
		Velocities[i] = Velocities_Half[i] + (Duration / 2) * Pre_As[i];
	}
	TI.Tick("Update Positions");
}


