#include "FRTClothSystem.h"

#include <unordered_map>

void FRTClothSystem::Init (
	std::shared_ptr<FClothRawMesh> const& AMesh,
	FRTClothPhysicalMaterial<float> const& Material,
	std::shared_ptr<IRTClothSolver<float>> const& ASolver
)
{
	UpdateMaterial(Material);
    _UpdateMesh(AMesh);
    Solver = ASolver;
    //
    PrepareSimulation();
}

void FRTClothSystem::UpdateMaterial(FRTClothPhysicalMaterial<float> const& M)
{
    M_Material = M;
    IsFirstFrame = true;
}

void FRTClothSystem::UpdateMesh(std::shared_ptr<FClothRawMesh> const&AMesh)
{
    _UpdateMesh(AMesh);
    PrepareSimulation();
}

void FRTClothSystem::_UpdateMesh(std::shared_ptr<FClothRawMesh> const&AMesh)
{
    IsFirstFrame = true;
	Mesh = AMesh;
    Masses.SetNumZeroed(Mesh->Positions.Num());
	MakeDirectedEdgeModel();
    // setup shear and stretch conditions
    ShearConditions.Reserve(this->Mesh->Indices.Num() / 3);
    StretchConditions.Reserve(this->Mesh->Indices.Num() / 3);
    for (int32 i = 0; i < this->Mesh->Indices.Num() / 3; i ++)
    {
        auto &F = getFaceAt(i);
        ShearConditions.Add({Mesh.get(), F.vertex_index[0], F.vertex_index[1], F.vertex_index[2]});
        StretchConditions.Add({Mesh.get(), F.vertex_index[0], F.vertex_index[1], F.vertex_index[2], 100.f, 100.f});
    }

    // update masses
    for (int32 FaceID = 0; FaceID < Mesh->Indices.Num(); FaceID += 3)
    {
        uint32 const V0 = Mesh->Indices[FaceID];
        uint32 const V1 = Mesh->Indices[FaceID + 1];
        uint32 const V2 = Mesh->Indices[FaceID + 2];
        auto E01 = Mesh->TexCoords[V1] - Mesh->TexCoords[V0];
        auto E02 = Mesh->TexCoords[V2] - Mesh->TexCoords[V0];
        float const Mass = 0.5 * M_Material.Density * abs(FVector2D::CrossProduct(E01, E02));
        Masses[V0] += Mass / 3;
        Masses[V1] += Mass / 3;
        Masses[V2] += Mass / 3;
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
                BendConditions.Add({V0, V1, V2, V3});
                VisitedEdges.Add(Edge);
                VisitedEdges.Add(OtherE);
            }
        }
    }
}

void FRTClothSystem::TickOnce(float Duration)
{
    if (!IsFirstFrame)
    {
        auto Timer = FPlatformTime::Seconds();
        ForcesAndDerivatives();
        Timer = (FPlatformTime::Seconds() - Timer) * 1000.0;
        UE_LOG(LogTemp, Warning, TEXT("ForcesAndDerivatives: %f"), Timer);
    }
    IsFirstFrame = false;
    // // build up equation for solver, A x = b
    // // A = M -dfdx * dt * dt - dfdv * dt;
    // // b = f * dt + dfdx * v * dt * dt;
    // Df_Dx.Execute(A, Df_Dv,
    //     [this, Duration](int32 Id, float This_Value, float Other_Value)
    //     {
    //         return this->Masses[Id / 3] - (This_Value * Duration + Other_Value) * Duration;
    //     },
    //     [Duration](float This_Value, float Other_Value)
    //     {
    //         return - (This_Value * Duration + Other_Value) * Duration;
    //     });
    //
    // Df_Dx.MulVector(B.GetData(), (float *)Velocity.GetData(), Velocity.Num() * 3);
    // for (int32 i = 0; i < Forces.Num(); i ++)
    // {
    //     B[3 * i] = (B[3 * i] * Duration + Forces[i][0]) * Duration;
    //     B[3 * i + 1] = (B[3 * i + 1] * Duration + Forces[i][1]) * Duration;
    //     B[3 * i + 2] = (B[3 * i + 2] * Duration + Forces[i][2]) * Duration;
    // }
    // // load constraints
    // TArray<uint32> ConsIds;
    // Constraints.GetKeys(ConsIds);
    // TArray<FRTMatrix3> ConsMats;
    // ConsMats.Reserve(ConsIds.Num());
    // for (auto const Id : ConsIds)
    //     ConsMats.Add(Constraints[Id]);
    // Solver->UpdateConstraints(ConsIds, ConsMats);
    // // solve equation
    // TArray<float> dV;
    // dV.SetNumZeroed(Velocity.Num() * 3);
    // Solver->Solve(A, B, dV);

    // update position
    for (int32 i = 0; i < Velocity.Num(); i ++)
    {
        const FVector Acc = Forces[i] / Masses[i];
        FVector Dv = Acc * Duration * 0.5;
        if (Constraints.Contains(i))
        {
            Dv = Constraints[i] * Dv;
        }
        Velocity[i] += Dv;
        Mesh->Positions[i] += Velocity[i] * Duration;
        Velocity[i] += Dv;
    }
}

void FRTClothSystem::MakeDirectedEdgeModel()
{
    this->first_directed_edge_of_vertex.SetNumZeroed(this->Mesh->Positions.Num());
    for (auto &x : this->first_directed_edge_of_vertex)
        x = UNKNOWN_HALF_EDGE;
    this->other_half_of_edge.SetNumUninitialized(this->Mesh->Indices.Num());
    for (auto &x : this->other_half_of_edge)
        x = UNKNOWN_HALF_EDGE;

    // find out first directed edge and half edge
    // iterate over all faces
    std::unordered_map<HalfEdge, HalfEdgeRef, _halfedge_hashfunc, _halfedge_eqfunc> edge2edgeindex_map;
    for (int32 face_index = 0; face_index < this->Mesh->Indices.Num() / 3; face_index ++) {
        Face face = getFaceAt(face_index);
        // iterate over all three vertices on this face, this loop is O(3)
        for (unsigned int i = 0; i < 3; i ++) {
            // get vertex index and to_edge index, O(1)
            unsigned int const vertex_index = face.vertex_index[i];
            HalfEdgeRef to_edge_ref = 3 * face_index + (i + 1) % 3;

            // update first directed edge, O(1)
            if (first_directed_edge_of_vertex[vertex_index] == UNKNOWN_HALF_EDGE) {     
                first_directed_edge_of_vertex[vertex_index] = to_edge_ref;
            }
            // init edge as key, O(1)
            HalfEdge to_edge({vertex_index, face.vertex_index[(i + 1) % 3]});
            HalfEdge other_edge({to_edge.vertex_to, to_edge.vertex_from});
            auto p = edge2edgeindex_map.find(to_edge);

            // check if the half edge is correct or not, O(1)
            if ( p == edge2edgeindex_map.end()) {
                // update map
                edge2edgeindex_map[to_edge] = to_edge_ref;
            }

            // try to find out the other_edge, O(1)
            p = edge2edgeindex_map.find(other_edge);
            if (p != edge2edgeindex_map.end()) {
                // find the half-edge pair, just update, , O(1)
                other_half_of_edge[to_edge_ref] = p->second;
                other_half_of_edge[p->second] = to_edge_ref;
            }
        }
    }
}

void FRTClothSystem::ForcesAndDerivatives()
{
    for (int32 i = 0; i < Forces.Num(); i ++)
    {
        Forces[i].Set(0, 0, -10 * Masses[i]);
    }
    for (auto &D : DampingForces)
    {
        D.Set(0, 0, 0);
    }
    Df_Dx.SetValues(0.f);
    Df_Dv.SetValues(0.f);
    for (auto &Con : StretchConditions)
    {
        Con.ComputeForces(Mesh->Positions, Velocity, Mesh->TexCoords
            , M_Material.K_Stretch, M_Material.D_Stretch, Forces, Df_Dx,
            DampingForces, Df_Dx, Df_Dv);
    }
    if (M_Material.K_Shear > 0.01)
    {
        for (auto &Con : ShearConditions)
        {
            Con.ComputeForces(Mesh->Positions, Velocity, Mesh->TexCoords
                , M_Material.K_Shear, M_Material.D_Shear, Forces, Df_Dx,
                DampingForces, Df_Dx, Df_Dv);
        }
    }
    
    if (M_Material.K_Bend > 0.01)
    {
        for (auto &Con : BendConditions)
        {
            Con.ComputeForces(Mesh->Positions, Velocity, Mesh->TexCoords
                , M_Material.K_Bend, M_Material.D_Bend, Forces, Df_Dx,
                DampingForces, Df_Dx, Df_Dv);
        }
    }
    
}

void FRTClothSystem::PrepareSimulation()
{
    // set up runtime simulation variables
    Velocity.SetNumZeroed(Mesh->Positions.Num());
    Forces.SetNumZeroed(Mesh->Positions.Num());
    DampingForces.SetNumZeroed(Mesh->Positions.Num());

    // set up sparse matrix
    Df_Dx.UpdateSize(Mesh->Positions.Num() * 3);
    Df_Dv.UpdateSize(Mesh->Positions.Num() * 3);
    ForcesAndDerivatives();
    Df_Dx.LockPattern();
    Df_Dv.LockPattern();

    // should have same pattern
    check(Df_Dx.HasSamePattern(Df_Dv));

    // set up layout of A
    A = FRTBBSSMatrix<float>::MatrixFromOtherPattern(Df_Dx);

    // alloc memory for B
    B.SetNumUninitialized(Mesh->Positions.Num() * 3);

    // prepare solver
    Solver->Init(A);
}