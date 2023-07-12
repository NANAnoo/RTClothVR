#include "FRTClothSystem.h"

#include <unordered_map>

void FRTClothSystem::Init (
	std::shared_ptr<FClothRawMesh> const& AMesh,
	FRTClothPhysicalMaterial<float> const& Material,
	std::shared_ptr<IRTClothSolver<float>> const& ASolver
)
{
	_UpdateMesh(AMesh);
	UpdateMaterial(Material);
    Solver = ASolver;
    
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
        StretchConditions.Add({Mesh.get(), F.vertex_index[0], F.vertex_index[1], F.vertex_index[2], 1.f, 1.f});
    }
    
    // set up bend conditions
    uint32 PairNum = 0;
    for (auto const E : other_half_of_edge)
    {
        PairNum += (E != UNKNOWN_HALF_EDGE);
    }
    BendConditions.Reserve(PairNum / 2);
    // get all triangle pairs, BFS
    TSet<uint32> VisitedFaces;
    VisitedFaces.Reserve(this->Mesh->Indices.Num() / 3);
    TQueue<uint32> ToVisit;
    ToVisit.Enqueue(0);
    while (!ToVisit.IsEmpty())
    {
        uint32 NextFace;
        if (ToVisit.Dequeue(NextFace))
        {
            VisitedFaces.Add(NextFace);
            auto const F = getFaceAt(NextFace);
            FVector E01 = Mesh->Positions[F.vertex_index[1]] - Mesh->Positions[F.vertex_index[0]];
            FVector E02 = Mesh->Positions[F.vertex_index[2]] - Mesh->Positions[F.vertex_index[0]];
            float Mass = 0.5 * M_Material.Density * FVector::CrossProduct(E01, E02).Size();
            for (uint32 i = 0; i < 3; i ++)
            {
                auto const VID = F.vertex_index[i];
                Masses[VID] += Mass / 3;
                HalfEdgeRef const Edge = firstDirectedHalfEdgeOnVertex(VID);
                HalfEdgeRef const OtherE = otherHalfEdge(Edge);
                if (OtherE != UNKNOWN_HALF_EDGE)
                {
                    auto const OtherF = faceIndexOfHalfEdge(OtherE);
                    if (!VisitedFaces.Contains(OtherF))
                    {
                        uint32 const V0 = toVertexIndexOfHalfEdge(nextHalfEdge(Edge));
                        uint32 const V1 = fromVertexIndexOfHalfEdge(Edge);
                        uint32 const V2 = toVertexIndexOfHalfEdge(Edge);
                        uint32 const V3 = toVertexIndexOfHalfEdge(nextHalfEdge(OtherE));
                        BendConditions.Add({V0, V1, V2, V3});
                        ToVisit.Enqueue(OtherF);
                    }
                }
            }
        }
    }
}

void FRTClothSystem::TickOnce(float Duration)
{
    if (!IsFirstFrame)
    {
        ForcesAndDerivatives();
    }
    IsFirstFrame = false;
    // build up equation for solver, A x = b
    // A = M -dfdx * dt * dt - dfdv * dt;
    // b = f * dt + dfdx * v * dt * dt;
    Df_Dx.Execute(A, Df_Dv,
        [this, Duration](int32 Id, float A, float B) {return this->Masses[Id / 3] - (A * Duration + B) * Duration;},
        [Duration](float A, float B) {return - (A * Duration + B) * Duration;});
    
    Df_Dx.MulVector(B.GetData(), (float *)Velocity.GetData(), Velocity.Num() * 3);
    for (int32 i = 0; i < Forces.Num(); i ++)
    {
        B[3 * i] = (B[3 * i] * Duration + Forces[i][0]) * Duration;
        B[3 * i + 1] = (B[3 * i + 1] * Duration + Forces[i][0]) * Duration;
        B[3 * i + 2] = (B[3 * i + 2] * Duration + Forces[i][0]) * Duration;
    }
    // load constraints
    TArray<uint32> ConsIds;
    Constraints.GetKeys(ConsIds);
    TArray<FRTMatrix3> ConsMats;
    ConsMats.Reserve(ConsIds.Num());
    for (auto const Id : ConsIds)
        ConsMats.Add(Constraints[Id]);
    Solver->UpdateConstraints(ConsIds, ConsMats);
    // solve equation
    TArray<float> dV;
    dV.SetNumZeroed(Velocity.Num() * 3);
    Solver->Solve(A, B, dV);

    // update position
    for (int32 i = 0; i < Velocity.Num(); i ++)
    {
        Velocity[i] += {dV[3 * i], dV[3 * i + 1], dV[3 * i + 2]};
        Mesh->Positions[i] += Velocity[i];
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
    for (auto &Con : StretchConditions)
    {
        Con.ComputeForces(Mesh->Positions, Velocity, Mesh->TexCoords
            , M_Material.K_Stretch, M_Material.D_Stretch, Forces, Df_Dx,
            DampingForces, Df_Dx, Df_Dv);
    }

    for (auto &Con : ShearConditions)
    {
        Con.ComputeForces(Mesh->Positions, Velocity, Mesh->TexCoords
            , M_Material.K_Shear, M_Material.D_Shear, Forces, Df_Dx,
            DampingForces, Df_Dx, Df_Dv);
    }

    for (auto &Con : BendConditions)
    {
        Con.ComputeForces(Mesh->Positions, Velocity, Mesh->TexCoords
            , M_Material.K_Bend, M_Material.D_Bend, Forces, Df_Dx,
            DampingForces, Df_Dx, Df_Dv);
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