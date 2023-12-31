﻿#include "FRTClothSystemBase.h"

#include <unordered_map>

#include <FBVHTree.h>
using namespace RTCloth;

DECLARE_STATS_GROUP(TEXT("RTCloth(Collision)"), STATGROUP_RTCloth_Collision, STATCAT_Advanced);
DECLARE_CYCLE_STAT(TEXT("Build BVH"), STAT_BUILD_BVH, STATGROUP_RTCloth_Collision);
DECLARE_CYCLE_STAT(TEXT("Solve Self Collision"), STAT_SOLVE_Self_COLLISION,STATGROUP_RTCloth_Collision);
DECLARE_CYCLE_STAT(TEXT("Solve External Collision"), STAT_SOLVE_External_COLLISION,STATGROUP_RTCloth_Collision);

void FRTClothSystemBase::Init (
	std::shared_ptr<FClothRawMesh> const& AMesh,
	FRTClothPhysicalMaterial<float> const& Material
)
{
	UpdateMaterial(Material);
	UpdateMesh(AMesh);
}

void FRTClothSystemBase::UpdateMaterial(FRTClothPhysicalMaterial<float> const& M)
{
	M_Material = M;
}

void FRTClothSystemBase::UpdateMesh(std::shared_ptr<FClothRawMesh> const&AMesh)
{
	_UpdateMesh(AMesh);
	PrepareSimulation();
}

void FRTClothSystemBase::TickOnce(float Duration)
{
    CurrentBox.Max = Mesh->Positions[0];
    CurrentBox.Min = Mesh->Positions[0];
    for (auto &pos: Mesh->Positions)
    {
        for (int i = 0; i < 3; i ++)
        {
            CurrentBox.Max[i] = std::max(CurrentBox.Max[i], pos[i]);
            CurrentBox.Min[i] = std::min(CurrentBox.Min[i], pos[i]);
        }
    }
}


void FRTClothSystemBase::UpdatePositionDataTo(FRHICommandList &CmdList, FRTDynamicVertexBuffer &DstBuffer)
{
    // default implementation is to copy position into DstBuffer
    check(IsInRenderingThread());
    // upload to GPU, update position only
    void* VertexBufferData = RHILockVertexBuffer(DstBuffer.VertexBufferRHI, 0, DstBuffer.GetDataSize(), RLM_WriteOnly);
    FMemory::Memcpy(VertexBufferData, Mesh->Positions.GetData(), DstBuffer.GetDataSize());
    RHIUnlockVertexBuffer(DstBuffer.VertexBufferRHI);
}

void FRTClothSystemBase::_UpdateMesh(std::shared_ptr<FClothRawMesh> const&AMesh)
{
    Mesh = AMesh;
    Masses.SetNumZeroed(Mesh->Positions.Num());
    ExternalForces.SetNumZeroed(Mesh->Positions.Num());
    TriangleBoundingSpheres.SetNumZeroed(Mesh->Indices.Num() / 3);
    MakeDirectedEdgeModel();

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
    CurrentBox.Max = Mesh->Positions[0];
    CurrentBox.Min = Mesh->Positions[0];
    for (auto &pos: Mesh->Positions)
    {
        for (int i = 0; i < 3; i ++)
        {
            CurrentBox.Max[i] = std::max(CurrentBox.Max[i], pos[i]);
            CurrentBox.Min[i] = std::min(CurrentBox.Min[i], pos[i]);
        }
    }
    // // For T shirt constriants
    // auto Mid = (CurrentBox.Max + CurrentBox.Min) / 2;
    // for (int i = 0; i < other_half_of_edge.Num(); i ++)
    // {
    //     if (other_half_of_edge[i] == UNKNOWN_HALF_EDGE)
    //     {
    //         auto V1 = toVertexIndexOfHalfEdge(i);
    //         auto V2 = fromVertexIndexOfHalfEdge(i);
    //         if (Mesh->Positions[V1].Y < Mid.Y)
    //         {
    //             AddConstraint( V1,  {});
    //         }
    //         if (Mesh->Positions[V2].Y < Mid.Y)
    //         {
    //             AddConstraint( V2, {});
    //         }
    //         
    //     }
    // }
}

// refer to https://github.com/NANAnoo/A1_manifold_test/blob/main/manifold_test/DirectedEdge.cpp
void FRTClothSystemBase::MakeDirectedEdgeModel()
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
            HalfEdge to_edge({vertex_index, static_cast<uint32>(face.vertex_index[(i + 1) % 3])});
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

void FRTClothSystemBase::UpdateTriangleProperties(TArray<FVector> const&Positions, TArray<FVector> const&Velocities)
{
    for (int i = 0; i < Mesh->Indices.Num() / 3; i ++)
    {
        auto const F = getFaceAt(i);
        auto P0 = Positions[F.vertex_index[0]];
        auto P1 = Positions[F.vertex_index[1]];
        auto P2 = Positions[F.vertex_index[2]];
        auto Center = (P0 + P1 + P2) / 3;
        float Len = (Center - P0).Size();
        auto V = (Velocities[F.vertex_index[0]] +
            Velocities[F.vertex_index[1]] +
            Velocities[F.vertex_index[2]]) / 3;
        TriangleBoundingSpheres[i] = {Center, Len, V, i};
    }
}

void FRTClothSystemBase::UpdateTransform(FTransform const& ClothToWorld, float Duration)
{
    FVector const Pre_pos =  Mesh->LocalToWorld.GetLocation();
    FVector newClothAttachedVelocity = (ClothToWorld.GetLocation() - Pre_pos) / Duration;
    for (auto &Pair : Colliders)
    {
        Pair.Value.ClothToCollider =  ClothToWorld.ToMatrixNoScale() * Pair.Key->GetComponentToWorld().ToInverseMatrixWithScale();
        Pair.Value.ColliderToCloth =  Pair.Value.ClothToCollider.Inverse();
    }
    Gravity = ClothToWorld.InverseTransformVector(WorldSpaceGravity);
    WindVelocity = ClothToWorld.InverseTransformVector(WorldSpaceWindVelocity);
    newClothAttachedVelocity = -ClothToWorld.InverseTransformVector(newClothAttachedVelocity);
    DeltaClothAttachedVelocity = (newClothAttachedVelocity - ClothAttachedVelocity);
    ClothAttachedVelocity = newClothAttachedVelocity;
    Mesh->LocalToWorld = ClothToWorld;
}
void FRTClothSystemBase::AddCollisionSpringForces(TArray<FVector> &Forces, TArray<FVector> const&Positions, TArray<FVector> const&Velocities)
{
    // // build bvh tree
    TArray<FInnerCollisionBVHNode> BVH_Tree;
    BVH_Tree.Reserve(TriangleBoundingSpheres.Num() * 2);
    int Root = 0;
    {
        SCOPE_CYCLE_COUNTER(STAT_BUILD_BVH);
        Root = FInnerCollisionBVHNode::BuildFrom(TriangleBoundingSpheres, BVH_Tree, 0, TriangleBoundingSpheres.Num() - 1);
    }
    {
        SCOPE_CYCLE_COUNTER(STAT_SOLVE_Self_COLLISION);
        for (int V = 0; V < Mesh->Positions.Num(); V ++)
        {
            auto P = Positions[V];
            BVH_Tree[Root].Intersect(P, BVH_Tree, [this, P, V, &Forces, &Velocities](FHitSphere const&Sphere)
            {
                auto F = getFaceAt(Sphere.ID);
                if (F.vertex_index[0] == V || F.vertex_index[1] == V || F.vertex_index[2] == V) return;
                FVector const Center = Sphere.Center;
                float const Len = Sphere.Radius;
                auto C_P = P - Center;
                auto const CenterV = Sphere.Velocity;
                float const Dis = C_P.Size();
                if (Dis < Len)
                {
                    C_P.Normalize();
                    auto const f = Masses[V] * (C_P * (Len - Dis) * M_Material.K_Collision - (Velocities[V] - CenterV) * M_Material.D_Collision) ;
                    Forces[V] += f;
                }
            });
        }
    }
}

void FRTClothSystemBase::SolveCollision(TArray<FVector>& Pre_Positions, TArray<FVector>& Positions, TArray<FVector>& Velocities, float Duration)
{
    SCOPE_CYCLE_COUNTER(STAT_SOLVE_External_COLLISION);
    for (auto &Pair : Colliders)
    {
        FMatrix const& ClothToCollider = Pair.Value.ClothToCollider;
        FMatrix const& ColliderToCloth = Pair.Value.ColliderToCloth;
        for (int i = 0; i < Positions.Num(); i ++)
        {
            if (!Constraints.Contains(i))
            {
                // transform cloth positions and velocities to collider space
                FVector Vel = ClothToCollider.TransformVector(Velocities[i]);
                FVector4 Pos_W = ClothToCollider.TransformPosition(Positions[i]);
                FVector Pos = (Pos_W / Pos_W.W);
                if (Pair.Value.Collision(Pos, Vel))
                {
                    // transform velocity and position back to cloth space
                    Velocities[i] = ColliderToCloth.TransformVector(Vel);
                    Pos_W = ColliderToCloth.TransformPosition(Pos);
                    Positions[i] = (Pos_W / Pos_W.W);
                    if (Pre_Positions.Num() > 0)
                        // for Integrator that needs previous positions
                        Pre_Positions[i] = Positions[i] - Velocities[i] * Duration;
                }
            }
        }
    }
}
