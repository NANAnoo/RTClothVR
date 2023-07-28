#include "FRTClothSystemBase.h"

#include <unordered_map>

#include <functional>

DECLARE_STATS_GROUP(TEXT("RTCloth(Collision)"), STATGROUP_RTCloth_Collision, STATCAT_Advanced);
DECLARE_CYCLE_STAT(TEXT("Build BVH"), STAT_BUILD_BVH, STATGROUP_RTCloth_Collision);
DECLARE_CYCLE_STAT(TEXT("Solve Collision"), STAT_SOLVE_COLLISION,STATGROUP_RTCloth_Collision);

struct AABB
{
    bool Contains(FVector const& Pos) const
    {
        return Pos[0] < Max[0] && Pos[0] > Min[0] &&
           Pos[1] < Max[1] && Pos[1] > Min[1] &&
           Pos[2] < Max[2] && Pos[2] > Min[2];
    }
    bool Intersect(AABB const&other) const
    {
        return Contains(other.Max) || Contains(other.Min) || other.Contains(Max) || other.Contains(Min);
    }
    AABB operator+(AABB const&Other)
    {
        AABB Result;
        Result.Max[0] = std::max(Max[0], Other.Max[0]);
        Result.Max[1] = std::max(Max[1], Other.Max[1]);
        Result.Max[2] = std::max(Max[2], Other.Max[2]);

        Result.Min[0] = std::min(Min[0], Other.Min[0]);
        Result.Min[1] = std::min(Min[1], Other.Min[1]);
        Result.Min[2] = std::min(Min[2], Other.Min[2]);

        return Result;
    }
    FVector Max;
    FVector Min;
};

template<typename T, typename AABB_builder>
struct BVH_Node
{
    explicit BVH_Node(T const& Obj) : box(AABB_builder{}(Obj)), value(Obj)
    {
    } 
    explicit BVH_Node(AABB const& Box)
    {
        box = Box;
    }

    static int BuildFrom(TArray<T> const&Objects, TArray<BVH_Node<T, AABB_builder>>&Result, int Begin, int End)
    {
        if (End == Begin)
        {
            BVH_Node NewNode(Objects[Begin]);
            Result.Add(NewNode);
        } else
        {
            int Mid = (Begin + End) >> 1;
            int L = BuildFrom(Objects, Result, Begin, Mid);
            int R = BuildFrom(Objects, Result, Mid + 1, End);
            BVH_Node NewNode(Result[L].box + Result[R].box);
            NewNode.L = L;
            NewNode.R = R;
            Result.Add(NewNode);
        }
        return Result.Num() - 1;
    }
    
    void Intersect(FVector const&Pos, TArray<BVH_Node> const& Nodes, std::function<void(T const&Value)> const&Handler) const
    {
        if (box.Contains(Pos))
        {
            if (L == -1 && R == -1)
            {
                Handler(value);
            } else
            {
                if (L >= 0)
                    Nodes[L].Intersect(Pos, Nodes, Handler);
                if (R >= 0)
                    Nodes[R].Intersect(Pos, Nodes, Handler);
            }
            
        }
    }
    int L = -1;
    int R = -1;
    AABB box;
    T value;
};

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
    TriangleBoundingSpheres.SetNumZeroed(Mesh->Indices.Num());
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
}

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

void FRTClothSystemBase::AddCollisionSpringForces(TArray<FVector> &Forces, TArray<FVector> const&Positions, TArray<FVector> const&Velocities)
{
    struct FHitSphereToAABB
    {
        AABB operator()(FHitSphere const&Sphere) const
        {
            FVector const Diff(Sphere.Radius, Sphere.Radius, Sphere.Radius);
            return {Sphere.Center + Diff, Sphere.Center - Diff};
        }
    };
    // // build bvh tree
    TArray<BVH_Node<FHitSphere, FHitSphereToAABB>> BVH_Tree;
    BVH_Tree.Reserve(TriangleBoundingSpheres.Num() * 2);
    int Root = 0;
    {
        SCOPE_CYCLE_COUNTER(STAT_BUILD_BVH);
        Root = BVH_Node<FHitSphere, FHitSphereToAABB>::BuildFrom(TriangleBoundingSpheres, BVH_Tree, 0, TriangleBoundingSpheres.Num() - 1);
    }
    {
        SCOPE_CYCLE_COUNTER(STAT_SOLVE_COLLISION);
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
                    auto f = Masses[V] * (C_P * (Len - Dis) * M_Material.K_Collision - (Velocities[V] - CenterV) * M_Material.D_Collision) ;
                    Forces[V] += f;
                    f = f / (-3);
                    Forces[F.vertex_index[0]] += f;
                    Forces[F.vertex_index[1]] += f;
                    Forces[F.vertex_index[2]] += f;
                }
            });
        }
    }
}