#include "FRTClothSystemBase.h"

#include <unordered_map>

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

void FRTClothSystemBase::UpdatePositionDataTo(FRTDynamicVertexBuffer &DstBuffer)
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