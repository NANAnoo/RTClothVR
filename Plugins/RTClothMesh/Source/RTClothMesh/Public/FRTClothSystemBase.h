#pragma once

#include "RTClothStructures.h"

#include <memory>

#include "FRTClothSolver.h"
#include "FRTDynamicVertexBuffer.h"

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
	Real K_Shear;
	Real D_Shear;

	// rest U V
	Real Rest_U;
	Real Rest_V;

	// mass per area for each triangle
	Real Density;
};

class FRTClothSystemBase
{
public:
	FRTClothSystemBase() : M_Material({0, 0, 0, 0, 0, 0, 0, 0, 0}){};

	virtual ~FRTClothSystemBase() {}
	
	// init system
	void Init(
		std::shared_ptr<FClothRawMesh> const& Mesh,
		FRTClothPhysicalMaterial<float> const& Material
	);

	FORCEINLINE void SetGravity(FVector const&G)
	{
		Gravity = G;
	}

	void AddConstraint(uint32 Id, FClothConstraint const& Constraint)
	{
		Constraints.Add(Id, Constraint.ConstraintsMat());
	}
	void RemoveConstraint(uint32 Id)
	{
		Constraints.Remove(Id);
	}

	// TODO: set up Collision bodies
	// void AddCollider();

	// update material
	void UpdateMaterial(FRTClothPhysicalMaterial<float> const&M);

	// update Mesh, rebuild Conditions
	void UpdateMesh(std::shared_ptr<FClothRawMesh> const&);

	const TArray<FVector> &CurrentPositions() const
	{
		return Mesh->Positions;
	}

	// simulate for one tick
	virtual void TickOnce(float Duration) = 0;

	// update data into DstBuffer
	virtual void UpdatePositionDataTo(FRTDynamicVertexBuffer &DstBuffer);
	
protected:

	// setup runtime variables, before tick
	virtual void PrepareSimulation() = 0;
	
	// Directed Edge Structure
	typedef unsigned int HalfEdgeRef;
	const HalfEdgeRef UNKNOWN_HALF_EDGE = std::numeric_limits<unsigned int>::max();
	struct Vertex
	{
		float x = 0;
		float y = 0;
		float z = 0;
	};
	// same as that in FaceIndex, record vertices index in CCW
	struct Face
	{
		int vertex_index[3];
	};

	// temporary structure, used only in hash map
	struct HalfEdge
	{
		unsigned int vertex_from;
		unsigned int vertex_to;
	};

	/* structure used to build hash map */
	struct _halfedge_hashfunc
	{
		size_t operator()(const HalfEdge e) const
		{
			return size_t(e.vertex_from*14514+e.vertex_to*19);
		}	
	};
	struct _halfedge_eqfunc
	{
		bool operator()(const HalfEdge e1, HalfEdge e2) const
		{
			return (e1.vertex_from == e2.vertex_from && e1.vertex_to == e2.vertex_to);
		}	
	};
	
	const Face& getFaceAt(unsigned int index) const { return ((Face *)Mesh->Indices.GetData())[index]; }
    const Vertex& getVertexAt(unsigned int index) const { return ((Vertex *)Mesh->Positions.GetData())[index]; }

    // ------------------------------all methods below this line are O(1)------------------------------------------------------
    // get index of the face that the edge is on
    unsigned int static faceIndexOfHalfEdge(HalfEdgeRef edge) { return edge / 3;}

    // get the from vertex index of the edge
    // edge : [from vertex] -> to vertex
    unsigned int fromVertexIndexOfHalfEdge(HalfEdgeRef edge) const {
        return getFaceAt(faceIndexOfHalfEdge(edge)).vertex_index[(edge + 2) % 3];
    }

    // get the to vertex index of the edge
    // edge : from vertex -> [to vertex]
    unsigned int toVertexIndexOfHalfEdge(HalfEdgeRef edge) const {
        return ((Face *)Mesh->Indices.GetData())[faceIndexOfHalfEdge(edge)].vertex_index[edge % 3];
    }
    
    // first half edge of the vertex
    // first falf edge is the edge that follows the vertex
    // For example: 
    //          first_falf_edge(vertex_1) == half_edge_2
    //..........................................................
    //....[vertex_0]<-------(half_edge_0)-----------[vertex_2]..
    //.........\........................................->......
    //..........\....................................../........
    //.......(half_edge_1).........................(half_edge_2)
    //............\................................../..........
    //.............\------->.[vertex_1]-------------/...........
    //..........................................................
    HalfEdgeRef firstDirectedHalfEdgeOnVertex(unsigned int vertex_index) { return first_directed_edge_of_vertex[vertex_index];}
    HalfEdgeRef static nextHalfEdge(HalfEdgeRef edge) { return (edge + 1) % 3 + 3 * faceIndexOfHalfEdge(edge); }
    HalfEdgeRef static prevHalfEdge(HalfEdgeRef edge) { return (edge + 2) % 3 + 3 * faceIndexOfHalfEdge(edge); }
    HalfEdgeRef otherHalfEdge(HalfEdgeRef edge) { return other_half_of_edge[edge]; }

	/* store the first directed edge*/
	TArray<HalfEdgeRef> first_directed_edge_of_vertex;

	/* store the opposite edge of each edge*/
	TArray<HalfEdgeRef> other_half_of_edge;

	// Make Directed Edge
	FORCEINLINE void MakeDirectedEdgeModel();
	// Directed Edge End

	// update Mesh, rebuild Conditions
	void _UpdateMesh(std::shared_ptr<FClothRawMesh> const&);
	
	FRTClothPhysicalMaterial<float> M_Material;
	std::shared_ptr<FClothRawMesh> Mesh;

	// forces and derivatives
	TArray<float> Masses;

	// Constraints
	TMap<uint32, FRTMatrix3> Constraints;

	// Gravity
	FVector Gravity = {0, 0, 0};

	TArray<FVector> ExternalForces;
};