#pragma once

#include "RTClothStructures.h"

#include "Math/FRTSparseMatrix.h"
#include "FRTShearCondition.h"
#include "FRTStretchCondition.h"
#include "FRTBendCondition.h"

#include <memory>

#include "FRTClothSolver.h"

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

	// mass per area for each triangle
	Real Density;
};

class FRTClothSystem
{
public:
	FRTClothSystem() : M_Material({0, 0, 0, 0, 0, 0, 0}){};
	// init system
	void Init(
		std::shared_ptr<FClothRawMesh> const& Mesh,
		FRTClothPhysicalMaterial<float> const& Material,
		std::shared_ptr<IRTClothSolver<float>> const& ASolver
	);

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

	// simulate for one tick
	void TickOnce(float Duration);

	const TArray<FVector> &CurrentPositions() const
	{
		return Mesh->Positions;
	}
	
private:
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
		unsigned int vertex_index[3];
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
	void MakeDirectedEdgeModel();
	// Directed Edge End

	// update Mesh, rebuild Conditions
	void _UpdateMesh(std::shared_ptr<FClothRawMesh> const&);
	
	// calculate forces and derivatives
	void ForcesAndDerivatives();

	// void setup sparse matrix and runtime variables
	void PrepareSimulation();
	
	// TODO : solve inner collision
	
	FRTClothPhysicalMaterial<float> M_Material;
	std::shared_ptr<FClothRawMesh> Mesh;

	// instant positions
	TArray<FVector> Current_Positions;

	// simulation states
	// TArray<FVector> M_Positions;
	TArray<FVector> M_Velocity;
	// TArray<FVector> M_UV;

	// pre computed conditions cache
	TArray<FRTStretchCondition> StretchConditions;
	TArray<FRTShearCondition> ShearConditions;
	TArray<FRTBendCondition> BendConditions;

	// forces and derivatives
	TArray<float> Masses;
	TArray<FVector> Forces;
	TArray<FVector> DampingForces;
	FRTBBSSMatrix<float> Df_Dx;
	FRTBBSSMatrix<float> Df_Dv;

	// For solvers x = A/B
	FRTBBSSMatrix<float> A;
	TArray<float> B;
	std::shared_ptr<IRTClothSolver<float>> Solver;
	
	// physics properties at each particle
	TArray<FVector> Velocity;

	// check if it's fist frame of the incoming mesh
	bool IsFirstFrame = false;

	// Constraints
	TMap<uint32, FRTMatrix3> Constraints;
};