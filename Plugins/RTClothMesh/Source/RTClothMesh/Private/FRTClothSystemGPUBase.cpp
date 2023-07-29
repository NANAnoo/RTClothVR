#include "FRTClothSystemGPUBase.h"

#include "FStretchShearConditionCS.h"
#include "FLocalForceAssemplerCS.h"
#include "FVerletIntegratorCS.h"
#include "FRTBendForceCS.h"
#include "FInnerCollisionForcesCS.h"

IMPLEMENT_GLOBAL_SHADER_PARAMETER_STRUCT(FRTClothSimulationParameters, "RTClothMaterialUB");

void FRTClothSystemGPUBase::PrepareSimulation()
{
	// set up simulation variables
	Indices.Reserve(this->Mesh->Indices.Num());
	for (auto const &Idx : this->Mesh->Indices) Indices.Add(Idx);
	ConditionBasis.Reserve(this->Mesh->Indices.Num() / 3);
	InnerHitBVH.SetNum(this->Mesh->Indices.Num() / 3 * 2 - 1);
	Velocities.SetNum(this->Mesh->Positions.Num());
	LocalForces.SetNum(this->Mesh->Indices.Num());
	GlobalForces.SetNum(Mesh->Positions.Num());
	Pre_Positions.Reserve(this->Mesh->Positions.Num());
	Positions.Reserve(this->Mesh->Positions.Num());
	VertMasses.Reserve(this->Mesh->Positions.Num());
	TArray<TArray<int>> SubForceRefsMap;
	SubForceRefsMap.SetNumZeroed(this->Mesh->Positions.Num());
	for (int32 i = 0; i < this->Mesh->Indices.Num() / 3; i ++)
	{
		auto &F = getFaceAt(i);
		FClothTriangleStaticProperties Tri(
			F.vertex_index[0], F.vertex_index[1], F.vertex_index[2],
			Mesh->TexCoords[F.vertex_index[0]],
			Mesh->TexCoords[F.vertex_index[1]],
			Mesh->TexCoords[F.vertex_index[2]]
		);
		Tri.ID = i * 3;
		ConditionBasis.Add(Tri);

		for (int j = 0; j < 3; j ++)
			SubForceRefsMap[F.vertex_index[j]].Add(Tri.ID + j);
	}

	// build up sub force look up table
	SubForceIndices.Reserve(Mesh->Indices.Num());
	PreSumOfRefIndices.Reserve(Mesh->Positions.Num() + 1);
	ConstraintMap.Reserve(Mesh->Indices.Num());
	ConstraintData.Reserve(Constraints.Num() * 3);
	int PreSum = 0;
	int ConstraintID = 1;
	ConstraintData.Add({1, 0, 0});
	ConstraintData.Add({0, 1, 0});
	ConstraintData.Add({0, 0, 1});
	PreSumOfRefIndices.Add(0);
	for (int i = 0; i < Mesh->Positions.Num(); i ++)
	{
		VertMasses.Add(Masses[i]);
		InvMasses.Add(1.0f / Masses[i]);
		Pre_Positions.Add(Mesh->Positions[i]);
		Positions.Add(Mesh->Positions[i]);
		PreSum += SubForceRefsMap[i].Num();
		PreSumOfRefIndices.Add(PreSum);
		for (auto id : SubForceRefsMap[i])
		{
			SubForceIndices.Add(id);
		}
		if (Constraints.Contains(i))
		{
			ConstraintMap.Add(3 * (ConstraintID ++));
			auto &C = Constraints[i];
			ConstraintData.Add({C[0][0], C[0][1], C[0][2]});
			ConstraintData.Add({C[1][0], C[1][1], C[1][2]});
			ConstraintData.Add({C[2][0], C[2][1], C[2][2]});
		} else
		{
			ConstraintMap.Add(0);
		}
	}

	// set up bend conditions, iterate within all triangle pairs:
	uint32 PairNum = 0;
	for (auto const E : other_half_of_edge)
	{
		PairNum += (E != UNKNOWN_HALF_EDGE);
	}
	uint32 const Num_BendConditions = PairNum / 2;
	Bend_Conditions.Reserve(Num_BendConditions * 4);
	SharedEdgeUVLengths.Reserve(Num_BendConditions);
	Bend_SubForceIndices.Reserve(Num_BendConditions * 4);
	Bend_PreSumOfRefIndices.Reserve(Masses.Num());
	Bend_LocalForces.SetNum(Num_BendConditions * 4);

	TSet<uint32> VisitedEdges;
	for (auto &Refs : SubForceRefsMap)
		Refs.Reset();
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
				int const Bend_Condition_ID = Bend_Conditions.Num();
				SubForceRefsMap[V0].Add(Bend_Condition_ID);
				SubForceRefsMap[V1].Add(Bend_Condition_ID + 1);
				SubForceRefsMap[V2].Add(Bend_Condition_ID + 2);
				SubForceRefsMap[V3].Add(Bend_Condition_ID + 3);
				Bend_Conditions.Add(V0);
				Bend_Conditions.Add(V1);
				Bend_Conditions.Add(V2);
				Bend_Conditions.Add(V3);
				VisitedEdges.Add(Edge);
				VisitedEdges.Add(OtherE);
				SharedEdgeUVLengths.Add((Mesh->TexCoords[V1] - Mesh->TexCoords[V2]).Size());
			}
		}
	}
	PreSum = 0;
	Bend_PreSumOfRefIndices.Add(0);
	for (int i = 0; i < Mesh->Positions.Num(); i ++)
	{
		PreSum += SubForceRefsMap[i].Num();
		Bend_PreSumOfRefIndices.Add(PreSum);
		for (auto id : SubForceRefsMap[i])
		{
			Bend_SubForceIndices.Add(id);
		}
	}
	
	ENQUEUE_RENDER_COMMAND(FRTClothSystemGPUBase_PrepareSimulation)(
	[this](FRHICommandList &CmdLists)
	{
		Indices.InitRHI();
		ConditionBasis.InitRHI();
		Velocities.InitRHI();
		LocalForces.InitRHI();
		SubForceIndices.InitRHI();
		PreSumOfRefIndices.InitRHI();
		InvMasses.InitRHI();
		VertMasses.InitRHI();
		ConstraintMap.InitRHI();
		ConstraintData.InitRHI();
		Pre_Positions.InitRHI();
		GlobalForces.InitRHI();
		Positions.InitRHI();
		Bend_Conditions.InitRHI();
		Bend_LocalForces.InitRHI();
		Bend_SubForceIndices.InitRHI();
		Bend_PreSumOfRefIndices.InitRHI();
		SharedEdgeUVLengths.InitRHI();
		InnerHitBVH.InitRHI();
	});
	FlushRenderingCommands();
}


void FRTClothSystemGPUBase::TickOnce(float Duration)
{
	FRTClothSystemBase::TickOnce(Duration);
	ENQUEUE_RENDER_COMMAND(FRTClothSystemGPUBase_TickOnce)(
	[this, Duration](FRHICommandList &RHICommands)
	{
		// set up uniform data
		SimParam.D_Bend = M_Material.D_Bend;
		SimParam.D_Shear = M_Material.D_Shear;
		SimParam.D_Stretch = M_Material.D_Stretch;
		SimParam.K_Bend = M_Material.K_Bend;
		SimParam.K_Shear = M_Material.K_Shear;
		SimParam.K_Stretch = M_Material.K_Stretch;
		SimParam.Rest_U = M_Material.Rest_U;
		SimParam.Rest_V = M_Material.Rest_V;
		SimParam.DT = Duration;
		SimParam.InitTheta = M_Material.InitTheta;
		SimParam.K_Collision = M_Material.K_Collision;
		SimParam.D_Collision = M_Material.D_Collision;
		auto const SimParamUBO = FRTClothSimulationParameters::CreateUniformBuffer(SimParam, UniformBuffer_SingleFrame);
		SetupExternalForces_RenderThread();
		if (M_Material.EnableInnerCollision)
		{
			UpdateTriangleProperties(Mesh->Positions, Velocities.GetBufferData());
			TArray<FInnerCollisionBVHNode> BVHTree;
			BVHTree.Reserve(TriangleBoundingSpheres.Num() * 2 - 1);
			FInnerCollisionBVHNode::BuildFrom(TriangleBoundingSpheres, BVHTree, 0, TriangleBoundingSpheres.Num() - 1);
			InnerHitBVH.UploadData(BVHTree);
			InnerCollisionForces_RenderThread(SimParamUBO, RHICommands);
		}
		UpdateStretchAndShearForce_RenderThread(SimParamUBO, RHICommands);
		UpdateBendForce_RenderThread(SimParamUBO, RHICommands);
		AssembleForce(RHICommands);
		VerletIntegration_RenderThread(SimParamUBO, RHICommands);
		auto const* data = RHILockStructuredBuffer(Positions.RHI, 0,  Positions.GetDataSize(), RLM_ReadOnly);
		FMemory::Memcpy(Mesh->Positions.GetData(), data, Positions.GetDataSize());
		RHIUnlockStructuredBuffer(Positions.RHI);
	});
}

void FRTClothSystemGPUBase::InnerCollisionForces_RenderThread(FUniformBufferRHIRef const&UBO, FRHICommandList &RHICommands) const
{
	TShaderMapRef<FInnerCollisionForcesCS> const InnerCollisionCSRef(GetGlobalShaderMap(ERHIFeatureLevel::SM5));
	FRHIComputeShader* CS = InnerCollisionCSRef.GetComputeShader();
	RHICommands.SetShaderUniformBuffer(CS, InnerCollisionCSRef->SimParams.GetBaseIndex(), UBO);
	RHICommands.SetShaderResourceViewParameter(CS, InnerCollisionCSRef->InnerHitBVH.GetBaseIndex(), InnerHitBVH.SRV);
	RHICommands.SetShaderResourceViewParameter(CS, InnerCollisionCSRef->Masses.GetBaseIndex(), VertMasses.SRV);
	RHICommands.SetShaderResourceViewParameter(CS, InnerCollisionCSRef->Indices.GetBaseIndex(), Indices.SRV);
	RHICommands.SetShaderResourceViewParameter(CS, InnerCollisionCSRef->Velocities.GetBaseIndex(), Velocities.SRV);
	RHICommands.SetShaderResourceViewParameter(CS, InnerCollisionCSRef->Positions.GetBaseIndex(), Positions.SRV);
	RHICommands.SetUAVParameter(CS, InnerCollisionCSRef->Forces.GetBaseIndex(), GlobalForces.UAV);
	
	RHICommands.SetComputeShader(CS);
	DispatchComputeShader(RHICommands, InnerCollisionCSRef, Masses.Num() / 16 + 1, 1, 1);
}

void FRTClothSystemGPUBase::UpdateStretchAndShearForce_RenderThread(FUniformBufferRHIRef const&UBO, FRHICommandList &RHICommands) const
{
	TShaderMapRef<FStretchShearConditionCS> const SSForceCSRef(GetGlobalShaderMap(ERHIFeatureLevel::SM5));
	FRHIComputeShader* CS = SSForceCSRef.GetComputeShader();
	RHICommands.SetShaderUniformBuffer(CS, SSForceCSRef->SimParams.GetBaseIndex(), UBO);
	RHICommands.SetShaderResourceViewParameter(CS, SSForceCSRef->ConditionBasis.GetBaseIndex(), ConditionBasis.SRV);
	RHICommands.SetShaderResourceViewParameter(CS, SSForceCSRef->Velocities.GetBaseIndex(), Velocities.SRV);
	RHICommands.SetShaderResourceViewParameter(CS, SSForceCSRef->Positions.GetBaseIndex(), Positions.SRV);
	RHICommands.SetUAVParameter(CS, SSForceCSRef->LocalForces.GetBaseIndex(), LocalForces.UAV);
	RHICommands.SetComputeShader(CS);
	DispatchComputeShader(RHICommands, SSForceCSRef, ConditionBasis.Num() / 16 + 1, 1, 1);
}

void FRTClothSystemGPUBase::UpdateBendForce_RenderThread(FUniformBufferRHIRef const&UBO, FRHICommandList &RHICommands) const
{
	// second pass, bend forces
	TShaderMapRef<FRTBendForceCS> const BendForcesCSRef(GetGlobalShaderMap(ERHIFeatureLevel::SM5));
	FRHIComputeShader* CS = BendForcesCSRef.GetComputeShader();
			
	RHICommands.SetShaderUniformBuffer(CS, BendForcesCSRef->SimParams.GetBaseIndex(), UBO);
	RHICommands.SetShaderResourceViewParameter(CS, BendForcesCSRef->BendConditions.GetBaseIndex(), Bend_Conditions.SRV);
	RHICommands.SetShaderResourceViewParameter(CS, BendForcesCSRef->SharedEdgeLengths.GetBaseIndex(), SharedEdgeUVLengths.SRV);
	RHICommands.SetShaderResourceViewParameter(CS, BendForcesCSRef->Velocities.GetBaseIndex(), Velocities.SRV);
	RHICommands.SetShaderResourceViewParameter(CS, BendForcesCSRef->Positions.GetBaseIndex(), Positions.SRV);
	RHICommands.SetUAVParameter(CS, BendForcesCSRef->LocalForces.GetBaseIndex(), Bend_LocalForces.UAV);
	RHICommands.SetComputeShader(CS);
			
	DispatchComputeShader(RHICommands, BendForcesCSRef, SharedEdgeUVLengths.Num() / 16 + 1, 1, 1);
}

void FRTClothSystemGPUBase::AssembleForce(FRHICommandList &RHICommands) const
{
	TShaderMapRef<FLocalForceAssemplerCS> const ForceAssembleCSRef(GetGlobalShaderMap(ERHIFeatureLevel::SM5));
	FRHIComputeShader* CS = ForceAssembleCSRef.GetComputeShader();
	// Stretch and shear
	RHICommands.SetUAVParameter(CS, ForceAssembleCSRef->LocalForces.GetBaseIndex(), LocalForces.UAV);
	RHICommands.SetUAVParameter(CS, ForceAssembleCSRef->Forces.GetBaseIndex(), GlobalForces.UAV);
	RHICommands.SetShaderResourceViewParameter(CS, ForceAssembleCSRef->PreSumOfRefIndices.GetBaseIndex(), PreSumOfRefIndices.SRV);
	RHICommands.SetShaderResourceViewParameter(CS, ForceAssembleCSRef->SubForceIndices.GetBaseIndex(), SubForceIndices.SRV);
			
	RHICommands.SetComputeShader(CS);
	DispatchComputeShader(RHICommands, ForceAssembleCSRef, Masses.Num() / 16 + 1, 1, 1);
	// Bend
	RHICommands.SetUAVParameter(CS, ForceAssembleCSRef->LocalForces.GetBaseIndex(), Bend_LocalForces.UAV);
	RHICommands.SetUAVParameter(CS, ForceAssembleCSRef->Forces.GetBaseIndex(), GlobalForces.UAV);
	RHICommands.SetShaderResourceViewParameter(CS, ForceAssembleCSRef->PreSumOfRefIndices.GetBaseIndex(), Bend_PreSumOfRefIndices.SRV);
	RHICommands.SetShaderResourceViewParameter(CS, ForceAssembleCSRef->SubForceIndices.GetBaseIndex(), Bend_SubForceIndices.SRV);
			
	RHICommands.SetComputeShader(CS);
	DispatchComputeShader(RHICommands, ForceAssembleCSRef, Masses.Num() / 16 + 1, 1, 1);
}

void FRTClothSystemGPUBase::VerletIntegration_RenderThread(FUniformBufferRHIRef const&UBO, FRHICommandList &RHICommands) const
{
	// final Pass, update Positions
	TShaderMapRef<FVerletIntegratorCS> const VerletCSRef(GetGlobalShaderMap(ERHIFeatureLevel::SM5));
	FRHIComputeShader* CS = VerletCSRef.GetComputeShader();

	RHICommands.SetShaderUniformBuffer(CS, VerletCSRef->SimParams.GetBaseIndex(), UBO);
	RHICommands.SetUAVParameter(CS, VerletCSRef->Velocities.GetBaseIndex(), Velocities.UAV);
	RHICommands.SetUAVParameter(CS, VerletCSRef->Pre_Positions.GetBaseIndex(), Pre_Positions.UAV);
	RHICommands.SetUAVParameter(CS, VerletCSRef->Positions.GetBaseIndex(), Positions.UAV);
	RHICommands.SetShaderResourceViewParameter(CS, VerletCSRef->Forces.GetBaseIndex(), GlobalForces.SRV);
	RHICommands.SetShaderResourceViewParameter(CS, VerletCSRef->InvMasses.GetBaseIndex(), InvMasses.SRV);
	RHICommands.SetShaderResourceViewParameter(CS, VerletCSRef->ConstraintMap.GetBaseIndex(), ConstraintMap.SRV);
	RHICommands.SetShaderResourceViewParameter(CS, VerletCSRef->ConstraintData.GetBaseIndex(), ConstraintData.SRV);
	RHICommands.SetComputeShader(CS);
			
	DispatchComputeShader(RHICommands, VerletCSRef, Masses.Num() / 16 + 1, 1, 1);
}

void FRTClothSystemGPUBase::SetupExternalForces_RenderThread()
{
	for (int i = 0; i < Masses.Num(); i ++)
		ExternalForces[i] = Gravity * Masses[i];
	void *Data = RHILockStructuredBuffer(GlobalForces.RHI, 0, GlobalForces.GetDataSize(), RLM_WriteOnly);
	memcpy(Data, ExternalForces.GetData(), GlobalForces.GetDataSize());
	RHIUnlockStructuredBuffer(GlobalForces.RHI);
}

FRTClothSystemGPUBase::~FRTClothSystemGPUBase()
{
	
}

