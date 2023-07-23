#include "FRTClothSystemGPUBase.h"

#include "FStretchShearConditionCS.h"
#include "FLocalForceAssemplerCS.h"
#include "FVerletIntegratorCS.h"
#include "FVecCopyCS.h"

IMPLEMENT_GLOBAL_SHADER_PARAMETER_STRUCT(FRTClothSimulationParameters, "RTClothMaterialUB");

void FRTClothSystemGPUBase::PrepareSimulation()
{
	// setup condition basis
	ConditionBasis.Reserve(this->Mesh->Indices.Num() / 3);
	Velocities.SetNum(this->Mesh->Positions.Num());
	LocalForces.SetNum(this->Mesh->Indices.Num());
	GlobalForces.SetNum(Mesh->Positions.Num());
	Pre_Positions.Reserve(this->Mesh->Positions.Num());
	Positions.Reserve(this->Mesh->Positions.Num());
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
		Pre_Positions.Add(Mesh->Positions[i]);
		Positions.Add(Mesh->Positions[i]);
		PreSum += SubForceRefsMap[i].Num();
		PreSumOfRefIndices.Add(PreSum);
		for (auto id : SubForceRefsMap[i])
		{
			SubForceIndices.Add(id);
		}
		VertMasses.Add(Masses[i]);
		if (Constraints.Contains(i))
		{
			ConstraintMap.Add(ConstraintID ++);
			auto &C = Constraints[i];
			ConstraintData.Add({C[0][0], C[0][1], C[0][2]});
			ConstraintData.Add({C[1][0], C[1][1], C[1][2]});
			ConstraintData.Add({C[2][0], C[2][1], C[2][2]});
		} else
		{
			ConstraintMap.Add(0);
		}
	}
	ENQUEUE_RENDER_COMMAND(FRTClothSystemGPUBase_PrepareSimulation)(
	[this](FRHICommandList &CmdLists)
	{
		ConditionBasis.InitRHI();
		Velocities.InitRHI();
		LocalForces.InitRHI();
		SubForceIndices.InitRHI();
		PreSumOfRefIndices.InitRHI();
		VertMasses.InitRHI();
		ConstraintMap.InitRHI();
		ConstraintData.InitRHI();
		Pre_Positions.InitRHI();
		GlobalForces.InitRHI();
		Positions.InitRHI();
		// create uniform buffer
		SimParam.D_Bend = M_Material.D_Bend;
		SimParam.D_Shear = M_Material.D_Shear;
		SimParam.D_Stretch = M_Material.D_Stretch;
		SimParam.K_Bend = M_Material.K_Bend;
		SimParam.K_Shear = M_Material.K_Shear;
		SimParam.K_Stretch = M_Material.K_Stretch;
		SimParam.Rest_U = M_Material.Rest_U;
		SimParam.Rest_V = M_Material.Rest_V;
	});
}


void FRTClothSystemGPUBase::TickOnce(float Duration)
{
	ENQUEUE_RENDER_COMMAND(FRTClothSystemGPUBase_TickOnce)(
	[this, Duration](FRHICommandList &RHICommands)
	{
		SimParam.DT = Duration;
		SimParam.ExternalForce = Gravity;
		auto const SimParamUBO = FRTClothSimulationParameters::CreateUniformBuffer(SimParam, UniformBuffer_SingleFrame);
		TArray<FVector> AssembledForces;
		AssembledForces.SetNumZeroed(Mesh->Positions.Num());
		{	// First Pass, update force
			TShaderMapRef<FStretchShearConditionCS> const SSForceCSRef(GetGlobalShaderMap(ERHIFeatureLevel::SM5));
			FRHIComputeShader* CS = SSForceCSRef.GetComputeShader();
			
			RHICommands.SetShaderUniformBuffer(CS, SSForceCSRef->SimParams.GetBaseIndex(), SimParamUBO);
			RHICommands.SetShaderResourceViewParameter(CS, SSForceCSRef->ConditionBasis.GetBaseIndex(), ConditionBasis.SRV);
			RHICommands.SetShaderResourceViewParameter(CS, SSForceCSRef->Velocities.GetBaseIndex(), Velocities.SRV);
			RHICommands.SetShaderResourceViewParameter(CS, SSForceCSRef->Positions.GetBaseIndex(), Positions.SRV);
			RHICommands.SetUAVParameter(CS, SSForceCSRef->LocalForces.GetBaseIndex(), LocalForces.UAV);
			RHICommands.SetComputeShader(CS);
			DispatchComputeShader(RHICommands, SSForceCSRef, ConditionBasis.Num() / 16 + 1, 1, 1);

			// //
			// TArray<FVector> result;
			// result.SetNumUninitialized(LocalForces.Num());
			// auto const* data = RHILockStructuredBuffer(LocalForces.RHI, 0,  LocalForces.GetDataSize(), RLM_ReadOnly);
			// FMemory::Memcpy(result.GetData(), data, LocalForces.GetDataSize());		
			// RHIUnlockStructuredBuffer(LocalForces.RHI);
			//
			// for (int i = 0; i < Mesh->Positions.Num(); i ++)
			// {
			// 	int StartIndex = PreSumOfRefIndices[i];
			// 	int EndIndex = PreSumOfRefIndices[i + 1];
			//
			// 	FVector Force = FVector(0, 0, 0);
			// 	for (int Index = StartIndex; Index < EndIndex; Index ++)
			// 	{
			// 		int LocalID = SubForceIndices[Index];
			// 		Force += result[LocalID];
			// 	}
			// 	AssembledForces[i] = Force;
			// }
		}
		{
			// Second Pass, Assemble Forces
			TShaderMapRef<FLocalForceAssemplerCS> const ForceAssembleCSRef(GetGlobalShaderMap(ERHIFeatureLevel::SM5));
			FRHIComputeShader* CS = ForceAssembleCSRef.GetComputeShader();
			
			RHICommands.SetUAVParameter(CS, ForceAssembleCSRef->LocalForces.GetBaseIndex(), LocalForces.UAV);
			RHICommands.SetUAVParameter(CS, ForceAssembleCSRef->Forces.GetBaseIndex(), GlobalForces.UAV);
			RHICommands.SetShaderResourceViewParameter(CS, ForceAssembleCSRef->PreSumOfRefIndices.GetBaseIndex(), PreSumOfRefIndices.SRV);
			RHICommands.SetShaderResourceViewParameter(CS, ForceAssembleCSRef->SubForceIndices.GetBaseIndex(), SubForceIndices.SRV);
			
			RHICommands.SetComputeShader(CS);
			DispatchComputeShader(RHICommands, ForceAssembleCSRef, Masses.Num() / 16 + 1, 1, 1);

			// TArray<FVector> result;
			// result.SetNumUninitialized(Masses.Num());
			// auto const* data = RHILockStructuredBuffer(GlobalForces.RHI, 0,  GlobalForces.GetDataSize(), RLM_ReadOnly);
			// FMemory::Memcpy(result.GetData(), data, GlobalForces.GetDataSize());		
			// RHIUnlockStructuredBuffer(GlobalForces.RHI);
		}
		{
			// Third Pass, update Positions
			TShaderMapRef<FVerletIntegratorCS> const VerletCSRef(GetGlobalShaderMap(ERHIFeatureLevel::SM5));
			FRHIComputeShader* CS = VerletCSRef.GetComputeShader();

			RHICommands.SetShaderUniformBuffer(CS, VerletCSRef->SimParams.GetBaseIndex(), SimParamUBO);
			RHICommands.SetUAVParameter(CS, VerletCSRef->Velocities.GetBaseIndex(), Velocities.UAV);
			RHICommands.SetUAVParameter(CS, VerletCSRef->Pre_Positions.GetBaseIndex(), Pre_Positions.UAV);
			RHICommands.SetUAVParameter(CS, VerletCSRef->Positions.GetBaseIndex(), Positions.UAV);
			RHICommands.SetShaderResourceViewParameter(CS, VerletCSRef->Forces.GetBaseIndex(), GlobalForces.SRV);
			RHICommands.SetShaderResourceViewParameter(CS, VerletCSRef->Masses.GetBaseIndex(), VertMasses.SRV);
			RHICommands.SetComputeShader(CS);
			
			DispatchComputeShader(RHICommands, VerletCSRef, Masses.Num() / 16 + 1, 1, 1);
		}
	});
}

void FRTClothSystemGPUBase::UpdatePositionDataTo(FRTDynamicVertexBuffer& DstBuffer)
{
	check(IsInRenderingThread());
	
	FAutoTimer Ti("Copy");

	// auto &RHICommands = GetImmediateCommandList_ForRenderCommand();
	// TShaderMapRef<FVecCopyCS> const CopyCSRef(GetGlobalShaderMap(ERHIFeatureLevel::SM5));
	// FRHIComputeShader* CS = CopyCSRef.GetComputeShader();
	//
	// RHICommands.SetShaderResourceViewParameter(CS, CopyCSRef->InVec.GetBaseIndex(), Positions.SRV);
	// RHICommands.SetUAVParameter(CS, CopyCSRef->OutVec.GetBaseIndex(), DstBuffer.GetPositionUAV());
	//
	// RHICommands.SetComputeShader(CS);
	// DispatchComputeShader(RHICommands, CopyCSRef, Masses.Num() / 16 + 1, 1, 1);

	auto const* data = RHILockStructuredBuffer(Positions.RHI, 0,  Positions.GetDataSize(), RLM_ReadOnly);
	auto * data2 = RHILockVertexBuffer(DstBuffer.VertexBufferRHI, 0,  DstBuffer.GetDataSize(), RLM_WriteOnly);
	FMemory::Memcpy(data2, data, Positions.GetDataSize());	
	RHIUnlockVertexBuffer(DstBuffer.VertexBufferRHI);
	RHIUnlockStructuredBuffer(Positions.RHI);	
}

FRTClothSystemGPUBase::~FRTClothSystemGPUBase()
{
	
}


