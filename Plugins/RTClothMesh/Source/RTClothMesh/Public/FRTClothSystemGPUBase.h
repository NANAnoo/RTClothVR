#pragma once

#include "FRTClothSystemBase.h"

#include "GlobalShader.h"
#include "UniformBuffer.h"
#include "RHICommandList.h"

BEGIN_GLOBAL_SHADER_PARAMETER_STRUCT(FRTClothSimulationParameters, )
// for bend condition
SHADER_PARAMETER(float, K_Bend)
SHADER_PARAMETER(float, D_Bend)

// for stretch condition
SHADER_PARAMETER(float, K_Stretch)
SHADER_PARAMETER(float, D_Stretch)

// for shear condition
SHADER_PARAMETER(float, K_Shear)
SHADER_PARAMETER(float, D_Shear)

// rest U V
SHADER_PARAMETER(float, Rest_U)
SHADER_PARAMETER(float, Rest_V)

// duration
SHADER_PARAMETER(float, DT)

// bend initial angle
SHADER_PARAMETER(float, InitTheta)

// collision spring K
SHADER_PARAMETER(float, K_Collision)

// collision spring D
SHADER_PARAMETER(float, D_Collision)

END_GLOBAL_SHADER_PARAMETER_STRUCT()

template <typename T>
class FRTComputeBuffer
{
public:
	using ElementType = T;
	int Num() const {return (Data.Num() == 0 ? NumOfResource: Data.Num());}
	void Reserve(int Number)
	{
		Data.Reserve(Number);
	}
	void SetNum(int Number)
	{
		Data.SetNumZeroed(Number);
	}
	void Add(T const& Element)
	{
		Data.Add(Element);
	}
	T& operator[] (int Index)
	{
		return Data[Index];
	}
	FUnorderedAccessViewRHIRef UAV;
	FShaderResourceViewRHIRef SRV;
	FStructuredBufferRHIRef RHI;

	TArray<T> GetBufferData()
	{
		TArray<T> result;
		result.SetNumZeroed(Num());
		const void *data = RHILockStructuredBuffer(RHI, 0, GetDataSize(), RLM_ReadOnly);
		FMemory::Memcpy(result.GetData(), data, GetDataSize());
		RHIUnlockStructuredBuffer(RHI);
		return result;
	}

	void UploadData(TArray<T> const&NewData)
	{
		void *data = RHILockStructuredBuffer(RHI, 0, GetDataSize(), RLM_WriteOnly);
		FMemory::Memcpy(data, NewData.GetData(), GetDataSize());
		RHIUnlockStructuredBuffer(RHI);
	}

	void InitRHI()
	{
		NumOfResource = Data.Num();
		FRHIResourceCreateInfo createInfo;
		createInfo.ResourceArray = &Data;
		RHI = RHICreateStructuredBuffer(sizeof(T), GetDataSize(), BUF_UnorderedAccess | BUF_ShaderResource, createInfo);
		UAV = RHICreateUnorderedAccessView(RHI, false, false);
		SRV = RHICreateShaderResourceView(RHI);
	}

	uint32 GetDataSize() const
	{
		return sizeof(T) * Num();
	}
	virtual ~FRTComputeBuffer()
	{
		UAV.SafeRelease();
		SRV.SafeRelease();
		RHI.SafeRelease();
	}
private:
	TResourceArray<T> Data;
	uint32 NumOfResource = 0;
};

class FRTClothSystemGPUBase : public  FRTClothSystemBase
{
public:
	FRTClothSystemGPUBase() {}
	virtual ~FRTClothSystemGPUBase() override;
	
	// simulate for one tick
	virtual void TickOnce(float Duration) override;

	// update data into DstBuffer
	// virtual void UpdatePositionDataTo(FRHICommandList &CmdList, FRTDynamicVertexBuffer &DstBuffer) override;
	
protected:
	void SetupExternalForces_RenderThread();

	void InnerCollisionForces_RenderThread(FUniformBufferRHIRef const&UBO, FRHICommandList &RHICommands) const;
	void UpdateStretchAndShearForce_RenderThread(FUniformBufferRHIRef const&UBO, FRHICommandList &RHICommands) const;
	void UpdateBendForce_RenderThread(FUniformBufferRHIRef const&UBO, FRHICommandList &RHICommands) const;
	void AssembleForce(FRHICommandList &RHICommands) const;
	void VerletIntegration_RenderThread(FUniformBufferRHIRef const&UBO, FRHICommandList &RHICommands) const;
	
	// setup runtime variables, before tick
	virtual void PrepareSimulation() override;
	
	FRTComputeBuffer<FClothTriangleStaticProperties> ConditionBasis;
	FRTComputeBuffer<FVector> Velocities;
	FRTComputeBuffer<FVector> LocalForces;
	FRTComputeBuffer<FVector> GlobalForces;
	FRTComputeBuffer<FVector> Pre_Positions;
	FRTComputeBuffer<FVector> Positions;
	FRTComputeBuffer<float> InvMasses;
	FRTComputeBuffer<float> VertMasses;
	FRTComputeBuffer<int> Indices;
	FRTComputeBuffer<int> PreSumOfRefIndices;
	FRTComputeBuffer<int> SubForceIndices;
	FRTComputeBuffer<int> ConstraintMap;
	FRTComputeBuffer<FVector> ConstraintData;

	// buffers for bend conditions
	FRTComputeBuffer<uint32> Bend_Conditions;
	FRTComputeBuffer<int> Bend_PreSumOfRefIndices;
	FRTComputeBuffer<int> Bend_SubForceIndices;
	FRTComputeBuffer<FVector> Bend_LocalForces;
	FRTComputeBuffer<float> SharedEdgeUVLengths;

	// Collision Structure
	FRTComputeBuffer<FInnerCollisionBVHNode> InnerHitBVH;
	
	FRTClothSimulationParameters SimParam;

	FRTDynamicVertexBuffer *Updating = nullptr;
};