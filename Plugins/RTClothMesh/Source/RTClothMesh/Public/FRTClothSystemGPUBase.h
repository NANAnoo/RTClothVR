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
// external force
SHADER_PARAMETER(FVector, ExternalForce)

END_GLOBAL_SHADER_PARAMETER_STRUCT()

template <typename T>
class FRTComputeBuffer
{
public:
	int Num() const {return Data.Num();}
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

	void InitRHI()
	{
		FRHIResourceCreateInfo createInfo;
		createInfo.ResourceArray = &Data;

		RHI = RHICreateStructuredBuffer(sizeof(T), GetDataSize(), BUF_UnorderedAccess | BUF_ShaderResource, createInfo);
		UAV = RHICreateUnorderedAccessView(RHI, false, false);
		SRV = RHICreateShaderResourceView(RHI);
	}

	uint32 GetDataSize()
	{
		return sizeof(T) * Data.Num();
	}
	virtual ~FRTComputeBuffer()
	{
		UAV.SafeRelease();
		SRV.SafeRelease();
		RHI.SafeRelease();
	}
private:
	TResourceArray<T> Data;
};

class FRTClothSystemGPUBase : public  FRTClothSystemBase
{
public:
	FRTClothSystemGPUBase() {}
	virtual ~FRTClothSystemGPUBase() override;
	
	// simulate for one tick
	virtual void TickOnce(float Duration) override;

	// update data into DstBuffer
	virtual void UpdatePositionDataTo(FRTDynamicVertexBuffer &DstBuffer) override;
	
protected:
	// setup runtime variables, before tick
	virtual void PrepareSimulation() override;
	
	FRTComputeBuffer<FClothTriangleStaticProperties> ConditionBasis;
	FRTComputeBuffer<FVector> Velocities;
	FRTComputeBuffer<FVector> LocalForces;
	FRTComputeBuffer<FVector> GlobalForces;
	FRTComputeBuffer<FVector> Pre_Positions;
	FRTComputeBuffer<FVector> Positions;
	FRTComputeBuffer<float> VertMasses;
	FRTComputeBuffer<int> PreSumOfRefIndices;
	FRTComputeBuffer<int> SubForceIndices;
	FRTComputeBuffer<int> ConstraintMap;
	FRTComputeBuffer<FVector> ConstraintData;
	
	FRTClothSimulationParameters SimParam;
};