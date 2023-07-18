﻿#include "URTClothMeshComponent.h"

#include "ModifiedCGSolver.h"

#include <RenderingThread.h>
#include <RenderResource.h>

#include <PrimitiveViewRelevance.h>
#include <PrimitiveSceneProxy.h>
#include <SceneManagement.h>

#include <VertexFactory.h>
#include <LocalVertexFactory.h>

#include <MaterialShared.h>
#include <Materials/Material.h>

#include <DynamicMeshBuilder.h>

#include <Engine/Engine.h>

#include "FRTClothSystem_ImplicitIntegration_CPU.h"
#include "FRTClothSystem_Leapfrog_CPU.h"
#include "FRTClothSystem_Verlet_CPU.h"

#include "FTestCS.h"

class FRTDynamicVertexBuffer : public FVertexBuffer
{
private:
	struct FRTVertex
	{
		FRTVertex() {}
		FRTVertex(float x, float y, float z) : X(x), Y(y), Z(z) {}
		FRTVertex(FRTVertex const& Vec) : X(Vec.X), Y(Vec.Y), Z(Vec.Z) 
		{
		}
		FRTVertex& operator=(FVector const& Vec)
		{
			X = Vec.X;
			Y = Vec.Y;
			Z = Vec.Z;
			return *this;
		}
		float X = 0;
		float Y = 0;
		float Z = 0;
	};
	int32 NumVertices = 0;
	TResourceArray<FRTVertex> VertexData;
	FUnorderedAccessViewRHIRef Position_UAV;
	FShaderResourceViewRHIRef PositionComponentSRV;
	
	FVertexBufferRHIRef CreateRHIBuffer_RenderThread()
	{
		if (NumVertices)
		{
			FRHIResourceCreateInfo CreateInfo;
			CreateInfo.ResourceArray = &VertexData;
			return RHICreateVertexBuffer(sizeof(FRTVertex) * NumVertices, BUF_UnorderedAccess | BUF_ShaderResource, CreateInfo);
		}
		return nullptr;
	}
public:
	void Init(uint32 Num)
	{
		NumVertices = Num;
		VertexData.Init({0, 0, 0}, Num);
	}

	uint32 GetNumVertices() const {return NumVertices;}

	uint32 GetDataSize() const {return NumVertices * sizeof(FRTVertex);}

	FRTVertex &VertexPosition(uint32 i)
	{
		return VertexData[i];
	}

	void BindPositionVertexBuffer(const FVertexFactory* VertexFactory, FStaticMeshDataType& StaticMeshData) const
	{
		StaticMeshData.PositionComponent = FVertexStreamComponent(
			this,
			0,
			sizeof(FRTVertex),
			VET_Float3
		);
		StaticMeshData.PositionComponentSRV = PositionComponentSRV;
	}
	
	// FRenderResource interface.
	virtual void InitRHI() override
	{
		VertexBufferRHI = CreateRHIBuffer_RenderThread();
		if (VertexBufferRHI)
		{
			Position_UAV = RHICreateUnorderedAccessView(VertexBufferRHI, PF_FloatRGB);
			PositionComponentSRV = RHICreateShaderResourceView(FShaderResourceViewInitializer(VertexBufferRHI, PF_R32_FLOAT));
		}
	}
	FUnorderedAccessViewRHIRef const& GetPositionUAV() {return this->Position_UAV;}
	virtual void ReleaseRHI() override
	{
		Position_UAV.SafeRelease();
		PositionComponentSRV.SafeRelease();
		FVertexBuffer::ReleaseRHI();
	}
	virtual FString GetFriendlyName() const override { return TEXT("PositionOnly Static-mesh vertices"); }
};

// data pack
struct FClothMeshPackedData
{
	TArray<FVector> *Positions;
};

// a custom scene proxy
class FClothMeshSceneProxy : public FPrimitiveSceneProxy
{
public:
	FClothMeshSceneProxy(URTClothMeshComponent const* Component, std::shared_ptr<FClothRawMesh> const& ClothMesh) :
	FPrimitiveSceneProxy(Component),
	Material(Component->GetMaterial(0)),
	VertexFactory(GetScene().GetFeatureLevel(), "FClothMeshVertexFactory"),
	MaterialRelevance(Component->GetMaterialRelevance(GetScene().GetFeatureLevel()))
	{
		// Init Vertex Factory
		// Copy data from vertex buffer
		const int32 NumVerts = ClothMesh->Positions.Num();

		PositionVertexBuffer.Init(NumVerts);
		StaticMeshVertexBuffer.Init(NumVerts, 1);
		ColorVertexBuffer.Init(NumVerts);

		// Allocate verts
		for (int VertIdx = 0; VertIdx < NumVerts; VertIdx++)
		{
			PositionVertexBuffer.VertexPosition(VertIdx) = ClothMesh->Positions[VertIdx];
			StaticMeshVertexBuffer.SetVertexTangents(
				VertIdx,
				ClothMesh->TangentXArray[VertIdx],
				ClothMesh->TangentYArray[VertIdx],
				ClothMesh->TangentZArray[VertIdx]
			);
			StaticMeshVertexBuffer.SetVertexUV(VertIdx, 0, ClothMesh->TexCoords[VertIdx]);
			ColorVertexBuffer.VertexColor(VertIdx) = ClothMesh->Colors[VertIdx];
		}
		
		// set up resources
		IndexBuffer.Indices = ClothMesh->Indices;
		auto Self = this;
		ENQUEUE_RENDER_COMMAND(StaticMeshVertexBuffersLegacyInit)(
			[Self](FRHICommandListImmediate& RHICmdList)
			{
				auto const InitOrUpdateResource = [](FRenderResource* Resource)
				{
					if (!Resource->IsInitialized())
					{
						Resource->InitResource();
					}
					else
					{
						Resource->UpdateRHI();
					}
				};
				InitOrUpdateResource(&Self->PositionVertexBuffer);
				InitOrUpdateResource(&Self->StaticMeshVertexBuffer);
				InitOrUpdateResource(&Self->ColorVertexBuffer);

				FLocalVertexFactory::FDataType Data;
				Self->PositionVertexBuffer.BindPositionVertexBuffer(&Self->VertexFactory, Data);
				Self->StaticMeshVertexBuffer.BindTangentVertexBuffer(&Self->VertexFactory, Data);
				Self->StaticMeshVertexBuffer.BindPackedTexCoordVertexBuffer(&Self->VertexFactory, Data);
				Self->StaticMeshVertexBuffer.BindLightMapVertexBuffer(&Self->VertexFactory, Data, 0);
				Self->ColorVertexBuffer.BindColorVertexBuffer(&Self->VertexFactory, Data);
				Self->VertexFactory.SetData(Data);
				InitOrUpdateResource(&Self->VertexFactory);
			});

		// Enqueue initialization of render resource
		BeginInitResource(&PositionVertexBuffer);
		BeginInitResource(&StaticMeshVertexBuffer);
		BeginInitResource(&ColorVertexBuffer);
		BeginInitResource(&IndexBuffer);
		BeginInitResource(&VertexFactory);
		
		// set up material
		if (Material == nullptr)
		{
			Material = UMaterial::GetDefaultMaterial(MD_Surface);
		}
	}

	virtual ~FClothMeshSceneProxy() override
	{
		PositionVertexBuffer.ReleaseResource();
		ColorVertexBuffer.ReleaseResource();
		StaticMeshVertexBuffer.ReleaseResource();
		IndexBuffer.ReleaseResource();
		VertexFactory.ReleaseResource();
	}

	virtual void GetDynamicMeshElements(const TArray<const FSceneView*>& Views, const FSceneViewFamily& ViewFamily, uint32 VisibilityMap, FMeshElementCollector& Collector) const override
	{
		// statistic
		QUICK_SCOPE_CYCLE_COUNTER(STAT_FClothMeshSceneProxy_GetDynamicMeshElements)

		const bool bWireFrame = AllowDebugViewmodes() && ViewFamily.EngineShowFlags.Wireframe;
		auto const WireframeMaterialInstance = new FColoredMaterialRenderProxy(
			GEngine->WireframeMaterial ? GEngine->WireframeMaterial->GetRenderProxy() : nullptr,
			FLinearColor(0, 0.5f, 1.0f)
		);

		Collector.RegisterOneFrameMaterialProxy(WireframeMaterialInstance);

		FMaterialRenderProxy const* MaterialProxy = bWireFrame ? WireframeMaterialInstance : Material->GetRenderProxy();

		for (int32 ViewIndex = 0; ViewIndex < Views.Num(); ViewIndex ++)
		{
			if (VisibilityMap & (1 << ViewIndex))
			{
				// Draw the mesh.
				FMeshBatch& Mesh = Collector.AllocateMesh();
				FMeshBatchElement& BatchElement = Mesh.Elements[0];
				BatchElement.IndexBuffer = &IndexBuffer;
				Mesh.bWireframe = bWireFrame;
				Mesh.VertexFactory = &VertexFactory;
				Mesh.MaterialRenderProxy = MaterialProxy;

				bool bHasPrecomputedVolumetricLightmap;
				FMatrix PreviousLocalToWorld;
				int32 SingleCaptureIndex;
				bool bOutputVelocity;
				GetScene().GetPrimitiveUniformShaderParameters_RenderThread(GetPrimitiveSceneInfo(), bHasPrecomputedVolumetricLightmap, PreviousLocalToWorld, SingleCaptureIndex, bOutputVelocity);

				FDynamicPrimitiveUniformBuffer& DynamicPrimitiveUniformBuffer = Collector.AllocateOneFrameResource<FDynamicPrimitiveUniformBuffer>();
				DynamicPrimitiveUniformBuffer.Set(GetLocalToWorld(), PreviousLocalToWorld, GetBounds(), GetLocalBounds(), true, bHasPrecomputedVolumetricLightmap, DrawsVelocity(), bOutputVelocity);
				BatchElement.PrimitiveUniformBufferResource = &DynamicPrimitiveUniformBuffer.UniformBuffer;

				BatchElement.FirstIndex = 0;
				BatchElement.NumPrimitives =IndexBuffer.Indices.Num() / 3;
				BatchElement.MinVertexIndex = 0;
				BatchElement.MaxVertexIndex = PositionVertexBuffer.GetNumVertices() - 1;
				Mesh.ReverseCulling = IsLocalToWorldDeterminantNegative();
				Mesh.Type = PT_TriangleList;
				Mesh.DepthPriorityGroup = SDPG_World;
				Mesh.bCanApplyViewModeOverrides = false;
				Mesh.bDisableBackfaceCulling = true;
				Collector.AddMesh(ViewIndex, Mesh);
			}
		}
	}

	virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView* View) const override
	{
		FPrimitiveViewRelevance Result;
		Result.bDrawRelevance = IsShown(View);
		Result.bShadowRelevance = IsShadowCast(View);
		Result.bDynamicRelevance = true;
		Result.bRenderInMainPass = ShouldRenderInMainPass();
		Result.bUsesLightingChannels = GetLightingChannelMask() != GetDefaultLightingChannelMask();
		Result.bRenderCustomDepth = ShouldRenderCustomDepth();
		Result.bTranslucentSelfShadow = bCastVolumetricTranslucentShadow;
		MaterialRelevance.SetPrimitiveViewRelevance(Result);
		Result.bVelocityRelevance = IsMovable() && Result.bOpaque && Result.bRenderInMainPass;
		return Result;
	}

	virtual bool CanBeOccluded() const override
	{
		return !MaterialRelevance.bDisableDepthTest;
	}

	virtual uint32 GetMemoryFootprint(void) const
	{
		return(sizeof(*this) + GetAllocatedSize());
	}

	uint32 GetAllocatedSize(void) const
	{
		return(FPrimitiveSceneProxy::GetAllocatedSize());
	}

	// mesh builder
	void SetDynamicData_RenderThread(FClothMeshPackedData const* NewData)
	{
		check(IsInRenderingThread());
		
		BuildMesh(NewData);
	}
	void BuildMesh(FClothMeshPackedData const* NewData)
	{
		// // Lock vertex buffer
		const int32 NumVerts = NewData->Positions->Num();
		
		// // Iterate through vertex data, update Position
		// for(int32 i=0; i<NumVerts; i++)
		// {
		// 	VertexBuffers.PositionVertexBuffer.VertexPosition(i) = (*NewData->Positions)[i];
		// }
		//
		// {
		// 	// upload to GPU, update position only
		// 	auto& VertexBuffer = VertexBuffers.PositionVertexBuffer;
		// 	void* VertexBufferData = RHILockVertexBuffer(VertexBuffer.VertexBufferRHI, 0, VertexBuffer.GetNumVertices() * VertexBuffer.GetStride(), RLM_WriteOnly);
		// 	FMemory::Memcpy(VertexBufferData, VertexBuffer.GetVertexData(), VertexBuffer.GetNumVertices() * VertexBuffer.GetStride());
		// 	RHIUnlockVertexBuffer(VertexBuffer.VertexBufferRHI);
		// }
		auto & RHICommands = GetImmediateCommandList_ForRenderCommand();
		TShaderMapRef<FTestCS> const TestCS(GetGlobalShaderMap(ERHIFeatureLevel::SM5));
		FRHIComputeShader* AddCS = TestCS.GetComputeShader();
		
		RHICommands.SetUAVParameter(AddCS, TestCS->Positions.GetBaseIndex(), PositionVertexBuffer.GetPositionUAV());
		RHICommands.SetComputeShader(AddCS);
		DispatchComputeShader(RHICommands, TestCS, 16, 1, 1);

		TArray<FVector> result;
		result.SetNumUninitialized(PositionVertexBuffer.GetNumVertices());
		uint8* data = (uint8*)RHILockVertexBuffer(PositionVertexBuffer.VertexBufferRHI, 0,  PositionVertexBuffer.GetDataSize(), RLM_ReadOnly);
		FMemory::Memcpy(result.GetData(), data, PositionVertexBuffer.GetDataSize());		
		RHIUnlockVertexBuffer(PositionVertexBuffer.VertexBufferRHI);
	}

	virtual  SIZE_T GetTypeHash() const override
	{
		static size_t UniquePointer;
		return reinterpret_cast<size_t>(&UniquePointer);
	}
private: 
	
	/** Material applied to this section */
	UMaterialInterface* Material;
	/** Vertex buffer for this section */
	// FStaticMeshVertexBuffers VertexBuffers;
	
	/** The buffer containing vertex data. */
	FStaticMeshVertexBuffer StaticMeshVertexBuffer;

	/** The buffer containing vertex color data. */
	FColorVertexBuffer ColorVertexBuffer;
	
	/** The buffer containing the position vertex data, which is dynamic */
	FRTDynamicVertexBuffer PositionVertexBuffer;
	
	/** Index buffer for this section */
	FDynamicMeshIndexBuffer32 IndexBuffer;
	
	/** Vertex factory for this section */
	FLocalVertexFactory VertexFactory;
	
	FMaterialRelevance MaterialRelevance;
};

//////////////////////////////////////////////////////////////////////////
URTClothMeshComponent::URTClothMeshComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	PrimaryComponentTick.bCanEverTick = true;
	bAutoActivate = true;
}

void URTClothMeshComponent::OnRegister()
{
	Super::OnRegister();
	TArray<UStaticMeshComponent*> Components;
	GetOwner()->GetComponents<UStaticMeshComponent>(Components);
	if (Components.Num() > 0)
	{
		Components[0]->SetHiddenInGame(true);
		auto const Mesh = Components[0]->GetStaticMesh();
		if (Mesh->IsValidLowLevel() && Mesh->GetRenderData()->LODResources.Num() > 0)
		{
			// get the original vertex data
			auto Mats = Components[0]->GetMaterials();
			if (Mats.Num() > 0)
			{
				SetMaterial(0, Mats[0]);
			}
			// build up our mesh
			ClothMesh = std::make_shared<FClothRawMesh>();
			ClothMesh->LocalToWorld = Components[0]->GetComponentTransform();
			if (!SetupCloth_CPU(Mesh)) {
				ENQUEUE_RENDER_COMMAND(URTClothMeshComponent_Setup_Mesh)(
				[this, Mesh](FRHICommandListImmediate &CmdList)
				{
					SetupCloth_RenderThread(Mesh);
				});
			}
			FlushRenderingCommands();
			// setup cloth solver system
			ClothSystem = std::make_unique<FRTClothSystem_ImplicitIntegration_CPU>(std::make_shared<FModifiedCGSolver>());
			//ClothSystem = std::make_unique<FRTClothSystem_Verlet_CPU>();
			//ClothSystem = std::make_unique<FRTClothSystem_Leapfrog_CPU>();
			ClothSystem->Init(ClothMesh,
				{1.0, 0.5, 100, 25, 0.3, 0.1, 1, 95, 95}
				);
			//ClothSystem.AddConstraint(0, {FClothConstraint::ELockingType::ConstraintOnPlane, {0, 0, 1}});
			ClothSystem->AddConstraint(0, {});
			ClothSystem->AddConstraint(38, {});
			ClothSystem->SetGravity({0, -7, 0});
			MarkRenderDynamicDataDirty();
		}
	};
}

bool URTClothMeshComponent::SetupCloth_CPU(UStaticMesh *Mesh) const
{
	const auto& LODResource = Mesh->GetRenderData()->LODResources[0];
	const auto& VB = LODResource.VertexBuffers.PositionVertexBuffer;
	const auto& SVB = LODResource.VertexBuffers.StaticMeshVertexBuffer;
	const auto& IB = LODResource.IndexBuffer;
	auto const NumVertex = VB.GetNumVertices();

	if ( VB.GetVertexData() != nullptr &&
		(IB.AccessStream16() != nullptr || IB.AccessStream32() != nullptr) &&
		SVB.GetTangentData() != nullptr && SVB.GetTexCoordData() != nullptr
	)
	{
		ClothMesh->Positions.Reserve(NumVertex);
		ClothMesh->TexCoords.Reserve(NumVertex);
		ClothMesh->TangentXArray.Reserve(NumVertex);
		ClothMesh->TangentYArray.Reserve(NumVertex);
		ClothMesh->TangentZArray.Reserve(NumVertex);
		ClothMesh->Colors.Reserve(NumVertex);
		for (uint32 i = 0; i < NumVertex; i ++)
		{
			// position
			ClothMesh->Positions.Add( ClothMesh->LocalToWorld.GetScale3D() * VB.VertexPosition(i));
			// UV
			ClothMesh->TexCoords.Add(SVB.GetVertexUV(i, 0));
			// tangent
			ClothMesh->TangentXArray.Add(SVB.VertexTangentX(i));
			ClothMesh->TangentYArray.Add(SVB.VertexTangentY(i));
			ClothMesh->TangentZArray.Add(SVB.VertexTangentZ(i));
			ClothMesh->Colors.Add(FColor::Cyan);
		}
		// get index data;
		auto const &IBData = IB.GetArrayView();
		auto const NumIndices = IB.GetNumIndices();
		ClothMesh->Indices.Reserve(NumIndices);
		for (int32 i = 0; i < NumIndices; i ++)
		{
			ClothMesh->Indices.Add(IBData[i]);
		}
		UE_LOG(LogTemp, Warning, TEXT("Building Cloth Mesh on CPU"));
		return true;
	}
	return false;
}

void URTClothMeshComponent::SetupCloth_RenderThread(UStaticMesh *Mesh) const
{
	const auto& LODResource = Mesh->GetRenderData()->LODResources[0];
	const auto& VB = LODResource.VertexBuffers.PositionVertexBuffer;
	const auto& SVB = LODResource.VertexBuffers.StaticMeshVertexBuffer;
	const auto& IB = LODResource.IndexBuffer;
	auto const NumVertex = VB.GetNumVertices();

	UE_LOG(LogTemp, Warning, TEXT("Building Cloth Mesh on GPU"));
	UE_LOG(LogTemp, Warning, TEXT("Vertex CPU Data %p"), VB.GetVertexData());
	UE_LOG(LogTemp, Warning, TEXT("Vertex Count %d"), NumVertex);
	UE_LOG(LogTemp, Warning, TEXT("Vertex Status %d"), VB.IsInitialized() && VB.VertexBufferRHI.IsValid());
	
	ClothMesh->Positions.SetNumZeroed(NumVertex);
	ClothMesh->TexCoords.Reserve(NumVertex);
	ClothMesh->TangentXArray.Reserve(NumVertex);
	ClothMesh->TangentYArray.Reserve(NumVertex);
	ClothMesh->TangentZArray.Reserve(NumVertex);
	ClothMesh->Colors.Reserve(NumVertex);
	
	// get index data;
	auto const NumIndices = IB.GetNumIndices();
	ClothMesh->Indices.SetNumZeroed(NumIndices);
	UE_LOG(LogTemp, Warning, TEXT("Indices 16 CPU Data %p"), IB.AccessStream16());
	UE_LOG(LogTemp, Warning, TEXT("Indices 32 CPU Data %p"), IB.AccessStream32());
	UE_LOG(LogTemp, Warning, TEXT("Indices Count %d"), NumIndices);
	UE_LOG(LogTemp, Warning, TEXT("Indices Status %d"), IB.IsInitialized() && IB.IndexBufferRHI.IsValid());
	// copy from RHI
	TArray<uint8> IndexRawData;
	IndexRawData.SetNumZeroed(NumIndices * sizeof(IB.IndexBufferRHI->GetStride()));
	void const* indexBufferData = RHILockIndexBuffer(IB.IndexBufferRHI, 0, NumIndices * IB.IndexBufferRHI->GetStride(), RLM_ReadOnly);
	FMemory::Memcpy(IndexRawData.GetData(), indexBufferData, NumIndices * IB.IndexBufferRHI->GetStride());
	RHIUnlockIndexBuffer(IB.IndexBufferRHI);
	
	// copy to cloth mesh
	for (int32 i = 0; i < NumIndices; i ++)
	{
		if (IB.IndexBufferRHI->GetStride() == 2)
		{
			ClothMesh->Indices[i] = uint32(((uint16 *)IndexRawData.GetData())[i]);
		} else
		{
			ClothMesh->Indices[i] = ((uint32 *)IndexRawData.GetData())[i];
		}
	}
	
	// Position
	void const* PosBufferData = RHILockVertexBuffer(VB.VertexBufferRHI, 0, NumVertex *  VB.GetStride(), RLM_ReadOnly);
	FMemory::Memcpy(ClothMesh->Positions.GetData(), PosBufferData, NumVertex * sizeof(FVector));
	RHIUnlockVertexBuffer(VB.VertexBufferRHI);
	
	// UV
	auto const &UVBuffer = SVB.TexCoordVertexBuffer;
	struct HalfUV
	{
		FFloat16 U;
		FFloat16 V;
	};
	TArray<HalfUV> uvdata;
	uvdata.SetNumZeroed(NumVertex * SVB.GetNumTexCoords());
	
	void const* UVBufferData = RHILockVertexBuffer(UVBuffer.VertexBufferRHI, 0, NumVertex * SVB.GetNumTexCoords() * sizeof(HalfUV), RLM_ReadOnly);
	FMemory::Memcpy(uvdata.GetData(), UVBufferData, NumVertex * SVB.GetNumTexCoords() * sizeof(HalfUV));
	RHIUnlockVertexBuffer(UVBuffer.VertexBufferRHI);
	
	for (uint32 i = 0; i < NumVertex; i ++)
	{
		auto &uv = uvdata[i * SVB.GetNumTexCoords()];
		ClothMesh->TexCoords.Add({uv.U.GetFloat(), uv.V.GetFloat()});
	}
	
	// tangent
	auto const &TangBuffer = SVB.TangentsVertexBuffer;
	struct TangentType
	{
		FPackedNormal X;
		FPackedNormal Z;
		FVector GetTangentX() const
		{
			return X.ToFVector();
		}
		FVector4 GetTangentZ() const
		{
			return Z.ToFVector4();
		}
		FVector GetTangentY() const
		{
			FVector  x = X.ToFVector();
			FVector4 z = Z.ToFVector4();
			return (FVector(z) ^ x) * z.W;
		}
	};
	TArray<TangentType> PTdata;
	PTdata.SetNumZeroed(NumVertex);
	void const* TangBufferData = RHILockVertexBuffer(TangBuffer.VertexBufferRHI, 0, NumVertex * sizeof(TangentType), RLM_ReadOnly);
	FMemory::Memcpy(PTdata.GetData(), TangBufferData, NumVertex * sizeof(TangentType));
	RHIUnlockVertexBuffer(TangBuffer.VertexBufferRHI);
	
	for (uint32 i = 0; i < NumVertex; i ++)
	{
		// tangent
		ClothMesh->TangentXArray.Add(PTdata[i].GetTangentX());
		ClothMesh->TangentYArray.Add(PTdata[i].GetTangentY());
		ClothMesh->TangentZArray.Add(PTdata[i].GetTangentZ());
		ClothMesh->Colors.Add(FColor::Cyan);
	}
}
#include <iostream>
void URTClothMeshComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// get static mesh and material
	TArray<UStaticMeshComponent*> Components;
	GetOwner()->GetComponents<UStaticMeshComponent>(Components);
	if (Components.Num() > 0 && ClothMesh != nullptr)
	{
		// update attached transform, TODO : external force here
		ClothMesh->LocalToWorld = Components[0]->GetComponentTransform();
	}
	ENQUEUE_RENDER_COMMAND(URTClothMeshComponentTick)(
	[this, DeltaTime](FRHICommandListImmediate &CmdList)
	{
		//ClothSystem->TickOnce(std::max(0.001f, std::min(DeltaTime, 0.05f)));
		ClothSystem->TickOnce(0.01);
	});
	
	// Need to send new data to render thread
	MarkRenderDynamicDataDirty();
	UpdateComponentToWorld();
}

// override scene proxy
FPrimitiveSceneProxy *URTClothMeshComponent::CreateSceneProxy()
{
	return new FClothMeshSceneProxy(this, ClothMesh);
}

// get num of materials
int32 URTClothMeshComponent::GetNumMaterials() const
{
	return 1;
}

// override get bounds
FBoxSphereBounds URTClothMeshComponent::CalcBounds(const FTransform & LocalToWorld) const
{
	FBoxSphereBounds NewBounds;
	NewBounds.Origin = FVector::ZeroVector;
	NewBounds.BoxExtent = FVector(HALF_WORLD_MAX, HALF_WORLD_MAX, HALF_WORLD_MAX);
	NewBounds.SphereRadius = FMath::Sqrt(3.0f * FMath::Square(HALF_WORLD_MAX));
	return NewBounds;
}

// Render Data Pass Through
void URTClothMeshComponent::SendRenderDynamicData_Concurrent()
{
	if (SceneProxy)
	{
		ENQUEUE_RENDER_COMMAND(URTClothMeshComponentUpdate)(
			[this](FRHICommandListImmediate &CmdList)
			{
				FClothMeshPackedData Data;
				Data.Positions = &ClothMesh->Positions;
				auto const ProcProxy = static_cast<FClothMeshSceneProxy *>(SceneProxy);
				if (ProcProxy)
				{
					ProcProxy->SetDynamicData_RenderThread(&Data);
				}
			}
		);
	}
}
void URTClothMeshComponent::CreateRenderState_Concurrent(FRegisterComponentContext* Context)
{
	Super::CreateRenderState_Concurrent(Context);
	SendRenderDynamicData_Concurrent();
}