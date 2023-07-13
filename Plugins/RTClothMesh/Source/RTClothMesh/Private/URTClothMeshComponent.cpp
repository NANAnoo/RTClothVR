#include "URTClothMeshComponent.h"

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

// data pack
struct FClothMeshPackedData
{
	TArray<FVector> *Positions;
	TArray<uint32> *Indices;
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

		// Allocate verts

		TArray<FDynamicMeshVertex> Vertices;
		Vertices.SetNumUninitialized(NumVerts);
		// Copy verts
		for (int VertIdx = 0; VertIdx < NumVerts; VertIdx++)
		{
			FDynamicMeshVertex& Vert = Vertices[VertIdx];
			Vert.Position = ClothMesh->Positions[VertIdx];
			Vert.Color = ClothMesh->Colors[VertIdx];
			Vert.TextureCoordinate[0] = ClothMesh->TexCoords[VertIdx];
			Vert.SetTangents(
				ClothMesh->TangentXArray[VertIdx],
				ClothMesh->TangentYArray[VertIdx],
				ClothMesh->TangentZArray[VertIdx]
			);
		}

		// Copy index buffer
		IndexBuffer.Indices = ClothMesh->Indices;

		VertexBuffers.InitFromDynamicVertex(&VertexFactory, Vertices);

		// Enqueue initialization of render resource
		BeginInitResource(&VertexBuffers.PositionVertexBuffer);
		BeginInitResource(&VertexBuffers.StaticMeshVertexBuffer);
		BeginInitResource(&VertexBuffers.ColorVertexBuffer);
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
		VertexBuffers.PositionVertexBuffer.ReleaseResource();
		VertexBuffers.StaticMeshVertexBuffer.ReleaseResource();
		VertexBuffers.ColorVertexBuffer.ReleaseResource();
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
				BatchElement.MaxVertexIndex = VertexBuffers.PositionVertexBuffer.GetNumVertices() - 1;
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
		// Lock vertex buffer
		const int32 NumVerts = NewData->Positions->Num();
	
		// Iterate through vertex data, update Position
		for(int32 i=0; i<NumVerts; i++)
		{
			VertexBuffers.PositionVertexBuffer.VertexPosition(i) = (*NewData->Positions)[i];
		}
		
		{
			// upload to GPU, update position only
			auto& VertexBuffer = VertexBuffers.PositionVertexBuffer;
			void* VertexBufferData = RHILockVertexBuffer(VertexBuffer.VertexBufferRHI, 0, VertexBuffer.GetNumVertices() * VertexBuffer.GetStride(), RLM_WriteOnly);
			FMemory::Memcpy(VertexBufferData, VertexBuffer.GetVertexData(), VertexBuffer.GetNumVertices() * VertexBuffer.GetStride());
			RHIUnlockVertexBuffer(VertexBuffer.VertexBufferRHI);
		}
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
	FStaticMeshVertexBuffers VertexBuffers;
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
	bTickInEditor = true;
	bAutoActivate = true;
}

void URTClothMeshComponent::OnRegister()
{
	Super::OnRegister();
	
	// get static mesh and material
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
			const auto& LODResource = Mesh->GetRenderData()->LODResources[0];
			const auto& VB = LODResource.VertexBuffers.PositionVertexBuffer;
			const auto& SVB = LODResource.VertexBuffers.StaticMeshVertexBuffer;
			const auto& CVB = LODResource.VertexBuffers.ColorVertexBuffer;
			const auto& IB = LODResource.IndexBuffer;
			// build up our mesh
			ClothMesh = std::make_shared<FClothRawMesh>();
			// get vertex data
			auto const NumVertex = VB.GetNumVertices();
			const bool HasColor = CVB.IsInitialized() && CVB.GetNumVertices() == NumVertex;
			ClothMesh->Positions.Reserve(NumVertex);
			ClothMesh->TexCoords.Reserve(NumVertex);
			ClothMesh->TangentXArray.Reserve(NumVertex);
			ClothMesh->TangentYArray.Reserve(NumVertex);
			ClothMesh->TangentZArray.Reserve(NumVertex);
			ClothMesh->Colors.Reserve(NumVertex);
			ClothMesh->LocalToWorld = Components[0]->GetComponentTransform();
			auto CurT = GetComponentTransform();
			for (uint32 i = 0; i < NumVertex; i ++)
			{
				// position
				ClothMesh->Positions.Add( ClothMesh->LocalToWorld.GetScale3D() * VB.VertexPosition(i));
				// UV
				ClothMesh->TexCoords.Add(SVB.GetVertexUV(i, 0));
				// tangent, TODO Check W in tangents X and Z
				ClothMesh->TangentXArray.Add(SVB.VertexTangentX(i));
				ClothMesh->TangentYArray.Add(SVB.VertexTangentY(i));
				ClothMesh->TangentZArray.Add(SVB.VertexTangentZ(i));
				ClothMesh->Colors.Add(HasColor ? CVB.VertexColor(i) : FColor::Cyan);
			}
			// get index data;
			auto const &IBData = IB.GetArrayView();
			auto const NumIndices = IB.GetNumIndices();
			ClothMesh->Indices.Reserve(NumIndices);
			for (int32 i = 0; i < NumIndices; i ++)
			{
				ClothMesh->Indices.Add(IBData[i]);
			}
		}
	}
	// setup cloth solver system
	ClothSystem.Init(ClothMesh,
		{0, 0, 100, 10, 0.2, 0.05, 1},
		std::make_shared<FModifiedCGSolver>()
		);
	//ClothSystem.AddConstraint(0, {FClothConstraint::ELockingType::ConstraintOnPlane, {0, 0, 1}});
	ClothSystem.AddConstraint(0, {});
	ClothSystem.AddConstraint(95, {});
	MarkRenderDynamicDataDirty();
}

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
		ClothSystem.TickOnce(0.001);
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
				Data.Indices = &ClothMesh->Indices;
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