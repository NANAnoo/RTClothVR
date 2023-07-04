#pragma once

#include <memory>

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "Components/MeshComponent.h"
#include "URTClothMeshComponent.generated.h"


class UStaticMesh;
class FPrimitiveSceneProxy;

struct FClothRawMesh
{
	FClothRawMesh(){}
	// disable copy
	FClothRawMesh(FClothRawMesh &) = delete;
	FClothRawMesh operator=(FClothRawMesh &) = delete;
	// enable move
	FClothRawMesh (FClothRawMesh &&other) noexcept
	{
		Positions = std::move(other.Positions);
		TexCoords = std::move(other.TexCoords);
		Indices = std::move(other.Indices);
	}
	
	FClothRawMesh& operator=(FClothRawMesh &&other) noexcept
	{
		*this = std::move(other);
		return *this;
	}
	TArray<FVector> Positions;
	TArray<FVector2D> TexCoords;
	TArray<uint32> Indices;

	TArray<FVector> GetSmoothNormals()
	{
		return Positions;
	}
};

//This is a mesh effect component
UCLASS(hidecategories = (Object, LOD, Physics, Collision), editinlinenew, meta = (BlueprintSpawnableComponent), ClassGroup = Rendering, DisplayName = "URTClothMeshComponent")
class URTClothMeshComponent : public UMeshComponent
{
	GENERATED_UCLASS_BODY()

public:

private:
	
	// create a scene proxy
	virtual FPrimitiveSceneProxy *CreateSceneProxy() override;

	// get num of materials
	virtual int32 GetNumMaterials() const override;

	// override get bounds
	virtual FBoxSphereBounds CalcBounds(const FTransform &) const override;

	// Component Life Cycles
	void OnRegister() override;
	void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction)override;

	// Render Data Pass Through
	virtual void SendRenderDynamicData_Concurrent() override;
	virtual void CreateRenderState_Concurrent(FRegisterComponentContext* Context) override;
	std::unique_ptr<FClothRawMesh> ClothMesh;
};