#pragma once

#include <memory>

#include "RTClothStructures.h"
#include "FRTClothSystemBase.h"

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "Components/MeshComponent.h"
#include "URTClothMeshComponent.generated.h"

class UStaticMesh;
class FPrimitiveSceneProxy;
//This is a mesh effect component
UCLASS(hidecategories = (Object, LOD, Physics, Collision), editinlinenew, meta = (BlueprintSpawnableComponent), ClassGroup = Rendering, DisplayName = "URTClothMeshComponent")
class URTClothMeshComponent : public UMeshComponent
{
	GENERATED_UCLASS_BODY()

public:

private:
	// setup cloth mesh and cloth system in RenderThread
	bool SetupCloth_CPU(UStaticMesh *OriginalMesh) const;
	void SetupCloth_RenderThread(UStaticMesh *OriginalMesh) const;
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
	std::shared_ptr<FClothRawMesh> ClothMesh;
	std::unique_ptr<FRTClothSystemBase> ClothSystem;
};