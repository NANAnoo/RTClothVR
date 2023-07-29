#pragma once

#include <memory>

#include "RTClothStructures.h"
#include "FRTClothSystemBase.h"

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "Components/MeshComponent.h"
#include "Components/BoxComponent.h"
#include "URTClothMeshComponent.generated.h"

class UStaticMesh;
class FPrimitiveSceneProxy;
//This is a mesh effect component
UCLASS(hidecategories = (Object, LOD, Physics, Collision), editinlinenew, meta = (BlueprintSpawnableComponent), ClassGroup = Rendering, DisplayName = "URTClothMeshComponent")
class URTClothMeshComponent : public UMeshComponent
{
	GENERATED_UCLASS_BODY()

public:
	UPROPERTY(EditAnywhere, Category = ClothParameters, meta=(DisplayName="K_Stretch", ClampMin="0", ClampMax="1000"))
	float K_Stretch = 100;

	UPROPERTY(EditAnywhere, Category = ClothParameters, meta=(DisplayName="D_Stretch", ClampMin="0", ClampMax="100"))
	float D_Stretch = 6;

	UPROPERTY(EditAnywhere, Category = ClothParameters, meta=(DisplayName="K_Shear", ClampMin="0", ClampMax="1"))
	float K_Shear = 0;

	UPROPERTY(EditAnywhere, Category = ClothParameters, meta=(DisplayName="D_Shear", ClampMin="0", ClampMax="1"))
	float D_Shear = 0;

	UPROPERTY(EditAnywhere, Category = ClothParameters, meta=(DisplayName="K_Bend", ClampMin="0", ClampMax="100"))
	float K_Bend = 0.1;

	UPROPERTY(EditAnywhere, Category = ClothParameters, meta=(DisplayName="D_Bend", ClampMin="0", ClampMax="100"))
	float D_Bend = 0;

	UPROPERTY(EditAnywhere, Category = ClothParameters, meta=(DisplayName="Density", ClampMin="0", ClampMax="1"))
	float Density = 1;

	UPROPERTY(EditAnywhere, Category = ClothParameters, meta=(DisplayName="Rest_U", ClampMin="0", ClampMax="1000"))
	float Rest_U = 100;

	UPROPERTY(EditAnywhere, Category = ClothParameters, meta=(DisplayName="Rest_V", ClampMin="0", ClampMax="1000"))
	float Rest_V = 100;
	
	UPROPERTY(EditAnywhere, Category = ClothParameters, meta=(DisplayName="InitTheta", ClampMin="-180", ClampMax="180"))
	float InitTheta = 0;
	
	UPROPERTY(EditAnywhere, Category = ClothParameters, meta=(DisplayName="K_Collision", ClampMin="0", ClampMax="1000"))
	float K_Collision = 10;
	
	UPROPERTY(EditAnywhere, Category = ClothParameters, meta=(DisplayName="D_Collision", ClampMin="0", ClampMax="1000"))
	float D_Collision = 1;

	UPROPERTY(EditAnywhere, Category = ClothParameters, meta=(DisplayName="Inner Collision"))
	bool EnableInnerCollision = true;
	
	UPROPERTY(EditAnywhere, Category = ClothParameters, meta=(DisplayName="Collision"))
	bool EnableCollision = true;

	UPROPERTY(EditAnywhere, Category = ClothParameters, meta=(DisplayName="Friction", ClampMin="0", ClampMax="1"))
	float Friction = 0.3;

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
	virtual void OnRegister() override;
	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction)override;
	virtual void BeginPlay() override;
	
	// Render Data Pass Through
	virtual void SendRenderDynamicData_Concurrent() override;
	virtual void CreateRenderState_Concurrent(FRegisterComponentContext* Context) override;
	std::shared_ptr<FClothRawMesh> ClothMesh;
	std::unique_ptr<FRTClothSystemBase> ClothSystem;

	UPROPERTY(EditInstanceOnly)
	UBoxComponent *HitBox;
	
	UFUNCTION()
	void OnOverLapBegin(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult & SweepResult);

	UFUNCTION()
	void OnOverLapEnd(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex);
};