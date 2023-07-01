#pragma once

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
	// create a scene proxy
	virtual FPrimitiveSceneProxy *CreateSceneProxy() override;

	// get num of materials
	virtual int32 GetNumMaterials() const override;

	// override get bounds
	virtual FBoxSphereBounds CalcLocalBounds() const override;

	// Component Life Cycles
	void OnRegister() override;
	void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction)override;

	// Render Data Pass Through
	virtual void SendRenderDynamicData_Concurrent() override;
	virtual void CreateRenderState_Concurrent(FRegisterComponentContext* Context) override;
	
};