#include "URTClothMeshComponent.h"

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
	for( int32 i=0; i<Components.Num(); i++ )
	{
		Components[i]->SetHiddenInGame(true);
	}
	MarkRenderDynamicDataDirty();
}

void URTClothMeshComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	
	// Need to send new data to render thread
	MarkRenderDynamicDataDirty();

	UpdateComponentToWorld();
}

// override scene proxy