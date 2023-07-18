#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class FRTClothShaderModule : public IModuleInterface
{
public:
    // register shader path
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};
