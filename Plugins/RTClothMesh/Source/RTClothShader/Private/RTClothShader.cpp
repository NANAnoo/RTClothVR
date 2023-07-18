#include "RTClothShader.h"

#include "Interfaces/IPluginManager.h"

#define LOCTEXT_NAMESPACE "FRTClothShaderModule"

void FRTClothShaderModule::StartupModule()
{
	FString const PluginShaderDir = FPaths::Combine(IPluginManager::Get().FindPlugin(TEXT("RTClothMesh"))->GetBaseDir(), TEXT("Shaders"));
	AddShaderSourceDirectoryMapping(TEXT("/Plugins/RTClothMesh/Shaders"), PluginShaderDir);
}

void FRTClothShaderModule::ShutdownModule()
{
    
}

#undef LOCTEXT_NAMESPACE
    
IMPLEMENT_MODULE(FRTClothShaderModule, RTClothShader)