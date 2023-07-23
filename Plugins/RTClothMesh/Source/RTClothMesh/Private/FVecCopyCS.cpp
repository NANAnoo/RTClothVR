#include "FVecCopyCS.h"

#include "ShaderCompilerCore.h"
#include "RHIStaticStates.h"

FVecCopyCS::FVecCopyCS(const ShaderMetaType::CompiledShaderInitializerType& Initializer) : FGlobalShader(Initializer)
{
	InVec.Bind(Initializer.ParameterMap, TEXT("InVec"));
	OutVec.Bind(Initializer.ParameterMap, TEXT("OutVec"));
}

void FVecCopyCS::ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
{
	FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
	OutEnvironment.CompilerFlags.Add(CFLAG_StandardOptimization);
}

IMPLEMENT_SHADER_TYPE(, FVecCopyCS, TEXT("/Plugins/RTClothMesh/Shaders/VectorCopyCS.usf"), TEXT("CopyVec"), SF_Compute);