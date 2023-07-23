#include "FLocalForceAssemplerCS.h"

#include "ShaderCompilerCore.h"
#include "RHIStaticStates.h"

FLocalForceAssemplerCS::FLocalForceAssemplerCS(const ShaderMetaType::CompiledShaderInitializerType& Initializer) : FGlobalShader(Initializer)
{
	LocalForces.Bind(Initializer.ParameterMap, TEXT("LocalForces"));
	Forces.Bind(Initializer.ParameterMap, TEXT("Forces"));
	PreSumOfRefIndices.Bind(Initializer.ParameterMap, TEXT("PreSumOfRefIndices"));
	SubForceIndices.Bind(Initializer.ParameterMap, TEXT("SubForceIndices"));
}

void FLocalForceAssemplerCS::ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
{
	FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
	OutEnvironment.CompilerFlags.Add(CFLAG_StandardOptimization);
}

IMPLEMENT_SHADER_TYPE(, FLocalForceAssemplerCS, TEXT("/Plugins/RTClothMesh/Shaders/ForceAssembler.usf"), TEXT("AssembleForce"), SF_Compute);