#include "FInnerCollisionForcesCS.h"

#include "ShaderCompilerCore.h"
#include "RHIStaticStates.h"

FInnerCollisionForcesCS::FInnerCollisionForcesCS(const ShaderMetaType::CompiledShaderInitializerType& Initializer) : FGlobalShader(Initializer)
{
	SimParams.Bind(Initializer.ParameterMap, TEXT("SimParams"));
	InnerHitBVH.Bind(Initializer.ParameterMap, TEXT("InnerHitBVH"));
	Masses.Bind(Initializer.ParameterMap, TEXT("Masses"));
	Indices.Bind(Initializer.ParameterMap, TEXT("Indices"));
	Positions.Bind(Initializer.ParameterMap, TEXT("Positions"));
	Velocities.Bind(Initializer.ParameterMap, TEXT("Velocities"));
	Forces.Bind(Initializer.ParameterMap, TEXT("Forces"));
}

void FInnerCollisionForcesCS::ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
{
	FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
	OutEnvironment.CompilerFlags.Add(CFLAG_StandardOptimization);
}

IMPLEMENT_SHADER_TYPE(, FInnerCollisionForcesCS, TEXT("/Plugins/RTClothMesh/Shaders/InnerCollision.usf"), TEXT("Solve"), SF_Compute);