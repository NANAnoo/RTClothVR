#include "FExternalCollisonCS.h"

#include "ShaderCompilerCore.h"
#include "RHIStaticStates.h"

FExternalCollisionCS::FExternalCollisionCS(const ShaderMetaType::CompiledShaderInitializerType& Initializer) : FGlobalShader(Initializer)
{
	SimParams.Bind(Initializer.ParameterMap, TEXT("SimParams"));
	ExColliders.Bind(Initializer.ParameterMap, TEXT("ExColliders"));
	ConstraintMap.Bind(Initializer.ParameterMap, TEXT("ConstraintMap"));
	Velocities.Bind(Initializer.ParameterMap, TEXT("Velocities"));
	Pre_Positions.Bind(Initializer.ParameterMap, TEXT("Pre_Positions"));
	Positions.Bind(Initializer.ParameterMap, TEXT("Positions"));
}

void FExternalCollisionCS::ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
{
	FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
	OutEnvironment.CompilerFlags.Add(CFLAG_StandardOptimization);
}

IMPLEMENT_SHADER_TYPE(, FExternalCollisionCS, TEXT("/Plugins/RTClothMesh/Shaders/ExternalCollision.usf"), TEXT("Solve"), SF_Compute);