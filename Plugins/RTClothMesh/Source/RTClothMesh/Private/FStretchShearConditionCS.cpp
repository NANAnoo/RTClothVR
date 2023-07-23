#include "FStretchShearConditionCS.h"

#include "ShaderCompilerCore.h"
#include "RHIStaticStates.h"

FStretchShearConditionCS::FStretchShearConditionCS(const ShaderMetaType::CompiledShaderInitializerType& Initializer) : FGlobalShader(Initializer)
{
	SimParams.Bind(Initializer.ParameterMap, TEXT("SimParams"));
	ConditionBasis.Bind(Initializer.ParameterMap, TEXT("ConditionBasis"));
	Velocities.Bind(Initializer.ParameterMap, TEXT("Velocities"));
	Positions.Bind(Initializer.ParameterMap, TEXT("Positions"));
	LocalForces.Bind(Initializer.ParameterMap, TEXT("LocalForces"));
}

void FStretchShearConditionCS::ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
{
	FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
	OutEnvironment.CompilerFlags.Add(CFLAG_StandardOptimization);
}

IMPLEMENT_SHADER_TYPE(, FStretchShearConditionCS, TEXT("/Plugins/RTClothMesh/Shaders/StretchAndShearForces.usf"), TEXT("UpdateForces"), SF_Compute);