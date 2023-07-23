#include "FVerletIntegratorCS.h"

#include "ShaderCompilerCore.h"
#include "RHIStaticStates.h"

FVerletIntegratorCS::FVerletIntegratorCS(const ShaderMetaType::CompiledShaderInitializerType& Initializer) : FGlobalShader(Initializer)
{
	SimParams.Bind(Initializer.ParameterMap, TEXT("SimParams"));
	Velocities.Bind(Initializer.ParameterMap, TEXT("Velocities"));
	Pre_Positions.Bind(Initializer.ParameterMap, TEXT("Pre_Positions"));
	Positions.Bind(Initializer.ParameterMap, TEXT("Positions"));
	Masses.Bind(Initializer.ParameterMap, TEXT("Masses"));
	Forces.Bind(Initializer.ParameterMap, TEXT("Forces"));
}

void FVerletIntegratorCS::ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
{
	FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
	OutEnvironment.CompilerFlags.Add(CFLAG_StandardOptimization);
}

IMPLEMENT_SHADER_TYPE(, FVerletIntegratorCS, TEXT("/Plugins/RTClothMesh/Shaders/VerletIntegrator.usf"), TEXT("VerletIntegration"), SF_Compute);