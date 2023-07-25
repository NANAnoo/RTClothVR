#include "FRTBendForceCS.h"

#include "ShaderCompilerCore.h"
#include "RHIStaticStates.h"

FRTBendForceCS::FRTBendForceCS(const ShaderMetaType::CompiledShaderInitializerType& Initializer) : FGlobalShader(Initializer)
{
	SimParams.Bind(Initializer.ParameterMap, TEXT("SimParams"));
	BendConditions.Bind(Initializer.ParameterMap, TEXT("BendConditions"));
	SharedEdgeLengths.Bind(Initializer.ParameterMap, TEXT("SharedEdgeLengths"));
	Velocities.Bind(Initializer.ParameterMap, TEXT("Velocities"));
	Positions.Bind(Initializer.ParameterMap, TEXT("Positions"));
	LocalForces.Bind(Initializer.ParameterMap, TEXT("LocalForces"));
}

void FRTBendForceCS::ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
{
	FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
	OutEnvironment.CompilerFlags.Add(CFLAG_StandardOptimization);
}

IMPLEMENT_SHADER_TYPE(, FRTBendForceCS, TEXT("/Plugins/RTClothMesh/Shaders/BendForces.usf"), TEXT("UpdateForces"), SF_Compute);