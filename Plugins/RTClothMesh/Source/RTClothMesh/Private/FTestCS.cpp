#include "FTestCS.h"

#include "ShaderCompilerCore.h"
#include "RHIStaticStates.h"

FTestCS::FTestCS(const ShaderMetaType::CompiledShaderInitializerType& Initializer) : FGlobalShader(Initializer)
{
	Positions.Bind(Initializer.ParameterMap, TEXT("Positions"));
	// NumOfVertices.Bind(Initializer.ParameterMap, TEXT("NumIndices"));
}

void FTestCS::ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
{
	FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
	OutEnvironment.CompilerFlags.Add(CFLAG_StandardOptimization);
}

IMPLEMENT_SHADER_TYPE(, FTestCS, TEXT("/Plugins/RTClothMesh/Shaders/Test.usf"), TEXT("PositionUpdate"), SF_Compute);