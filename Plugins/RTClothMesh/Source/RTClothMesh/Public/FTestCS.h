#pragma once

#include "CoreMinimal.h"

#include "GlobalShader.h"
#include "UniformBuffer.h"
#include "RHICommandList.h"

#include <atomic>

class FTestCS : public FGlobalShader
{
	DECLARE_SHADER_TYPE(FTestCS, Global);

	FTestCS() {}

	explicit FTestCS(const ShaderMetaType::CompiledShaderInitializerType& Initializer);

	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters) {
		return GetMaxSupportedFeatureLevel(Parameters.Platform) >= ERHIFeatureLevel::SM5;
	};

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment);

	LAYOUT_FIELD(FShaderResourceParameter, Positions);
	//LAYOUT_FIELD(FShaderUniformBufferParameter, NumOfVertices);
};