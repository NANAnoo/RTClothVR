#pragma once

#include "CoreMinimal.h"

#include "GlobalShader.h"
#include "UniformBuffer.h"
#include "RHICommandList.h"

#include <atomic>

class FVecCopyCS : public FGlobalShader
{
	DECLARE_SHADER_TYPE(FVecCopyCS, Global);

	FVecCopyCS() {}

	explicit FVecCopyCS(const ShaderMetaType::CompiledShaderInitializerType& Initializer);

	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters) {
		return GetMaxSupportedFeatureLevel(Parameters.Platform) >= ERHIFeatureLevel::SM5;
	};

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment);
\
	LAYOUT_FIELD(FShaderResourceParameter, InVec);
	LAYOUT_FIELD(FShaderResourceParameter, OutVec);
};