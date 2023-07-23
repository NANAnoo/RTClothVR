#pragma once

#include "CoreMinimal.h"

#include "GlobalShader.h"
#include "UniformBuffer.h"
#include "RHICommandList.h"

#include <atomic>

class FLocalForceAssemplerCS : public FGlobalShader
{
	DECLARE_SHADER_TYPE(FLocalForceAssemplerCS, Global);

	FLocalForceAssemplerCS() {}

	explicit FLocalForceAssemplerCS(const ShaderMetaType::CompiledShaderInitializerType& Initializer);

	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters) {
		return GetMaxSupportedFeatureLevel(Parameters.Platform) >= ERHIFeatureLevel::SM5;
	};

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment);
	
	LAYOUT_FIELD(FShaderResourceParameter, LocalForces);
	LAYOUT_FIELD(FShaderResourceParameter, Forces);
	LAYOUT_FIELD(FShaderResourceParameter, PreSumOfRefIndices);
	LAYOUT_FIELD(FShaderResourceParameter, SubForceIndices);
};