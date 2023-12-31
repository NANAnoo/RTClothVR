﻿#pragma once

#include "CoreMinimal.h"

#include "GlobalShader.h"
#include "UniformBuffer.h"
#include "RHICommandList.h"

#include <atomic>

class FInnerCollisionForcesCS : public FGlobalShader
{
	DECLARE_SHADER_TYPE(FInnerCollisionForcesCS, Global);

	FInnerCollisionForcesCS() {}

	explicit FInnerCollisionForcesCS(const ShaderMetaType::CompiledShaderInitializerType& Initializer);

	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters) {
		return GetMaxSupportedFeatureLevel(Parameters.Platform) >= ERHIFeatureLevel::SM5;
	};

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment);
	
	LAYOUT_FIELD(FShaderUniformBufferParameter, SimParams);
	LAYOUT_FIELD(FShaderResourceParameter, InnerHitBVH);
	LAYOUT_FIELD(FShaderResourceParameter, Masses);
	LAYOUT_FIELD(FShaderResourceParameter, Indices);
	LAYOUT_FIELD(FShaderResourceParameter, Positions);
	LAYOUT_FIELD(FShaderResourceParameter, Velocities);
	LAYOUT_FIELD(FShaderResourceParameter, Forces);
};