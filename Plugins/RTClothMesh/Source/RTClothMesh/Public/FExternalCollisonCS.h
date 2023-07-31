#pragma once

#include "CoreMinimal.h"

#include "GlobalShader.h"
#include "UniformBuffer.h"
#include "RHICommandList.h"

#include <atomic>

class FExternalCollisionCS : public FGlobalShader
{
	DECLARE_SHADER_TYPE(FExternalCollisionCS, Global);

	FExternalCollisionCS() {}

	explicit FExternalCollisionCS(const ShaderMetaType::CompiledShaderInitializerType& Initializer);

	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters) {
		return GetMaxSupportedFeatureLevel(Parameters.Platform) >= ERHIFeatureLevel::SM5;
	};

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment);
	
	LAYOUT_FIELD(FShaderUniformBufferParameter, SimParams);
	LAYOUT_FIELD(FShaderResourceParameter, ExColliders);
	LAYOUT_FIELD(FShaderResourceParameter, ConstraintMap);
	LAYOUT_FIELD(FShaderResourceParameter, Velocities);
	LAYOUT_FIELD(FShaderResourceParameter, Pre_Positions);
	LAYOUT_FIELD(FShaderResourceParameter, Positions);
};