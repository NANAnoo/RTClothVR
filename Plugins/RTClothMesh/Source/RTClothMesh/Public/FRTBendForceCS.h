#pragma once

#include "CoreMinimal.h"

#include "GlobalShader.h"
#include "UniformBuffer.h"
#include "RHICommandList.h"

#include <atomic>

class FRTBendForceCS : public FGlobalShader
{
	DECLARE_SHADER_TYPE(FRTBendForceCS, Global);

	FRTBendForceCS() {}

	explicit FRTBendForceCS(const ShaderMetaType::CompiledShaderInitializerType& Initializer);

	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters) {
		return GetMaxSupportedFeatureLevel(Parameters.Platform) >= ERHIFeatureLevel::SM5;
	};

	static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment);

	LAYOUT_FIELD(FShaderUniformBufferParameter, SimParams);
	LAYOUT_FIELD(FShaderResourceParameter, BendConditions);
	LAYOUT_FIELD(FShaderResourceParameter, SharedEdgeLengths);
	LAYOUT_FIELD(FShaderResourceParameter, Velocities);
	LAYOUT_FIELD(FShaderResourceParameter, Positions);
	LAYOUT_FIELD(FShaderResourceParameter, LocalForces);
};