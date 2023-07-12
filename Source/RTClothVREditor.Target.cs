// Copyright Epic Games, Inc. All Rights Reserved.

using System;
using UnrealBuildTool;
using System.Collections.Generic;

public class RTClothVREditorTarget : TargetRules
{
	public RTClothVREditorTarget( TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.V2;
		ExtraModuleNames.AddRange( new string[] { "RTClothVR" } );

		if (UnrealTargetPlatform.Mac !=Target.Platform )
		{
			return;
		}
		bOverrideBuildEnvironment = true;
		AdditionalCompilerArguments = " -Wno-bitwise-instead-of-logical -Wno-unused-but-set-variable";
	}
}
