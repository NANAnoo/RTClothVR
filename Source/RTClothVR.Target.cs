// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.Collections.Generic;

public class RTClothVRTarget : TargetRules
{
	public RTClothVRTarget( TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
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
