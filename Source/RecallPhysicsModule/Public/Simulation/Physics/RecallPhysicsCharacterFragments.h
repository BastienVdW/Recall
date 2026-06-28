// Copyright (C) 2024 Van de Walle Bastien
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "Mass/EntityElementTypes.h"
#include "Physics/Character/JPRPhysicsCharacterShapeTypes.h"
#include "Physics/Character/JPRPhysicsCharacterTypes.h"
#include "Physics/RecallPhysicsTypes.h"

#include "RecallPhysicsCharacterFragments.generated.h"

USTRUCT()
struct RECALLPHYSICSMODULE_API FJPRPhysicsCharacterFragment :
	public FMassFragment
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere)
	bool bIsSupported = true;

	UPROPERTY(VisibleAnywhere)
	EJPRPhysicsCharacterGroundState GroundState = EJPRPhysicsCharacterGroundState::OnGround;
};

USTRUCT()
struct RECALLPHYSICSMODULE_API FJPRPhysicsCharacterShapeConstSharedFragment :
	public FMassConstSharedFragment
{
	GENERATED_BODY()
		
	UPROPERTY(VisibleAnywhere)
	FJPRPhysicsCharacter Shape;
	
	UPROPERTY(VisibleAnywhere)
	FJPRPhysicsBodyParameters Params;
};

USTRUCT()
struct RECALLPHYSICSMODULE_API FJPRPhysicsCharacterVirtualShapeConstSharedFragment :
	public FMassConstSharedFragment
{
	GENERATED_BODY()
		
	UPROPERTY(VisibleAnywhere)
	FJPRPhysicsCharacterVirtual Shape;
	
	UPROPERTY(VisibleAnywhere)
	FJPRPhysicsBodyParameters Params;
};
