// Copyright (C) 2024 Van de Walle Bastien
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "Mass/EntityElementTypes.h"
#include "Physics/RecallPhysicsTypes.h"
#include "Physics/Common/JPRPhysicsCommonShapeTypes.h"

#include "RecallPhysicsColliderFragments.generated.h"

USTRUCT()
struct RECALLPHYSICSMODULE_API FJPRPhysicsBoxShapeFragment : public FMassConstSharedFragment
{
	GENERATED_BODY()
		
	UPROPERTY(VisibleAnywhere)
	FJPRPhysicsBoxShape Shape;
	
	UPROPERTY(VisibleAnywhere)
	FJPRPhysicsBodyParameters Params;
};

USTRUCT()
struct RECALLPHYSICSMODULE_API FJPRPhysicsSphereShapeFragment : public FMassConstSharedFragment
{
	GENERATED_BODY()
		
	UPROPERTY(VisibleAnywhere)
	FJPRPhysicsSphereShape Shape;
	
	UPROPERTY(VisibleAnywhere)
	FJPRPhysicsBodyParameters Params;
};

USTRUCT()
struct RECALLPHYSICSMODULE_API FRecallPhysicsCapsuleFragment : public FMassConstSharedFragment
{
	GENERATED_BODY()
		
	UPROPERTY(VisibleAnywhere)
	FJPRPhysicsCapsuleShape Shape;
	
	UPROPERTY(VisibleAnywhere)
	FJPRPhysicsBodyParameters Params;
};

USTRUCT()
struct RECALLPHYSICSMODULE_API FRecallPhysicsMeshFragment : public FMassFragment
{
	GENERATED_BODY()
		
	UPROPERTY(VisibleAnywhere)
	FVector Scale = FVector::OneVector;
};

USTRUCT()
struct RECALLPHYSICSMODULE_API FRecallPhysicsMeshConstSharedFragment : public FMassConstSharedFragment
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere)
	TArray<FVector3f> Vertices;

	UPROPERTY(VisibleAnywhere)
	TArray<int32> Triangles;
	
	UPROPERTY(VisibleAnywhere)
	FJPRPhysicsBodyParameters Params;

	UPROPERTY(VisibleAnywhere)
	bool bConvexHull = true;
	
	UPROPERTY(VisibleAnywhere)
	FJPRPhysicsMeshShapeSettings MeshShapeSettings;
};

USTRUCT()
struct RECALLPHYSICSMODULE_API FRecallPhysicsHeightFieldConstSharedFragment : public FMassConstSharedFragment
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere)
	FJPRPhysicsHeightFieldShape Shape;
	
	UPROPERTY(VisibleAnywhere)
	FVector Location = FVector::ZeroVector;
	
	UPROPERTY(VisibleAnywhere)
	FQuat Rotation = FQuat::Identity;
	
	UPROPERTY(VisibleAnywhere)
	FJPRPhysicsBodyParameters Params;
};
