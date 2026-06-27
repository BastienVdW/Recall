// Copyright (C) 2024 Van de Walle Bastien
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0


#include "RecallPhysicsObjects.h"

#include "DrawDebugHelpers.h"
#include "RecallPhysicsTypes.h"
#include "Physics/JPRPhysicsMath.h"
#include "Utility/Math/RecallMathUtils.h"

#if WITH_JOLT_PHYSICS
// The Jolt headers don't include Jolt.h. Always include Jolt.h before including any other Jolt header.
// You can use Jolt.h in your precompiled header to speed up compilation.
#include <Jolt/Jolt.h>

// Jolt includes
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Character/Character.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>

// STL includes
#include <iostream>
#include <cstdarg>
#include <thread>

// Disable common warnings triggered by Jolt, you can use JPH_SUPPRESS_WARNING_PUSH / JPH_SUPPRESS_WARNING_POP to store and restore the warning state
JPH_SUPPRESS_WARNINGS

// All Jolt symbols are in the JPH namespace
using namespace JPH;

// If you want your code to compile using single or double precision write 0.0_r to get a Real value that compiles to double or float depending if JPH_DOUBLE_PRECISION is set or not.
using namespace JPH::literals;

// We're also using STL classes in this example
using namespace std;
#endif // WITH_JOLT_PHYSICS

DEFINE_LOG_CATEGORY(LogRecallPhysicsObject);

bool FRecallPhysicsBody::IsEnabled() const { return FJPRPhysicsBody::IsEnabled(); }
bool FRecallPhysicsBody::DoesTriggerHitEvents() const { return FJPRPhysicsBody::DoesTriggerHitEvents(); }
uint32 FRecallPhysicsBody::GetBodyID() const { return FJPRPhysicsBody::GetBodyID(); }
void FRecallPhysicsBody::SetActive(bool bActive) { FJPRPhysicsBody::SetActive(bActive); }
void FRecallPhysicsBody::Activate() { FJPRPhysicsBody::Activate(); }
void FRecallPhysicsBody::Desactivate() { FJPRPhysicsBody::Desactivate(); }
void FRecallPhysicsBody::ReleasePhysicsObject() { FJPRPhysicsBody::ReleasePhysicsObject(); }
void FRecallPhysicsBody::SetPosition(const FVector& Position) const { FJPRPhysicsBody::SetPosition(Position); }
void FRecallPhysicsBody::SetRotation(const FQuat& Rotation) const { FJPRPhysicsBody::SetRotation(Rotation); }
void FRecallPhysicsBody::GetPositionAndRotation(FVector& OutPosition, FQuat& OutRotation) const
{
	FJPRPhysicsBody::GetPositionAndRotation(OutPosition, OutRotation);
}
void FRecallPhysicsBody::SetPositionAndRotation(const FVector& Position, const FQuat& Rotation) const
{
	FJPRPhysicsBody::SetPositionAndRotation(Position, Rotation);
}
void FRecallPhysicsBody::GetPosition(FVector& OutPosition) const { FJPRPhysicsBody::GetPosition(OutPosition); }
FTransform FRecallPhysicsBody::GetTransform() const { return FJPRPhysicsBody::GetTransform(); }
void FRecallPhysicsBody::SetRotation(const FRotator& Rotation) const { FJPRPhysicsBody::SetRotation(Rotation); }
void FRecallPhysicsBody::GetRotation(FQuat& OutRotation) const { FJPRPhysicsBody::GetRotation(OutRotation); }
FVector FRecallPhysicsBody::GetForwardVector() const { return FJPRPhysicsBody::GetForwardVector(); }
FVector FRecallPhysicsBody::GetRightVector() const { return FJPRPhysicsBody::GetRightVector(); }
bool FRecallPhysicsBody::CollideShape(const FVector& Position, uint32& OutContactBodyID, FVector& OutContactPosition,
	FVector& OutContactNormal) const
{
	return FJPRPhysicsBody::CollideShape(Position, OutContactBodyID, OutContactPosition, OutContactNormal);
}
bool FRecallPhysicsBody::ShapeCast(const FVector& Position, const FVector& Direction, float Distance,
	uint32& OutContactBodyID, FVector& OutContactPosition, FVector& OutContactNormal) const
{
	return FJPRPhysicsBody::ShapeCast(Position, Direction, Distance, OutContactBodyID, OutContactPosition, OutContactNormal);
}
float FRecallPhysicsBody::GetMass() const { return FJPRPhysicsBody::GetMass(); }
void FRecallPhysicsBody::SetWorldContextObject(UObject* Object) { FJPRPhysicsBody::SetWorldContextObject(Object); }
UObject* FRecallPhysicsBody::GetWorldContextObject() const { return FJPRPhysicsBody::GetWorldContextObject(); }

#if WITH_JOLT_PHYSICS
void FRecallPhysicsBody::SetupBodyCreationSettings(BodyCreationSettings& body_creation_settings, const FRecallPhysicsBodyParameters& Params)
{
	body_creation_settings.mGravityFactor = Params.GravityFactor;
	body_creation_settings.mMotionQuality = static_cast<EMotionQuality>(Params.MotionQuality);
	body_creation_settings.mEnhancedInternalEdgeRemoval = Params.EnhancedInternalEdgeRemoval;
	body_creation_settings.mFriction = Params.Friction;
	body_creation_settings.mAllowDynamicOrKinematic = Params.bAllowDynamicOrKinematic;
	body_creation_settings.mIsSensor = Params.bIsSensor;
	body_creation_settings.mRestitution = Params.Restitution;
	body_creation_settings.mMaxAngularVelocity = DegreesToRadians(Params.MaxAngularVelocityDegrees);
	body_creation_settings.mAllowSleeping = Params.bAllowSleeping;
	body_creation_settings.mAllowedDOFs = static_cast<EAllowedDOFs>(Params.AllowedDOFs);
	body_creation_settings.mOverrideMassProperties = static_cast<EOverrideMassProperties>(Params.OverrideMassProperties);
	body_creation_settings.mInertiaMultiplier = Params.InertiaMultiplier;
	body_creation_settings.mMassPropertiesOverride.mMass = Params.MassPropertiesOverride.Mass;
	body_creation_settings.mMassPropertiesOverride.mInertia = Mat44::sScale(Params.MassPropertiesOverride.Inertia);
	body_creation_settings.mNumVelocityStepsOverride = Params.NumVelocityStepsOverride;
	body_creation_settings.mNumPositionStepsOverride = Params.NumPositionStepsOverride;
}
#endif // WITH_JOLT_PHYSICS

void FRecallPhysicsBody::AddLinearVelocity(const FVector& LinearVelocity)
{
#if WITH_JOLT_PHYSICS
	if (body_id.IsValid())
	{
		const FVector LinearVelocityPerSecond = Recall::Math::Utils::UnitsPerFrameToPerSecond(LinearVelocity);
		const FVector PhysicsLinearVelocity = UnrealToJoltPhysics(LinearVelocityPerSecond);

		Vec3 LinearVel;
		LinearVel.SetX(PhysicsLinearVelocity.X);
		LinearVel.SetY(PhysicsLinearVelocity.Y);
		LinearVel.SetZ(PhysicsLinearVelocity.Z);

		GetBodyInterface().AddLinearVelocity(*body_id.Get(), LinearVel);
	}
#endif // WITH_JOLT_PHYSICS
}

void FRecallPhysicsBody::AddLinearAndAngularVelocity(const FVector& LinearVelocity, const FVector& AngularVelocity)
{
#if WITH_JOLT_PHYSICS
	if (body_id.IsValid())
	{
		const FVector LinearVelocityPerSecond = Recall::Math::Utils::UnitsPerFrameToPerSecond(LinearVelocity);
		const FVector PhysicsLinearVelocity = UnrealToJoltPhysics(LinearVelocityPerSecond);
		const Vec3 LinearVel(PhysicsLinearVelocity.X, PhysicsLinearVelocity.Y, PhysicsLinearVelocity.Z);

		const FVector AngularVelocityPerSecond = Recall::Math::Utils::UnitsPerFrameToPerSecond(AngularVelocity);
		const FVector PhysicsAngularVelocity = FMath::DegreesToRadians(UnrealToJoltPhysics(AngularVelocityPerSecond));
		const Vec3 AngularVel(PhysicsAngularVelocity.X, PhysicsAngularVelocity.Y, PhysicsAngularVelocity.Z);

		GetBodyInterface().AddLinearAndAngularVelocity(*body_id.Get(), LinearVel, AngularVel);
	}
#endif // WITH_JOLT_PHYSICS
}

void FRecallPhysicsBody::SetLinearAndAngularVelocity(const FVector& LinearVelocity, const FVector& AngularVelocity)
{
#if WITH_JOLT_PHYSICS
	if (body_id.IsValid())
	{
		const FVector LinearVelocityPerSecond = Recall::Math::Utils::UnitsPerFrameToPerSecond(LinearVelocity);
		const FVector PhysicsLinearVelocity = UnrealToJoltPhysics(LinearVelocityPerSecond);
		const Vec3 LinearVel(PhysicsLinearVelocity.X, PhysicsLinearVelocity.Y, PhysicsLinearVelocity.Z);

		const FVector AngularVelocityPerSecond = Recall::Math::Utils::UnitsPerFrameToPerSecond(AngularVelocity);
		const FVector PhysicsAngularVelocity = FMath::DegreesToRadians(UnrealToJoltPhysics(AngularVelocityPerSecond));
		const Vec3 AngularVel(PhysicsAngularVelocity.X, PhysicsAngularVelocity.Y, PhysicsAngularVelocity.Z);

		GetBodyInterface().SetLinearAndAngularVelocity(*body_id.Get(), LinearVel, AngularVel);
	}
#endif // WITH_JOLT_PHYSICS
}

void FRecallPhysicsBody::AddImpulse(const FVector& ForceNewton, bool bIgnoreMass)
{
	const double Mass = bIgnoreMass ? 1.0f : GetMass();
	const FVector Velocity = Recall::Math::Utils::UnitsPerSecondToPerFrame(ForceNewton) * 100.0 / Mass;

	AddLinearVelocity(Velocity);
}

void FRecallPhysicsBody::SetLinearVelocityFromImpulse(const FVector& ForceNewton, bool bIgnoreMass)
{
	const double Mass = bIgnoreMass ? 1.0f : GetMass();
	const FVector Velocity = Recall::Math::Utils::UnitsPerSecondToPerFrame(ForceNewton) * 100.0 / Mass;

	SetLinearVelocity(Velocity);
}

void FRecallPhysicsBody::SetLinearVelocity2DFromImpulse(const FVector& ForceNewton, bool bIgnoreMass)
{
	const double Mass = bIgnoreMass ? 1.0f : GetMass();
	const FVector Velocity = Recall::Math::Utils::UnitsPerSecondToPerFrame(ForceNewton) * 100.0 / Mass;

	SetLinearVelocity2D(FVector2D(Velocity.X, Velocity.Y));
}

void FRecallPhysicsBody::SetLinearVelocity2D(const FVector2D& LinearVelocity)
{
	FVector WorldVelocity = GetLinearVelocity();
	WorldVelocity.X = LinearVelocity.X;
	WorldVelocity.Y = LinearVelocity.Y;

	SetLinearVelocity(WorldVelocity);
}

void FRecallPhysicsBody::SetLinearZVelocity(float ZVelocity)
{
	FVector WorldVelocity = GetLinearVelocity();
	WorldVelocity.Z = ZVelocity;

	SetLinearVelocity(WorldVelocity);
}

void FRecallPhysicsBody::SetLinearVelocity(const FVector& LinearVelocity)
{
#if WITH_JOLT_PHYSICS
	if (body_id.IsValid())
	{
		const FVector LinearVelocityPerSecond = Recall::Math::Utils::UnitsPerFrameToPerSecond(LinearVelocity);
		const FVector PhysicsVelocity = UnrealToJoltPhysics(LinearVelocityPerSecond);
		const Vec3 Vel(PhysicsVelocity.X, PhysicsVelocity.Y, PhysicsVelocity.Z);

		GetBodyInterface().SetLinearVelocity(*body_id.Get(), Vel);
	}
#endif // WITH_JOLT_PHYSICS
}

FVector FRecallPhysicsBody::GetLinearVelocity() const
{
#if WITH_JOLT_PHYSICS
	if (body_id.IsValid())
	{
		const Vec3 PhysicsVelocity = GetBodyInterface().GetLinearVelocity(*body_id.Get());
		const FVector WorldVelocity = JoltPhysicsToUnreal(FVector(PhysicsVelocity.GetX(), PhysicsVelocity.GetY(), PhysicsVelocity.GetZ()));

		return Recall::Math::Utils::UnitsPerSecondToPerFrame(WorldVelocity);
	}
#endif // WITH_JOLT_PHYSICS
	return FVector::ZeroVector;
}

FVector2D FRecallPhysicsBody::GetLinearVelocity2D() const
{
	const FVector Velocity = GetLinearVelocity();
	return static_cast<FVector2D>(Velocity);
}

void FRecallPhysicsBody::SetAngularVelocity(const FVector& AngularVelocity)
{
#if WITH_JOLT_PHYSICS
	if (body_id.IsValid())
	{
		const FVector AngularVelocityPerSecond = Recall::Math::Utils::UnitsPerFrameToPerSecond(AngularVelocity);
		const FVector PhysicsVelocity = FMath::DegreesToRadians(UnrealToJoltPhysics(AngularVelocityPerSecond));
		const Vec3 Vel(PhysicsVelocity.X, PhysicsVelocity.Y, PhysicsVelocity.Z);

		GetBodyInterface().SetAngularVelocity(*body_id.Get(), Vel);
	}
#endif // WITH_JOLT_PHYSICS
}

FVector FRecallPhysicsBody::GetAngularVelocity() const
{
#if WITH_JOLT_PHYSICS
	if (body_id.IsValid())
	{
		const Vec3 PhysicsAngularVelocity = GetBodyInterface().GetAngularVelocity(*body_id.Get());
		const FVector WorldAngularVelocity = FMath::RadiansToDegrees(JoltPhysicsToUnreal(FVector(PhysicsAngularVelocity.GetX(), PhysicsAngularVelocity.GetY(), PhysicsAngularVelocity.GetZ())));

		return Recall::Math::Utils::UnitsPerSecondToPerFrame(WorldAngularVelocity);
	}
#endif // WITH_JOLT_PHYSICS
	return FVector::ZeroVector;
}

FVector FRecallPhysicsBody::GetMovementForwardVector() const
{
	const FVector WorldVelocity = GetLinearVelocity();
	if (!WorldVelocity.IsNearlyZero())
	{
		return WorldVelocity.GetSafeNormal();
	}
	
	return GetForwardVector();
}

#if WITH_JOLT_PHYSICS
void FRecallPhysicsBody::SaveVector(JPH::StateRecorder& State, const FVector& Vec)
{
	State.Write(DVec3(Vec.X, Vec.Y, Vec.Z));
}

void FRecallPhysicsBody::RestoreVector(JPH::StateRecorder& State, FVector& Vec)
{
	DVec3 Src = DVec3::sZero();
	State.Read(Src);
	Vec = FVector(Src.GetX(), Src.GetY(), Src.GetZ());
}

#endif // WITH_JOLT_PHYSICS

#if UE_BUILD_DEBUG || UE_BUILD_DEVELOPMENT
void FRecallPhysicsBody::DumpObject() const
{
#if WITH_JOLT_PHYSICS
	if (body_id.IsValid())
	{
		RVec3 PhysicsPosition = RVec3::sZero();
		Quat PhysicsRotation = Quat::sIdentity();
		GetBodyInterface().GetPositionAndRotation(*body_id.Get(), PhysicsPosition, PhysicsRotation);

		const FVector WorldPosition = JoltPhysicsToUnreal(FVector(PhysicsPosition.GetX(), PhysicsPosition.GetY(), PhysicsPosition.GetZ()));
		const FQuat WorldRotation = JoltPhysicsToUnreal(FQuat(PhysicsRotation.GetX(), PhysicsRotation.GetY(), PhysicsRotation.GetZ(), PhysicsRotation.GetW()));

		const Vec3 PhysicsVelocity = GetBodyInterface().GetLinearVelocity(*body_id.Get());
		const FVector WorldVelocity = JoltPhysicsToUnreal(FVector(PhysicsVelocity.GetX(), PhysicsVelocity.GetY(), PhysicsVelocity.GetZ()));

		FString DumpString;
		DumpString += FString::Printf(TEXT("[BodyID: %d]\n"), GetBodyID());

		DumpString += FString::Printf(TEXT("PhysicsVelocity: %s\n"), *FString::Printf(TEXT("X=%3.3f Y=%3.3f Z=%3.3f"), PhysicsVelocity.GetX(), PhysicsVelocity.GetY(), PhysicsVelocity.GetZ()));
		DumpString += FString::Printf(TEXT("WorldVelocity: %s\n"), *WorldVelocity.ToString());

		DumpString += FString::Printf(TEXT("PhysicsPosition: %s\n"), *FString::Printf(TEXT("X=%3.3f Y=%3.3f Z=%3.3f"), PhysicsPosition.GetX(), PhysicsPosition.GetY(), PhysicsPosition.GetZ()));
		DumpString += FString::Printf(TEXT("WorldPosition: %s\n"), *WorldPosition.ToString());

		UE_LOG(LogRecallPhysicsObject, Log, TEXT("%s"), *DumpString);
	}
#endif // WITH_JOLT_PHYSICS
}
#endif // UE_BUILD_DEBUG || UE_BUILD_DEVELOPMENT
// FRecallPhysicsBody End
