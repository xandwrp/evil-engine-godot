/**************************************************************************/
/*  jolt_character_3d.cpp                                                 */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "jolt_character_3d.h"

#include "../jolt_physics_server_3d.h"
#include "../misc/jolt_type_conversions.h"
#include "../spaces/jolt_broad_phase_layer.h"
#include "../spaces/jolt_space_3d.h"
#include "jolt_object_3d.h"
#include "jolt_shaped_object_3d.h"

#include "core/object/class_db.h"

#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/StateRecorderImpl.h>

JoltSpace3D *JoltCharacter3D::_get_space() const {
	JoltPhysicsServer3D *server = JoltPhysicsServer3D::get_singleton();
	ERR_FAIL_NULL_V_MSG(server, nullptr, "A Jolt character needs the Jolt physics server.");
	return server->get_space(space_rid);
}

JPH::ShapeRefC JoltCharacter3D::_make_box(const Vector3 &p_size, const Vector3 &p_center, float p_convex_radius) const {
	const JPH::Vec3 half = JPH::Vec3::sMax(to_jolt(p_size * 0.5) - JPH::Vec3::sReplicate(character_padding), JPH::Vec3::sReplicate(0.001f));
	const float radius = MIN(MAX(p_convex_radius, 0.0f), half.ReduceMin());
	const JPH::Vec3 offset = to_jolt(p_center) - JPH::Vec3(0, character_padding, 0);

	JPH::RotatedTranslatedShapeSettings settings(offset, JPH::Quat::sIdentity(), new JPH::BoxShapeSettings(half, radius));
	const JPH::ShapeSettings::ShapeResult result = settings.Create();
	ERR_FAIL_COND_V_MSG(result.HasError(), nullptr, vformat("A Jolt character's box couldn't be made: %s", to_godot(result.GetError())));
	return result.Get();
}

bool JoltCharacter3D::_blocked_by_steep(const JPH::Vec3 &p_direction) const {
	const float cos_max_slope = JPH::Cos(max_slope_angle);
	for (const JPH::CharacterContact &contact : character->GetActiveContacts()) {
		if (contact.mHadCollision && !contact.mWasDiscarded && !contact.mIsSensorB && contact.mContactNormal.Dot(p_direction) < -1.0e-3f && contact.mContactNormal.GetY() < cos_max_slope) {
			return true;
		}
	}
	return false;
}

bool JoltCharacter3D::_step_up(JoltSpace3D &p_space, const JPH::Vec3 &p_forward, float p_delta, float p_step_up, const JPH::Vec3 &p_gravity) {
	const JPH::PhysicsSystem &system = p_space.get_physics_system();
	const JPH::DefaultBroadPhaseLayerFilter broad_phase_filter = system.GetDefaultBroadPhaseLayerFilter(object_layer);
	const JPH::DefaultObjectLayerFilter object_layer_filter = system.GetDefaultLayerFilter(object_layer);
	const JPH::BodyFilter body_filter;
	const JPH::ShapeFilter shape_filter;

	JPH::StateRecorderImpl saved;
	character->SaveState(saved);
	const JPH::RVec3 start = character->GetPosition();
	const JPH::Vec3 velocity = character->GetLinearVelocity();

	const JPH::Vec3 up(0, p_step_up, 0);
	JPH::ShapeCastSettings settings;
	settings.mBackFaceModeTriangles = JPH::EBackFaceMode::CollideWithBackFaces;
	settings.mUseShrunkenShapeAndConvexRadius = true;
	settings.mReturnDeepestPoint = false;
	const JPH::RMat44 transform = character->GetCenterOfMassTransform();
	JPH::RShapeCast cast(character->GetShape(), JPH::Vec3::sOne(), transform, up);
	JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> ceiling;
	system.GetNarrowPhaseQuery().CastShape(cast, settings, transform.GetTranslation(), ceiling, broad_phase_filter, object_layer_filter, body_filter, shape_filter);
	const float rise = ceiling.HadHit() ? MAX(0.0f, ceiling.mHit.mFraction * p_step_up - character_padding) : p_step_up;
	if (rise <= 1.0e-3f) {
		return false;
	}

	character->SetPosition(start + JPH::Vec3(0, rise, 0));
	character->SetLinearVelocity(p_forward / p_delta);
	character->Update(p_delta, p_gravity, broad_phase_filter, object_layer_filter, body_filter, shape_filter, p_space.get_temp_allocator());
	const JPH::Vec3 moved = JPH::Vec3(character->GetPosition() - start);
	const bool landed = character->StickToFloor(JPH::Vec3(0, -(rise + walk_stairs_step_down_extra), 0), broad_phase_filter, object_layer_filter, body_filter, shape_filter, p_space.get_temp_allocator());
	const JPH::Vec3 stepped = JPH::Vec3(character->GetPosition() - start);
	const float forward = p_forward.Length();
	if (landed && character->GetGroundState() == JPH::CharacterBase::EGroundState::OnGround && stepped.GetY() > 1.0e-4f && moved.Dot(p_forward / forward) > 0.05f * forward) {
		character->SetLinearVelocity(velocity);
		return true;
	}

	saved.Rewind();
	character->RestoreState(saved);
	character->SetLinearVelocity(velocity);
	return false;
}

JoltCharacter3D::~JoltCharacter3D() {
	destroy();
}

bool JoltCharacter3D::create(RID p_space, const Vector3 &p_position, const Vector3 &p_size, const Vector3 &p_center, float p_convex_radius) {
	ERR_FAIL_COND_V_MSG(character != nullptr, false, "A Jolt character is created once.");
	space_rid = p_space;
	JoltSpace3D *space = _get_space();
	ERR_FAIL_NULL_V_MSG(space, false, "A Jolt character is created in a Jolt space.");

	const JPH::ShapeRefC shape = _make_box(p_size, p_center, p_convex_radius);
	ERR_FAIL_COND_V(shape == nullptr, false);

	JPH::CharacterVirtualSettings settings;
	settings.mShape = shape;
	settings.mUp = JPH::Vec3::sAxisY();
	settings.mMaxSlopeAngle = max_slope_angle;
	settings.mMass = mass;
	settings.mMaxStrength = max_strength;
	settings.mCharacterPadding = character_padding;
	settings.mPredictiveContactDistance = predictive_contact_distance;
	settings.mPenetrationRecoverySpeed = penetration_recovery_speed;
	settings.mEnhancedInternalEdgeRemoval = enhanced_internal_edge_removal;
	settings.mBackFaceMode = JPH::EBackFaceMode::CollideWithBackFaces;

	object_layer = space->map_to_object_layer(JoltBroadPhaseLayer::BODY_DYNAMIC, collision_layer, collision_mask);
	character = new JPH::CharacterVirtual(&settings, to_jolt_r(p_position), JPH::Quat::sIdentity(), 0, &space->get_physics_system());
	slid_velocity = Vector3();
	refresh_contacts();
	return true;
}

void JoltCharacter3D::destroy() {
	character = nullptr;
	space_rid = RID();
	slid_velocity = Vector3();
}

bool JoltCharacter3D::set_box(const Vector3 &p_size, const Vector3 &p_center, float p_convex_radius, float p_max_penetration) {
	ERR_FAIL_NULL_V(character, false);
	JoltSpace3D *space = _get_space();
	ERR_FAIL_NULL_V(space, false);
	const JPH::ShapeRefC shape = _make_box(p_size, p_center, p_convex_radius);
	ERR_FAIL_COND_V(shape == nullptr, false);

	space->flush_pending_objects();
	const JPH::PhysicsSystem &system = space->get_physics_system();
	const JPH::DefaultBroadPhaseLayerFilter broad_phase_filter = system.GetDefaultBroadPhaseLayerFilter(object_layer);
	const JPH::DefaultObjectLayerFilter object_layer_filter = system.GetDefaultLayerFilter(object_layer);
	return character->SetShape(shape, p_max_penetration, broad_phase_filter, object_layer_filter, JPH::BodyFilter(), JPH::ShapeFilter(), space->get_temp_allocator());
}

Vector3 JoltCharacter3D::get_position() const {
	ERR_FAIL_NULL_V(character, Vector3());
	return to_godot(character->GetPosition());
}

void JoltCharacter3D::set_position(const Vector3 &p_position) {
	ERR_FAIL_NULL(character);
	character->SetPosition(to_jolt_r(p_position));
}

Vector3 JoltCharacter3D::get_linear_velocity() const {
	ERR_FAIL_NULL_V(character, Vector3());
	return to_godot(character->GetLinearVelocity());
}

void JoltCharacter3D::set_linear_velocity(const Vector3 &p_velocity) {
	ERR_FAIL_NULL(character);
	character->SetLinearVelocity(to_jolt(p_velocity));
}

void JoltCharacter3D::move(float p_delta, const Vector3 &p_gravity, float p_step_up, float p_stick_down) {
	ERR_FAIL_NULL(character);
	JoltSpace3D *space = _get_space();
	ERR_FAIL_NULL(space);
	ERR_FAIL_COND_MSG(space->is_stepping(), "A Jolt character can't move while its space is stepping.");

	space->flush_pending_objects();
	const JPH::PhysicsSystem &system = space->get_physics_system();
	const JPH::DefaultBroadPhaseLayerFilter broad_phase_filter = system.GetDefaultBroadPhaseLayerFilter(object_layer);
	const JPH::DefaultObjectLayerFilter object_layer_filter = system.GetDefaultLayerFilter(object_layer);

	JPH::CharacterVirtual::ExtendedUpdateSettings settings;
	settings.mStickToFloorStepDown = JPH::Vec3(0, -MAX(p_stick_down, 0.0f), 0);
	settings.mWalkStairsStepUp = JPH::Vec3(0, MAX(p_step_up, 0.0f), 0);
	settings.mWalkStairsMinStepForward = walk_stairs_min_step_forward;
	settings.mWalkStairsStepForwardTest = walk_stairs_step_forward_test;
	settings.mWalkStairsCosAngleForwardContact = walk_stairs_cos_angle_forward_contact;
	settings.mWalkStairsStepDownExtra = JPH::Vec3(0, -MAX(walk_stairs_step_down_extra, 0.0f), 0);

	Vector3 velocity = to_godot(character->GetLinearVelocity());
	const bool supported = character->IsSupported();
	const JPH::RVec3 start = character->GetPosition();
	character->ExtendedUpdate(p_delta, to_jolt(p_gravity), settings, broad_phase_filter, object_layer_filter, JPH::BodyFilter(), JPH::ShapeFilter(), space->get_temp_allocator());

	const JPH::Vec3 desired = JPH::Vec3((float)velocity.x, 0, (float)velocity.z) * p_delta;
	const float desired_length = desired.Length();
	if (p_step_up > 0 && supported && desired_length > 0) {
		const JPH::Vec3 direction = desired / desired_length;
		const float achieved = MAX(0.0f, JPH::Vec3(character->GetPosition() - start).Dot(direction));
		if (achieved + 1.0e-4f < desired_length && _blocked_by_steep(direction)) {
			_step_up(*space, direction * MAX(walk_stairs_min_step_forward, desired_length - achieved), p_delta, p_step_up, to_jolt(p_gravity));
		}
	}

	for (int pass = 0; pass < 2; pass++) {
		for (const JPH::CharacterContact &contact : character->GetActiveContacts()) {
			if (!contact.mHadCollision || contact.mWasDiscarded || contact.mIsSensorB) {
				continue;
			}
			const Vector3 normal = to_godot(contact.mContactNormal);
			const real_t into = normal.dot(velocity - to_godot(contact.mLinearVelocity));
			if (into < 0) {
				velocity -= normal * into;
			}
		}
	}
	slid_velocity = velocity;
}

void JoltCharacter3D::refresh_contacts() {
	ERR_FAIL_NULL(character);
	JoltSpace3D *space = _get_space();
	ERR_FAIL_NULL(space);
	space->flush_pending_objects();
	const JPH::PhysicsSystem &system = space->get_physics_system();
	const JPH::DefaultBroadPhaseLayerFilter broad_phase_filter = system.GetDefaultBroadPhaseLayerFilter(object_layer);
	const JPH::DefaultObjectLayerFilter object_layer_filter = system.GetDefaultLayerFilter(object_layer);
	character->RefreshContacts(broad_phase_filter, object_layer_filter, JPH::BodyFilter(), JPH::ShapeFilter(), space->get_temp_allocator());
}

bool JoltCharacter3D::can_walk_stairs(const Vector3 &p_velocity) const {
	ERR_FAIL_NULL_V(character, false);
	return character->CanWalkStairs(to_jolt(p_velocity));
}

bool JoltCharacter3D::walk_stairs(float p_delta, float p_step_up, const Vector3 &p_step_forward, const Vector3 &p_step_forward_test, float p_step_down_extra) {
	ERR_FAIL_NULL_V(character, false);
	JoltSpace3D *space = _get_space();
	ERR_FAIL_NULL_V(space, false);
	space->flush_pending_objects();
	const JPH::PhysicsSystem &system = space->get_physics_system();
	const JPH::DefaultBroadPhaseLayerFilter broad_phase_filter = system.GetDefaultBroadPhaseLayerFilter(object_layer);
	const JPH::DefaultObjectLayerFilter object_layer_filter = system.GetDefaultLayerFilter(object_layer);
	return character->WalkStairs(p_delta, JPH::Vec3(0, MAX(p_step_up, 0.0f), 0), to_jolt(p_step_forward), to_jolt(p_step_forward_test), JPH::Vec3(0, -MAX(p_step_down_extra, 0.0f), 0), broad_phase_filter, object_layer_filter, JPH::BodyFilter(), JPH::ShapeFilter(), space->get_temp_allocator());
}

JoltCharacter3D::GroundState JoltCharacter3D::get_ground_state() const {
	ERR_FAIL_NULL_V(character, GROUND_STATE_IN_AIR);
	return (GroundState)character->GetGroundState();
}

bool JoltCharacter3D::is_supported() const {
	ERR_FAIL_NULL_V(character, false);
	return character->IsSupported();
}

Vector3 JoltCharacter3D::get_ground_normal() const {
	ERR_FAIL_NULL_V(character, Vector3());
	return to_godot(character->GetGroundNormal());
}

Vector3 JoltCharacter3D::get_ground_position() const {
	ERR_FAIL_NULL_V(character, Vector3());
	return to_godot(character->GetGroundPosition());
}

Vector3 JoltCharacter3D::get_ground_velocity() const {
	ERR_FAIL_NULL_V(character, Vector3());
	return to_godot(character->GetGroundVelocity());
}

RID JoltCharacter3D::get_ground_body() const {
	ERR_FAIL_NULL_V(character, RID());
	const JoltSpace3D *space = _get_space();
	ERR_FAIL_NULL_V(space, RID());
	const JoltObject3D *object = character->GetGroundBodyID().IsInvalid() ? nullptr : space->try_get_object(character->GetGroundBodyID());
	return object != nullptr ? object->get_rid() : RID();
}

int JoltCharacter3D::get_ground_shape() const {
	ERR_FAIL_NULL_V(character, -1);
	const JoltSpace3D *space = _get_space();
	ERR_FAIL_NULL_V(space, -1);
	const JoltShapedObject3D *object = character->GetGroundBodyID().IsInvalid() ? nullptr : space->try_get_shaped(character->GetGroundBodyID());
	return object != nullptr ? object->find_shape_index(character->GetGroundSubShapeID()) : -1;
}

TypedArray<Dictionary> JoltCharacter3D::get_contacts() const {
	TypedArray<Dictionary> contacts;
	ERR_FAIL_NULL_V(character, contacts);
	const JoltSpace3D *space = _get_space();
	ERR_FAIL_NULL_V(space, contacts);
	for (const JPH::CharacterContact &contact : character->GetActiveContacts()) {
		const JoltShapedObject3D *object = contact.mBodyB.IsInvalid() ? nullptr : space->try_get_shaped(contact.mBodyB);
		Dictionary entry;
		entry["position"] = to_godot(contact.mPosition);
		entry["normal"] = to_godot(contact.mContactNormal);
		entry["surface_normal"] = to_godot(contact.mSurfaceNormal);
		entry["velocity"] = to_godot(contact.mLinearVelocity);
		entry["distance"] = contact.mDistance;
		entry["fraction"] = contact.mFraction;
		entry["had_collision"] = contact.mHadCollision;
		entry["discarded"] = contact.mWasDiscarded;
		entry["sensor"] = contact.mIsSensorB;
		entry["body"] = object != nullptr ? object->get_rid() : RID();
		entry["shape"] = object != nullptr ? object->find_shape_index(contact.mSubShapeIDB) : -1;
		entry["face"] = object != nullptr ? object->find_face_index(contact.mSubShapeIDB) : -1;
		contacts.push_back(entry);
	}
	return contacts;
}

int JoltCharacter3D::get_ground_face() const {
	ERR_FAIL_NULL_V(character, -1);
	const JoltSpace3D *space = _get_space();
	ERR_FAIL_NULL_V(space, -1);
	const JoltShapedObject3D *object = character->GetGroundBodyID().IsInvalid() ? nullptr : space->try_get_shaped(character->GetGroundBodyID());
	return object != nullptr ? object->find_face_index(character->GetGroundSubShapeID()) : -1;
}

PackedByteArray JoltCharacter3D::save_state() const {
	ERR_FAIL_NULL_V(character, PackedByteArray());
	JPH::StateRecorderImpl recorder;
	character->SaveState(recorder);
	const std::string data = recorder.GetData();
	PackedByteArray state;
	state.resize(data.size());
	memcpy(state.ptrw(), data.data(), data.size());
	return state;
}

bool JoltCharacter3D::restore_state(const PackedByteArray &p_state) {
	ERR_FAIL_NULL_V(character, false);
	JPH::StateRecorderImpl recorder;
	recorder.WriteBytes(p_state.ptr(), p_state.size());
	recorder.Rewind();
	character->RestoreState(recorder);
	return !recorder.IsFailed();
}

void JoltCharacter3D::set_mass(float p_mass) {
	mass = p_mass;
	if (character != nullptr) {
		character->SetMass(p_mass);
	}
}

void JoltCharacter3D::set_max_strength(float p_max_strength) {
	max_strength = p_max_strength;
	if (character != nullptr) {
		character->SetMaxStrength(p_max_strength);
	}
}

void JoltCharacter3D::set_max_slope_angle(float p_angle) {
	max_slope_angle = p_angle;
	if (character != nullptr) {
		character->SetMaxSlopeAngle(p_angle);
	}
}

void JoltCharacter3D::set_character_padding(float p_padding) {
	ERR_FAIL_COND_MSG(character != nullptr, "A Jolt character's padding is set before it's created.");
	character_padding = p_padding;
}

void JoltCharacter3D::set_predictive_contact_distance(float p_distance) {
	ERR_FAIL_COND_MSG(character != nullptr, "A Jolt character's predictive contact distance is set before it's created.");
	predictive_contact_distance = p_distance;
}

void JoltCharacter3D::set_penetration_recovery_speed(float p_speed) {
	penetration_recovery_speed = p_speed;
	if (character != nullptr) {
		character->SetPenetrationRecoverySpeed(p_speed);
	}
}

void JoltCharacter3D::set_enhanced_internal_edge_removal(bool p_enable) {
	enhanced_internal_edge_removal = p_enable;
	if (character != nullptr) {
		character->SetEnhancedInternalEdgeRemoval(p_enable);
	}
}

void JoltCharacter3D::set_collision_layer(uint32_t p_layer) {
	collision_layer = p_layer;
	if (JoltSpace3D *space = character != nullptr ? _get_space() : nullptr) {
		object_layer = space->map_to_object_layer(JoltBroadPhaseLayer::BODY_DYNAMIC, collision_layer, collision_mask);
	}
}

void JoltCharacter3D::set_collision_mask(uint32_t p_mask) {
	collision_mask = p_mask;
	if (JoltSpace3D *space = character != nullptr ? _get_space() : nullptr) {
		object_layer = space->map_to_object_layer(JoltBroadPhaseLayer::BODY_DYNAMIC, collision_layer, collision_mask);
	}
}

void JoltCharacter3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("create", "space", "position", "size", "center", "convex_radius"), &JoltCharacter3D::create);
	ClassDB::bind_method(D_METHOD("destroy"), &JoltCharacter3D::destroy);
	ClassDB::bind_method(D_METHOD("is_created"), &JoltCharacter3D::is_created);
	ClassDB::bind_method(D_METHOD("set_box", "size", "center", "convex_radius", "max_penetration"), &JoltCharacter3D::set_box);

	ClassDB::bind_method(D_METHOD("get_position"), &JoltCharacter3D::get_position);
	ClassDB::bind_method(D_METHOD("set_position", "position"), &JoltCharacter3D::set_position);
	ClassDB::bind_method(D_METHOD("get_linear_velocity"), &JoltCharacter3D::get_linear_velocity);
	ClassDB::bind_method(D_METHOD("set_linear_velocity", "velocity"), &JoltCharacter3D::set_linear_velocity);

	ClassDB::bind_method(D_METHOD("move", "delta", "gravity", "step_up", "stick_down"), &JoltCharacter3D::move);
	ClassDB::bind_method(D_METHOD("refresh_contacts"), &JoltCharacter3D::refresh_contacts);
	ClassDB::bind_method(D_METHOD("get_slid_velocity"), &JoltCharacter3D::get_slid_velocity);
	ClassDB::bind_method(D_METHOD("can_walk_stairs", "velocity"), &JoltCharacter3D::can_walk_stairs);
	ClassDB::bind_method(D_METHOD("walk_stairs", "delta", "step_up", "step_forward", "step_forward_test", "step_down_extra"), &JoltCharacter3D::walk_stairs);

	ClassDB::bind_method(D_METHOD("get_ground_state"), &JoltCharacter3D::get_ground_state);
	ClassDB::bind_method(D_METHOD("is_supported"), &JoltCharacter3D::is_supported);
	ClassDB::bind_method(D_METHOD("get_ground_normal"), &JoltCharacter3D::get_ground_normal);
	ClassDB::bind_method(D_METHOD("get_ground_position"), &JoltCharacter3D::get_ground_position);
	ClassDB::bind_method(D_METHOD("get_ground_velocity"), &JoltCharacter3D::get_ground_velocity);
	ClassDB::bind_method(D_METHOD("get_ground_body"), &JoltCharacter3D::get_ground_body);
	ClassDB::bind_method(D_METHOD("get_ground_shape"), &JoltCharacter3D::get_ground_shape);
	ClassDB::bind_method(D_METHOD("get_ground_face"), &JoltCharacter3D::get_ground_face);

	ClassDB::bind_method(D_METHOD("get_contacts"), &JoltCharacter3D::get_contacts);

	ClassDB::bind_method(D_METHOD("save_state"), &JoltCharacter3D::save_state);
	ClassDB::bind_method(D_METHOD("restore_state", "state"), &JoltCharacter3D::restore_state);

	ClassDB::bind_method(D_METHOD("get_mass"), &JoltCharacter3D::get_mass);
	ClassDB::bind_method(D_METHOD("set_mass", "mass"), &JoltCharacter3D::set_mass);
	ClassDB::bind_method(D_METHOD("get_max_strength"), &JoltCharacter3D::get_max_strength);
	ClassDB::bind_method(D_METHOD("set_max_strength", "max_strength"), &JoltCharacter3D::set_max_strength);
	ClassDB::bind_method(D_METHOD("get_max_slope_angle"), &JoltCharacter3D::get_max_slope_angle);
	ClassDB::bind_method(D_METHOD("set_max_slope_angle", "angle"), &JoltCharacter3D::set_max_slope_angle);
	ClassDB::bind_method(D_METHOD("get_character_padding"), &JoltCharacter3D::get_character_padding);
	ClassDB::bind_method(D_METHOD("set_character_padding", "padding"), &JoltCharacter3D::set_character_padding);
	ClassDB::bind_method(D_METHOD("get_predictive_contact_distance"), &JoltCharacter3D::get_predictive_contact_distance);
	ClassDB::bind_method(D_METHOD("set_predictive_contact_distance", "distance"), &JoltCharacter3D::set_predictive_contact_distance);
	ClassDB::bind_method(D_METHOD("get_penetration_recovery_speed"), &JoltCharacter3D::get_penetration_recovery_speed);
	ClassDB::bind_method(D_METHOD("set_penetration_recovery_speed", "speed"), &JoltCharacter3D::set_penetration_recovery_speed);
	ClassDB::bind_method(D_METHOD("get_enhanced_internal_edge_removal"), &JoltCharacter3D::get_enhanced_internal_edge_removal);
	ClassDB::bind_method(D_METHOD("set_enhanced_internal_edge_removal", "enable"), &JoltCharacter3D::set_enhanced_internal_edge_removal);
	ClassDB::bind_method(D_METHOD("get_walk_stairs_min_step_forward"), &JoltCharacter3D::get_walk_stairs_min_step_forward);
	ClassDB::bind_method(D_METHOD("set_walk_stairs_min_step_forward", "distance"), &JoltCharacter3D::set_walk_stairs_min_step_forward);
	ClassDB::bind_method(D_METHOD("get_walk_stairs_step_forward_test"), &JoltCharacter3D::get_walk_stairs_step_forward_test);
	ClassDB::bind_method(D_METHOD("set_walk_stairs_step_forward_test", "distance"), &JoltCharacter3D::set_walk_stairs_step_forward_test);
	ClassDB::bind_method(D_METHOD("get_walk_stairs_cos_angle_forward_contact"), &JoltCharacter3D::get_walk_stairs_cos_angle_forward_contact);
	ClassDB::bind_method(D_METHOD("set_walk_stairs_cos_angle_forward_contact", "cos_angle"), &JoltCharacter3D::set_walk_stairs_cos_angle_forward_contact);
	ClassDB::bind_method(D_METHOD("get_walk_stairs_step_down_extra"), &JoltCharacter3D::get_walk_stairs_step_down_extra);
	ClassDB::bind_method(D_METHOD("set_walk_stairs_step_down_extra", "distance"), &JoltCharacter3D::set_walk_stairs_step_down_extra);
	ClassDB::bind_method(D_METHOD("get_collision_layer"), &JoltCharacter3D::get_collision_layer);
	ClassDB::bind_method(D_METHOD("set_collision_layer", "layer"), &JoltCharacter3D::set_collision_layer);
	ClassDB::bind_method(D_METHOD("get_collision_mask"), &JoltCharacter3D::get_collision_mask);
	ClassDB::bind_method(D_METHOD("set_collision_mask", "mask"), &JoltCharacter3D::set_collision_mask);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mass"), "set_mass", "get_mass");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_strength"), "set_max_strength", "get_max_strength");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_slope_angle", PROPERTY_HINT_RANGE, "0,90,0.01,radians_as_degrees"), "set_max_slope_angle", "get_max_slope_angle");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "character_padding"), "set_character_padding", "get_character_padding");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "predictive_contact_distance"), "set_predictive_contact_distance", "get_predictive_contact_distance");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "penetration_recovery_speed"), "set_penetration_recovery_speed", "get_penetration_recovery_speed");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "enhanced_internal_edge_removal"), "set_enhanced_internal_edge_removal", "get_enhanced_internal_edge_removal");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "walk_stairs_min_step_forward"), "set_walk_stairs_min_step_forward", "get_walk_stairs_min_step_forward");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "walk_stairs_step_forward_test"), "set_walk_stairs_step_forward_test", "get_walk_stairs_step_forward_test");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "walk_stairs_cos_angle_forward_contact"), "set_walk_stairs_cos_angle_forward_contact", "get_walk_stairs_cos_angle_forward_contact");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "walk_stairs_step_down_extra"), "set_walk_stairs_step_down_extra", "get_walk_stairs_step_down_extra");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_layer", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_layer", "get_collision_layer");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");

	BIND_ENUM_CONSTANT(GROUND_STATE_ON_GROUND);
	BIND_ENUM_CONSTANT(GROUND_STATE_ON_STEEP_GROUND);
	BIND_ENUM_CONSTANT(GROUND_STATE_NOT_SUPPORTED);
	BIND_ENUM_CONSTANT(GROUND_STATE_IN_AIR);
}
