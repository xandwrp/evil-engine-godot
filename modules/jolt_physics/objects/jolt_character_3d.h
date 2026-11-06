/**************************************************************************/
/*  jolt_character_3d.h                                                   */
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

#pragma once

#include "core/object/ref_counted.h"
#include "core/templates/rid.h"
#include "core/variant/binder_common.h"
#include "core/variant/typed_array.h"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Character/CharacterVirtual.h>

class JoltSpace3D;

class JoltCharacter3D : public RefCounted {
	GDCLASS(JoltCharacter3D, RefCounted);

public:
	enum GroundState {
		GROUND_STATE_ON_GROUND,
		GROUND_STATE_ON_STEEP_GROUND,
		GROUND_STATE_NOT_SUPPORTED,
		GROUND_STATE_IN_AIR,
	};

private:
	class ContactFilter final : public JPH::CharacterContactListener {
	public:
		const JoltCharacter3D *owner = nullptr;
		bool OnContactValidate(const JPH::CharacterVirtual *p_character, const JPH::CharacterContact &p_contact) override;
		void OnContactAdded(const JPH::CharacterVirtual *p_character, const JPH::CharacterContact &p_contact, JPH::CharacterContactSettings &r_settings) override;
		void OnContactPersisted(const JPH::CharacterVirtual *p_character, const JPH::CharacterContact &p_contact, JPH::CharacterContactSettings &r_settings) override;
	};

	static constexpr float FLOOR_LIP_TOLERANCE = 0.01f;
	static constexpr float FLOOR_LIP_INSET = 0.01f;
	static constexpr float FLOOR_LIP_FACE_COS = 0.999848f;

	JPH::Ref<JPH::CharacterVirtual> character;
	ContactFilter contact_filter;
	RID space_rid;
	JPH::ObjectLayer object_layer = 0;

	Vector3 slid_velocity;

	float mass = 70.0f;
	float max_strength = 100.0f;
	float max_slope_angle = Math::deg_to_rad(50.0f);
	float character_padding = 0.02f;
	float predictive_contact_distance = 0.1f;
	float penetration_recovery_speed = 1.0f;
	bool enhanced_internal_edge_removal = true;
	float walk_stairs_min_step_forward = 0.02f;
	float walk_stairs_step_forward_test = 0.15f;
	float walk_stairs_cos_angle_forward_contact = Math::cos(Math::deg_to_rad(75.0f));
	float walk_stairs_step_down_extra = 0.0f;
	uint32_t collision_layer = 1;
	uint32_t collision_mask = 1;

	JoltSpace3D *_get_space() const;
	JPH::ShapeRefC _make_box(const Vector3 &p_size, const Vector3 &p_center, float p_convex_radius) const;
	bool _blocked_by_steep(const JPH::Vec3 &p_direction, JPH::Vec3 &r_normal) const;
	bool _is_floor_lip(const JPH::CharacterContact &p_contact) const;
	static bool _pushes_toward(const JPH::CharacterContact &p_contact);
	static Vector3 _follow_velocity(const JPH::CharacterContact &p_contact);
	bool _step_up(JoltSpace3D &p_space, const JPH::Vec3 &p_forward, const JPH::Vec3 &p_blocked, float p_delta, float p_step_up, float p_step_down, const JPH::Vec3 &p_gravity);
	bool _face_ahead(const JoltSpace3D &p_space, const JPH::Vec3 &p_displacement, JPH::Vec3 &r_normal) const;
	void _walk_onto(JoltSpace3D &p_space, const JPH::Vec3 &p_normal, const JPH::Vec3 &p_left, float p_time, const JPH::Vec3 &p_gravity, const JPH::CharacterVirtual::ExtendedUpdateSettings &p_settings);

protected:
	static void _bind_methods();

public:
	~JoltCharacter3D();

	bool create(RID p_space, const Vector3 &p_position, const Vector3 &p_size, const Vector3 &p_center, float p_convex_radius);
	void destroy();
	bool is_created() const { return character != nullptr; }

	bool set_box(const Vector3 &p_size, const Vector3 &p_center, float p_convex_radius, float p_max_penetration);

	Vector3 get_position() const;
	void set_position(const Vector3 &p_position);

	Vector3 get_linear_velocity() const;
	void set_linear_velocity(const Vector3 &p_velocity);

	void move(float p_delta, const Vector3 &p_gravity, float p_step_up, float p_stick_down);
	void refresh_contacts();
	bool can_walk_stairs(const Vector3 &p_velocity) const;
	bool walk_stairs(float p_delta, float p_step_up, const Vector3 &p_step_forward, const Vector3 &p_step_forward_test, float p_step_down_extra);

	Vector3 get_slid_velocity() const { return slid_velocity; }

	GroundState get_ground_state() const;
	bool is_supported() const;
	Vector3 get_ground_normal() const;
	Vector3 get_ground_position() const;
	Vector3 get_ground_velocity() const;
	RID get_ground_body() const;
	int get_ground_shape() const;
	int get_ground_face() const;

	TypedArray<Dictionary> get_contacts() const;

	PackedByteArray save_state() const;
	bool restore_state(const PackedByteArray &p_state);

	float get_mass() const { return mass; }
	void set_mass(float p_mass);

	float get_max_strength() const { return max_strength; }
	void set_max_strength(float p_max_strength);

	float get_max_slope_angle() const { return max_slope_angle; }
	void set_max_slope_angle(float p_angle);

	float get_character_padding() const { return character_padding; }
	void set_character_padding(float p_padding);

	float get_predictive_contact_distance() const { return predictive_contact_distance; }
	void set_predictive_contact_distance(float p_distance);

	float get_penetration_recovery_speed() const { return penetration_recovery_speed; }
	void set_penetration_recovery_speed(float p_speed);

	bool get_enhanced_internal_edge_removal() const { return enhanced_internal_edge_removal; }
	void set_enhanced_internal_edge_removal(bool p_enable);

	float get_walk_stairs_min_step_forward() const { return walk_stairs_min_step_forward; }
	void set_walk_stairs_min_step_forward(float p_distance) { walk_stairs_min_step_forward = p_distance; }

	float get_walk_stairs_step_forward_test() const { return walk_stairs_step_forward_test; }
	void set_walk_stairs_step_forward_test(float p_distance) { walk_stairs_step_forward_test = p_distance; }

	float get_walk_stairs_cos_angle_forward_contact() const { return walk_stairs_cos_angle_forward_contact; }
	void set_walk_stairs_cos_angle_forward_contact(float p_cos_angle) { walk_stairs_cos_angle_forward_contact = p_cos_angle; }

	float get_walk_stairs_step_down_extra() const { return walk_stairs_step_down_extra; }
	void set_walk_stairs_step_down_extra(float p_distance) { walk_stairs_step_down_extra = p_distance; }

	uint32_t get_collision_layer() const { return collision_layer; }
	void set_collision_layer(uint32_t p_layer);

	uint32_t get_collision_mask() const { return collision_mask; }
	void set_collision_mask(uint32_t p_mask);
};

VARIANT_ENUM_CAST(JoltCharacter3D::GroundState);
