/**************************************************************************/
/*  jolt_soft_body_3d.h                                                   */
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

#include "jolt_object_3d.h"

#include "core/variant/typed_array.h"
#include "servers/physics_3d/physics_server_3d_enums.h"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyManifold.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>

class JoltArea3D;
class JoltSpace3D;
class PhysicsServer3DRenderingServerHandler;

class JoltSoftBody3D final : public JoltObject3D {
public:
	struct ClothSettings {
		float modulus = 2000.0f;
		float poisson = 0.3f;
		float bend = 5.0e-5f;
		float tether_slack = 0.01f;
		float damping = 0.05f;
		float friction = 0.5f;
		float bounce = 0.0f;
		float thickness = 0.005f;
		int iterations = 10;
		float gravity_factor = 1.0f;
		float skin_multiplier = 1.0f;
		bool skin_constraints = true;
		bool update_position = true;
		bool double_sided = true;
		bool manual = false;
		float air_density = 1.225f;
		Vector3 wind;
		float drag = 1.0f;
		float lift = 0.3f;
		float pressure = 0.0f;
		float ambient = 101325.0f;
		bool sealed = false;
	};

private:
	struct ClothContact {
		RID body;
		Vector3 point;
		Vector3 normal;
		float depth = -FLT_MAX;
		float speed = 0.0f;
		int vertices = 0;
	};

	struct Cloth {
		Transform3D placement;
		Basis bake;
		LocalVector<Vector3> positions;
		LocalVector<JPH::uint32> faces;
		LocalVector<float> masses;
		LocalVector<int> bones;
		LocalVector<float> weights;
		LocalVector<float> max_distances;
		LocalVector<float> back_stops;
		LocalVector<Transform3D> binds;
		float back_stop_radius = 0.0f;
		float rest_volume = 0.0f;
		float gas_amount = 0.0f;
		float skin_pressure = 0.0f;
		float built_slack = 0.0f;

		LocalVector<Vector2> edge_terms;
		LocalVector<float> bend_terms;
		LocalVector<float> tether_lengths;

		ClothSettings settings;
		LocalVector<Transform3D> last_skin;
		LocalVector<Vector3> given_positions;
		LocalVector<float> given_tangents;
		LocalVector<JPH::Vec3> start_velocities;
		LocalVector<ClothContact> contacts;
	};

	Cloth *cloth = nullptr;

	HashSet<int> pinned_vertices;

	LocalVector<int> mesh_to_physics;
	LocalVector<JoltArea3D *> areas;
	LocalVector<Vector3> normals;
	LocalVector<RID> exceptions;

	RID mesh;

	JPH::SoftBodyCreationSettings *jolt_settings = new JPH::SoftBodyCreationSettings();

	JPH::Ref<JPH::SoftBodySharedSettings> shared_settings;
	LocalVector<float> base_rest_lengths;
	float compliance_step = 0.0f;

	float mass = 1.0f;
	float pressure = 0.0f;
	float linear_damping = 0.01f;
	float stiffness_coefficient = 0.5f;
	float shrinking_factor = 0.0f;

	int simulation_precision = 5;

	virtual JPH::BroadPhaseLayer _get_broad_phase_layer() const override;
	virtual JPH::ObjectLayer _get_object_layer() const override;

	virtual void _space_changing() override;
	virtual void _space_changed() override;

	virtual void _add_to_space() override;

	JPH::SoftBodySharedSettings *_create_shared_settings();
	JPH::SoftBodySharedSettings *_create_cloth_settings();

	Vector3 _compute_gravity() const;
	void _cloth_pre_step();
	void _update_cloth_mass();
	void _update_cloth_compliance();
	void _update_cloth_settings();

	void _apply_environmental_forces(float p_step);

	void _update_mass();
	void _update_pressure();
	void _update_damping();
	void _update_simulation_precision();
	void _update_group_filter();
	void _update_compliance(float p_step);
	void _update_rest_lengths();

	void _try_rebuild();

	void _mesh_changed();
	void _simulation_precision_changed();
	void _mass_changed();
	void _pressure_changed();
	void _stiffness_changed();
	void _shrinking_changed();
	void _damping_changed();
	void _pins_changed();
	void _vertices_changed();
	void _exceptions_changed();
	void _motion_changed();
	void _transform_changed();
	void _areas_changed();

public:
	JoltSoftBody3D();
	virtual ~JoltSoftBody3D() override;

	PackedInt32Array build(const Transform3D &p_frame, const PackedVector3Array &p_positions, const PackedInt32Array &p_indices, const Dictionary &p_settings);
	bool is_built() const { return cloth != nullptr; }

	void set_cloth_settings(const Dictionary &p_settings);
	Dictionary get_cloth_settings() const;

	PackedVector3Array get_vertices() const;
	PackedVector3Array get_velocities() const;
	float get_volume() const;
	float get_rest_volume() const;
	float get_gas_pressure() const;

	PackedByteArray write_vertices(const Transform3D &p_frame, uint64_t p_format) const;
	AABB get_local_bounds(const Transform3D &p_frame) const;

	void record_contacts(const JPH::Body &p_jolt_body, const JPH::SoftBodyManifold &p_manifold);
	Array get_contacts() const;

	void skin(const Transform3D &p_frame, const TypedArray<Transform3D> &p_joints, bool p_hard);
	void update(float p_delta);

	PackedByteArray save_state() const;
	bool restore_state(const PackedByteArray &p_state);

	void add_collision_exception(const RID &p_excepted_body);
	void remove_collision_exception(const RID &p_excepted_body);
	bool has_collision_exception(const RID &p_excepted_body) const;

	const LocalVector<RID> &get_collision_exceptions() const { return exceptions; }

	void add_area(JoltArea3D *p_area);
	void remove_area(JoltArea3D *p_area);

	virtual bool can_interact_with(const JoltBody3D &p_other) const override;
	virtual bool can_interact_with(const JoltSoftBody3D &p_other) const override;
	virtual bool can_interact_with(const JoltArea3D &p_other) const override;

	virtual bool reports_contacts() const override { return false; }

	virtual Vector3 get_velocity_at_position(const Vector3 &p_position) const override;

	virtual void pre_step(float p_step) override;

	void set_mesh(const RID &p_mesh);

	bool is_pickable() const { return pickable; }
	void set_pickable(bool p_enabled) { pickable = p_enabled; }

	bool is_sleeping() const;
	void set_is_sleeping(bool p_enabled);

	bool is_sleep_allowed() const;
	void set_is_sleep_allowed(bool p_enabled);

	void put_to_sleep() { set_is_sleeping(true); }
	void wake_up() { set_is_sleeping(false); }

	int get_simulation_precision() const { return simulation_precision; }
	void set_simulation_precision(int p_precision);

	float get_mass() const { return mass; }
	void set_mass(float p_mass);

	float get_stiffness_coefficient() const;
	void set_stiffness_coefficient(float p_coefficient);

	float get_shrinking_factor() const;
	void set_shrinking_factor(float p_shrinking_factor);

	float get_pressure() const { return pressure; }
	void set_pressure(float p_pressure);

	float get_linear_damping() const { return linear_damping; }
	void set_linear_damping(float p_damping);

	float get_drag() const;
	void set_drag(float p_drag);

	Variant get_state(PS3DE::BodyState p_state) const;
	void set_state(PS3DE::BodyState p_state, const Variant &p_value);

	Transform3D get_transform() const;
	void set_transform(const Transform3D &p_transform);

	AABB get_bounds() const;

	void update_rendering_server(PhysicsServer3DRenderingServerHandler *p_rendering_server_handler);

	Vector3 get_vertex_position(int p_index);
	void set_vertex_position(int p_index, const Vector3 &p_position);

	void pin_vertex(int p_index);
	void unpin_vertex(int p_index);

	void unpin_all_vertices();

	bool is_vertex_pinned(int p_index) const;

	void apply_vertex_impulse(int p_index, const Vector3 &p_impulse);
	void apply_vertex_force(int p_index, const Vector3 &p_force);
	void apply_central_impulse(const Vector3 &p_impulse);
	void apply_central_force(const Vector3 &p_force);
};
