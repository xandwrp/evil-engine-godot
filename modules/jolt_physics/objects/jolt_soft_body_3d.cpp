/**************************************************************************/
/*  jolt_soft_body_3d.cpp                                                 */
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

#include "jolt_soft_body_3d.h"

#include "../jolt_project_settings.h"
#include "../misc/jolt_type_conversions.h"
#include "../spaces/jolt_broad_phase_layer.h"
#include "../spaces/jolt_space_3d.h"
#include "jolt_area_3d.h"
#include "jolt_body_3d.h"
#include "jolt_group_filter.h"

#include "core/config/engine.h"
#include "servers/physics_3d/physics_server_3d_rendering_server_handler.h"
#include "servers/rendering/rendering_server.h"

#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/StateRecorderImpl.h>

namespace {

template <typename TJoltVertex>
void pin_vertices(const JoltSoftBody3D &p_body, const HashSet<int> &p_pinned_vertices, const LocalVector<int> &p_mesh_to_physics, JPH::Array<TJoltVertex> &r_physics_vertices) {
	const int mesh_vertex_count = p_mesh_to_physics.size();
	const int physics_vertex_count = (int)r_physics_vertices.size();

	for (int mesh_index : p_pinned_vertices) {
		ERR_CONTINUE_MSG(mesh_index < 0 || mesh_index >= mesh_vertex_count, vformat("Index %d of pinned vertex in soft body '%s' is out of bounds. There are only %d vertices in the current mesh.", mesh_index, p_body.to_string(), mesh_vertex_count));

		const int physics_index = p_mesh_to_physics[mesh_index];
		ERR_CONTINUE_MSG(physics_index < 0 || physics_index >= physics_vertex_count, vformat("Index %d of pinned vertex in soft body '%s' is out of bounds. There are only %d vertices in the current mesh. This should not happen. Please report this.", physics_index, p_body.to_string(), physics_vertex_count));

		r_physics_vertices[physics_index].mInvMass = 0.0f;
	}
}

JPH::uint64 edge_key(JPH::uint32 p_a, JPH::uint32 p_b) {
	return p_a < p_b ? (JPH::uint64(p_a) << 32) | p_b : (JPH::uint64(p_b) << 32) | p_a;
}

float triangle_area(const Vector3 &p_a, const Vector3 &p_b, const Vector3 &p_c) {
	return 0.5f * (float)(p_b - p_a).cross(p_c - p_a).length();
}

void read_cloth_settings(const Dictionary &p_settings, JoltSoftBody3D::ClothSettings &r_settings) {
	r_settings.modulus = MAX((float)p_settings.get("modulus", r_settings.modulus), 0.0f);
	r_settings.poisson = CLAMP((float)p_settings.get("poisson", r_settings.poisson), 0.0f, 0.49f);
	r_settings.bend = MAX((float)p_settings.get("bend", r_settings.bend), 0.0f);
	r_settings.tether_slack = p_settings.get("tether_slack", r_settings.tether_slack);
	r_settings.damping = MAX((float)p_settings.get("damping", r_settings.damping), 0.0f);
	r_settings.friction = MAX((float)p_settings.get("friction", r_settings.friction), 0.0f);
	r_settings.bounce = CLAMP((float)p_settings.get("bounce", r_settings.bounce), 0.0f, 1.0f);
	r_settings.thickness = MAX((float)p_settings.get("thickness", r_settings.thickness), 0.0f);
	r_settings.iterations = MAX((int)p_settings.get("iterations", r_settings.iterations), 1);
	r_settings.gravity_factor = p_settings.get("gravity_factor", r_settings.gravity_factor);
	r_settings.skin_multiplier = MAX((float)p_settings.get("skin_multiplier", r_settings.skin_multiplier), 0.0f);
	r_settings.skin_constraints = p_settings.get("skin_constraints", r_settings.skin_constraints);
	r_settings.update_position = p_settings.get("update_position", r_settings.update_position);
	r_settings.double_sided = p_settings.get("double_sided", r_settings.double_sided);
}

} // namespace

JPH::BroadPhaseLayer JoltSoftBody3D::_get_broad_phase_layer() const {
	return JoltBroadPhaseLayer::BODY_DYNAMIC;
}

JPH::ObjectLayer JoltSoftBody3D::_get_object_layer() const {
	ERR_FAIL_NULL_V(space, 0);

	return space->map_to_object_layer(_get_broad_phase_layer(), collision_layer, collision_mask);
}

void JoltSoftBody3D::_space_changing() {
	JoltObject3D::_space_changing();

	if (in_space()) {
		jolt_settings = new JPH::SoftBodyCreationSettings(jolt_body->GetSoftBodyCreationSettings());
		jolt_settings->mSettings = nullptr;
		jolt_settings->mVertexRadius = JoltProjectSettings::soft_body_point_radius;
	}

	shared_settings = nullptr;
}

void JoltSoftBody3D::_space_changed() {
	JoltObject3D::_space_changed();

	if (cloth != nullptr) {
		_update_cloth_mass();
		_update_cloth_settings();
		_update_group_filter();
		return;
	}

	_update_mass();
	_update_pressure();
	_update_damping();
	_update_simulation_precision();
	_update_group_filter();
}

void JoltSoftBody3D::_add_to_space() {
	if (unlikely(space == nullptr || (cloth == nullptr && !mesh.is_valid()))) {
		return;
	}

	JPH::SoftBodySharedSettings *new_shared_settings = cloth != nullptr ? _create_cloth_settings() : _create_shared_settings();
	ERR_FAIL_NULL(new_shared_settings);

	if (cloth != nullptr) {
		jolt_settings->mPosition = to_jolt_r(cloth->placement.origin);
		jolt_settings->mRotation = to_jolt(cloth->placement.basis);
		jolt_settings->mPressure = 0.0f;
	}

	JPH::CollisionGroup::GroupID group_id = 0;
	JPH::CollisionGroup::SubGroupID sub_group_id = 0;
	JoltGroupFilter::encode_object(this, group_id, sub_group_id);

	jolt_settings->mSettings = new_shared_settings;
	jolt_settings->mUserData = reinterpret_cast<JPH::uint64>(this);
	jolt_settings->mObjectLayer = _get_object_layer();
	jolt_settings->mCollisionGroup = JPH::CollisionGroup(nullptr, group_id, sub_group_id);
	jolt_settings->mMaxLinearVelocity = JoltProjectSettings::max_linear_velocity;

	JPH::Body *new_jolt_body = space->add_object(*this, *jolt_settings);
	if (new_jolt_body == nullptr) {
		return;
	}

	jolt_body = new_jolt_body;

	delete jolt_settings;
	jolt_settings = nullptr;
}

JPH::SoftBodySharedSettings *JoltSoftBody3D::_create_shared_settings() {
	RenderingServer *rendering = RenderingServer::get_singleton();

	// TODO: calling RenderingServer::mesh_surface_get_arrays() from the physics thread
	// is not safe and can deadlock when physics/3d/run_on_separate_thread is enabled.
	// This method blocks on the main thread to return data, but the main thread may be
	// blocked waiting on us in PhysicsServer3D::sync().
	const Array mesh_data = rendering->mesh_surface_get_arrays(mesh, 0);
	ERR_FAIL_COND_V(mesh_data.is_empty(), nullptr);

	const PackedInt32Array mesh_indices = mesh_data[RSE::ARRAY_INDEX];
	ERR_FAIL_COND_V(mesh_indices.is_empty(), nullptr);

	const PackedVector3Array mesh_vertices = mesh_data[RSE::ARRAY_VERTEX];
	ERR_FAIL_COND_V(mesh_vertices.is_empty(), nullptr);

	JPH::SoftBodySharedSettings *settings = new JPH::SoftBodySharedSettings();
	JPH::Array<JPH::SoftBodySharedSettings::Vertex> &physics_vertices = settings->mVertices;
	JPH::Array<JPH::SoftBodySharedSettings::Face> &physics_faces = settings->mFaces;

	HashMap<Vector3, int> vertex_to_physics;

	const int mesh_vertex_count = mesh_vertices.size();
	const int mesh_index_count = mesh_indices.size();

	mesh_to_physics.resize(mesh_vertex_count);
	for (int &index : mesh_to_physics) {
		index = -1;
	}
	physics_vertices.reserve(mesh_vertex_count);
	vertex_to_physics.reserve(mesh_vertex_count);

	int physics_index_count = 0;

	const JPH::RVec3 body_position = jolt_settings->mPosition;

	for (int i = 0; i < mesh_index_count; i += 3) {
		int physics_face[3];

		for (int j = 0; j < 3; ++j) {
			const int mesh_index = mesh_indices[i + j];
			const Vector3 vertex = mesh_vertices[mesh_index];

			HashMap<Vector3, int>::Iterator iter_physics_index = vertex_to_physics.find(vertex);

			if (iter_physics_index == vertex_to_physics.end()) {
				physics_vertices.emplace_back(JPH::Float3((float)(vertex.x - body_position.GetX()), (float)(vertex.y - body_position.GetY()), (float)(vertex.z - body_position.GetZ())), JPH::Float3(0.0f, 0.0f, 0.0f), 1.0f);
				iter_physics_index = vertex_to_physics.insert(vertex, physics_index_count++);
			}

			physics_face[j] = iter_physics_index->value;
			mesh_to_physics[mesh_index] = iter_physics_index->value;
		}

		if (physics_face[0] == physics_face[1] || physics_face[0] == physics_face[2] || physics_face[1] == physics_face[2]) {
			continue; // We skip degenerate faces, since they're problematic, and Jolt will assert about it anyway.
		}

		// Jolt uses a different winding order, so we swap the indices to account for that.
		physics_faces.emplace_back((JPH::uint32)physics_face[2], (JPH::uint32)physics_face[1], (JPH::uint32)physics_face[0]);
	}

	// Pin whatever pinned vertices we have currently. This is used during the `Optimize` call below to order the
	// constraints. Note that it's fine if the pinned vertices change later, but that will reduce the effectiveness
	// of the constraints a bit.
	pin_vertices(*this, pinned_vertices, mesh_to_physics, physics_vertices);

	JPH::SoftBodySharedSettings::VertexAttributes vertex_attrib;
	settings->CreateConstraints(&vertex_attrib, 1, JPH::SoftBodySharedSettings::EBendType::None);
	settings->Optimize();

	base_rest_lengths.resize((uint32_t)settings->mEdgeConstraints.size());
	for (uint32_t i = 0; i < base_rest_lengths.size(); ++i) {
		base_rest_lengths[i] = settings->mEdgeConstraints[i].mRestLength;
	}

	shared_settings = settings;
	_update_rest_lengths();
	_update_compliance(space->get_last_step() > 0.0f ? space->get_last_step() : 1.0f / Engine::get_singleton()->get_user_physics_ticks_per_second());

	return settings;
}

Vector3 JoltSoftBody3D::_compute_gravity() const {
	const Vector3 com_position = to_godot(jolt_body->GetCenterOfMassPosition());

	bool gravity_done = false;
	Vector3 gravity;
	for (JoltArea3D *area : areas) {
		gravity_done = JoltArea3D::apply_override(gravity, area->get_gravity_mode(), [&]() {
			return area->compute_gravity(com_position);
		});

		if (gravity_done) {
			break;
		}
	}

	if (!gravity_done) {
		gravity += space->get_default_area()->compute_gravity(com_position);
	}

	return gravity;
}

void JoltSoftBody3D::_apply_environmental_forces(float p_step) {
	// Get approximation of the center of the soft body.
	Vector3 com_position = to_godot(jolt_body->GetCenterOfMassPosition());

	LocalVector<JoltArea3D *> wind_areas;
	for (JoltArea3D *area : areas) {
		if (area->get_wind_pressure() > CMP_EPSILON) {
			wind_areas.push_back(area);
		}
	}

	const Vector3 gravity = _compute_gravity();

	// Apply gravity to soft body. Note that this only works so long as vertices have uniform mass (excluding pinned vertices).
	jolt_body->AddForce(to_jolt(gravity) * mass);

	if (!wind_areas.is_empty()) {
		JPH::SoftBodyMotionProperties &motion_properties = static_cast<JPH::SoftBodyMotionProperties &>(*jolt_body->GetMotionPropertiesUnchecked());
		JPH::Array<JPH::SoftBodyVertex> &physics_vertices = motion_properties.GetVertices();

		for (const JPH::SoftBodySharedSettings::Face &physics_face : motion_properties.GetFaces()) {
			JPH::SoftBodyVertex &physics_vertex0 = physics_vertices[physics_face.mVertex[0]];
			JPH::SoftBodyVertex &physics_vertex1 = physics_vertices[physics_face.mVertex[1]];
			JPH::SoftBodyVertex &physics_vertex2 = physics_vertices[physics_face.mVertex[2]];

			// Calculate the triangle centroid.
			Vector3 v0 = to_godot(physics_vertex0.mPosition);
			Vector3 v1 = to_godot(physics_vertex1.mPosition);
			Vector3 v2 = to_godot(physics_vertex2.mPosition);
			Vector3 centroid = com_position + (v0 + v1 + v2) * real_t(1.0 / 3.0);

			// Calculate the triangle normal.
			Vector3 normal = (v2 - v0).cross(v1 - v0);
			real_t normal_length = normal.length();
			if (normal_length > real_t(1.0e-6)) { // If the normal is near zero, the area is near zero so we can skip this triangle.
				normal /= normal_length;

				// Area is half the length of the cross product of two sides.
				real_t triangle_area = real_t(0.5) * normal_length;

				// Accumulate wind forces from all wind areas.
				Vector3 wind_force;
				for (const JoltArea3D *area : wind_areas) {
					const Vector3 &wind_direction = area->get_wind_direction();
					const Vector3 &wind_source = area->get_wind_source();

					// Calculate attenuation factor based on distance from wind source to triangle centroid.
					// We do not allow a projection below 1 to ensure that we never amplify and to avoid NaNs when the value would be negative.
					real_t projection_toward_centroid = MAX((centroid - wind_source).dot(wind_direction), real_t(1.0));
					real_t attenuation_over_distance = Math::pow(projection_toward_centroid, -real_t(area->get_wind_attenuation_factor()));

					// Calculate force magnitude.
					real_t force_magnitude = area->get_wind_pressure() * triangle_area;

					// Calculate the resulting wind force on the triangle by projecting wind direction onto triangle normal.
					// Divide by 3 to distribute force equally over each vertex.
					wind_force += (force_magnitude * attenuation_over_distance * real_t(1.0 / 3.0) * normal.dot(wind_direction)) * normal;
				}

				// Apply the force as an impulse over the timestep.
				JPH::Vec3 impulse = to_jolt(wind_force * p_step);
				physics_vertex0.mVelocity += impulse * physics_vertex0.mInvMass;
				physics_vertex1.mVelocity += impulse * physics_vertex1.mInvMass;
				physics_vertex2.mVelocity += impulse * physics_vertex2.mInvMass;
			}
		}
	}
}

void JoltSoftBody3D::_update_mass() {
	if (cloth != nullptr) {
		_update_cloth_mass();
		return;
	}

	if (!in_space()) {
		return;
	}

	JPH::SoftBodyMotionProperties &motion_properties = static_cast<JPH::SoftBodyMotionProperties &>(*jolt_body->GetMotionPropertiesUnchecked());
	JPH::Array<JPH::SoftBodyVertex> &physics_vertices = motion_properties.GetVertices();

	const float inverse_vertex_mass = (float)physics_vertices.size() / mass;

	for (JPH::SoftBodyVertex &vertex : physics_vertices) {
		vertex.mInvMass = inverse_vertex_mass;
	}

	pin_vertices(*this, pinned_vertices, mesh_to_physics, physics_vertices);
}

void JoltSoftBody3D::_update_pressure() {
	if (cloth != nullptr) {
		return;
	}

	if (!in_space()) {
		jolt_settings->mPressure = pressure;
		return;
	}

	JPH::SoftBodyMotionProperties &motion_properties = static_cast<JPH::SoftBodyMotionProperties &>(*jolt_body->GetMotionPropertiesUnchecked());
	motion_properties.SetPressure(pressure);
}

void JoltSoftBody3D::_update_damping() {
	if (cloth != nullptr) {
		return;
	}

	if (!in_space()) {
		jolt_settings->mLinearDamping = linear_damping;
		return;
	}

	JPH::SoftBodyMotionProperties &motion_properties = static_cast<JPH::SoftBodyMotionProperties &>(*jolt_body->GetMotionPropertiesUnchecked());
	motion_properties.SetLinearDamping(linear_damping);
}

void JoltSoftBody3D::_update_simulation_precision() {
	if (cloth != nullptr) {
		return;
	}

	if (!in_space()) {
		jolt_settings->mNumIterations = (JPH::uint32)simulation_precision;
		return;
	}

	JPH::SoftBodyMotionProperties &motion_properties = static_cast<JPH::SoftBodyMotionProperties &>(*jolt_body->GetMotionPropertiesUnchecked());
	motion_properties.SetNumIterations((JPH::uint32)simulation_precision);
}

void JoltSoftBody3D::_update_group_filter() {
	JPH::GroupFilter *group_filter = !exceptions.is_empty() ? JoltGroupFilter::instance : nullptr;

	if (!in_space()) {
		jolt_settings->mCollisionGroup.SetGroupFilter(group_filter);
	} else {
		jolt_body->GetCollisionGroup().SetGroupFilter(group_filter);
	}
}

void JoltSoftBody3D::_update_compliance(float p_step) {
	compliance_step = p_step;

	if (shared_settings == nullptr || simulation_precision <= 0) {
		return;
	}

	// Since Godot's stiffness is input as a coefficient between 0 and 1, and Jolt uses actual stiffness for its
	// edge constraints, we must map one to the other.
	//
	// Godot uses classic PBD edge constraints, which have a stiffness parameter k that is used in the position correction formula as follows:
	// delta_x1 = -k * w1 / (w1 + w2) * (l - l0) / l * (x2 - x1)
	// where k is the stiffness, w1 and w2 are the inverse masses of the two vertices, l is the current length of the edge = |x2 - x1|, l0 is the rest length of the edge, and x1 and x2 are the vertex positions.
	//
	// Jolt uses XPBD which goes as follows:
	// delta_x1 = -w1 / (w1 + w2 + compliance / dt^2) * (l - l0) / l * (x2 - x1)
	// where compliance is the inverse of stiffness and dt is the timestep of one substep.
	//
	// We can derive Jolt's compliance from Godot's stiffness by evaluating:
	// k * w1 / (w1 + w2) = w1 / (w1 + w2 + compliance / dt^2)
	// which simplifies to:
	// compliance = dt^2 * (1 / k - 1) * (w1 + w2)
	//
	// The substep is the step the space is actually given over the iteration count, so this is refreshed whenever that step changes.

	const float w1_plus_w2 = 2.0f * shared_settings->mVertices.size() / mass;
	const float dt = p_step / simulation_precision;
	const float compliance = stiffness_coefficient > 0.0f ? dt * dt * (1.0f / stiffness_coefficient - 1.0f) * w1_plus_w2 : FLT_MAX;

	for (JPH::SoftBodySharedSettings::Edge &edge : shared_settings->mEdgeConstraints) {
		edge.mCompliance = compliance;
	}
}

void JoltSoftBody3D::_update_rest_lengths() {
	if (shared_settings == nullptr) {
		return;
	}

	const float multiplier = 1.0f - shrinking_factor;
	JPH::Array<JPH::SoftBodySharedSettings::Edge> &edges = shared_settings->mEdgeConstraints;
	ERR_FAIL_COND(edges.size() != base_rest_lengths.size());

	for (uint32_t i = 0; i < base_rest_lengths.size(); ++i) {
		edges[i].mRestLength = base_rest_lengths[i] * multiplier;
	}
}

void JoltSoftBody3D::_try_rebuild() {
	if (space != nullptr) {
		_reset_space();
	}
}

void JoltSoftBody3D::_mesh_changed() {
	_try_rebuild();
}

void JoltSoftBody3D::_simulation_precision_changed() {
	_update_simulation_precision();
	_update_compliance(compliance_step);
	wake_up();
}

void JoltSoftBody3D::_mass_changed() {
	if (cloth != nullptr) {
		float total = 0.0f;
		for (float vertex_mass : cloth->masses) {
			total += vertex_mass;
		}
		if (total > 0.0f) {
			for (float &vertex_mass : cloth->masses) {
				vertex_mass *= mass / total;
			}
		}
	}

	_update_mass();
	_update_compliance(compliance_step);
	wake_up();
}

void JoltSoftBody3D::_stiffness_changed() {
	_update_compliance(compliance_step);
	wake_up();
}

void JoltSoftBody3D::_shrinking_changed() {
	_update_rest_lengths();
	wake_up();
}

void JoltSoftBody3D::_pressure_changed() {
	_update_pressure();
	wake_up();
}

void JoltSoftBody3D::_damping_changed() {
	_update_damping();
	wake_up();
}

void JoltSoftBody3D::_pins_changed() {
	_update_mass();
	wake_up();
}

void JoltSoftBody3D::_vertices_changed() {
	wake_up();
}

void JoltSoftBody3D::_exceptions_changed() {
	_update_group_filter();
}

void JoltSoftBody3D::_motion_changed() {
	wake_up();
}

void JoltSoftBody3D::_transform_changed() {
	wake_up();
}

void JoltSoftBody3D::_areas_changed() {
	wake_up();
}

JoltSoftBody3D::JoltSoftBody3D() :
		JoltObject3D(OBJECT_TYPE_SOFT_BODY) {
	jolt_settings->mRestitution = 0.0f;
	jolt_settings->mFriction = 1.0f;
	jolt_settings->mUpdatePosition = true;
	jolt_settings->mMakeRotationIdentity = false;
}

JoltSoftBody3D::~JoltSoftBody3D() {
	if (jolt_settings != nullptr) {
		delete jolt_settings;
		jolt_settings = nullptr;
	}

	if (cloth != nullptr) {
		memdelete(cloth);
		cloth = nullptr;
	}
}

JPH::SoftBodySharedSettings *JoltSoftBody3D::_create_cloth_settings() {
	const ClothSettings &cs = cloth->settings;
	const uint32_t vertex_count = cloth->positions.size();

	JPH::SoftBodySharedSettings *settings = new JPH::SoftBodySharedSettings();

	settings->mVertices.resize(vertex_count);
	for (uint32_t i = 0; i < vertex_count; ++i) {
		JPH::SoftBodySharedSettings::Vertex &vertex = settings->mVertices[i];
		const Vector3 &position = cloth->positions[i];
		vertex.mPosition = JPH::Float3((float)position.x, (float)position.y, (float)position.z);
		vertex.mVelocity = JPH::Float3(0.0f, 0.0f, 0.0f);
		const bool skinned = !cloth->weights.is_empty() && cloth->weights[i * 4] > 0.0f;
		const bool kinematic = skinned && cloth->max_distances[i] <= 0.0f;
		vertex.mInvMass = (cloth->masses[i] > 0.0f && !kinematic) ? 1.0f / cloth->masses[i] : 0.0f;
	}

	HashSet<JPH::uint64> face_edges;
	for (uint32_t f = 0; f < cloth->faces.size(); f += 3) {
		const JPH::uint32 a = cloth->faces[f];
		const JPH::uint32 b = cloth->faces[f + 1];
		const JPH::uint32 c = cloth->faces[f + 2];
		settings->mFaces.emplace_back(a, b, c);
		face_edges.insert(edge_key(a, b));
		face_edges.insert(edge_key(b, c));
		face_edges.insert(edge_key(c, a));
	}

	const bool tethered = cs.tether_slack >= 0.0f;
	const JPH::SoftBodySharedSettings::VertexAttributes attributes(0.0f, 0.0f, 0.0f, tethered ? JPH::SoftBodySharedSettings::ELRAType::GeodesicDistance : JPH::SoftBodySharedSettings::ELRAType::None, 1.0f + MAX(cs.tether_slack, 0.0f));
	settings->CreateConstraints(&attributes, 1, JPH::SoftBodySharedSettings::EBendType::Dihedral);
	cloth->built_slack = MAX(cs.tether_slack, 0.0f);

	JPH::Array<JPH::SoftBodySharedSettings::Edge> &edges = settings->mEdgeConstraints;
	JPH::uint kept = 0;
	for (const JPH::SoftBodySharedSettings::Edge &edge : edges) {
		if (face_edges.has(edge_key(edge.mVertex[0], edge.mVertex[1]))) {
			edges[kept++] = edge;
		}
	}
	edges.resize(kept);

	if (!cloth->weights.is_empty()) {
		const Transform3D unbake(cloth->bake.inverse());
		for (uint32_t j = 0; j < cloth->binds.size(); ++j) {
			settings->mInvBindMatrices.emplace_back(j, to_jolt(cloth->binds[j] * unbake));
		}

		for (uint32_t i = 0; i < vertex_count; ++i) {
			if (cloth->weights[i * 4] <= 0.0f) {
				continue;
			}

			JPH::SoftBodySharedSettings::Skinned skinned(i, MAX(cloth->max_distances[i], 0.0f), cloth->back_stops[i], cloth->back_stop_radius);
			for (int k = 0; k < 4; ++k) {
				skinned.mWeights[k] = JPH::SoftBodySharedSettings::SkinWeight((JPH::uint32)cloth->bones[i * 4 + k], cloth->weights[i * 4 + k]);
			}
			settings->mSkinnedConstraints.push_back(skinned);
		}

		settings->CalculateSkinnedConstraintNormals();
	}

	settings->Optimize();

	HashMap<JPH::uint64, Vector2> terms;
	for (uint32_t f = 0; f < cloth->faces.size(); f += 3) {
		const JPH::uint32 corner[3] = { cloth->faces[f], cloth->faces[f + 1], cloth->faces[f + 2] };
		const float area = triangle_area(cloth->positions[corner[0]], cloth->positions[corner[1]], cloth->positions[corner[2]]);
		if (area <= 0.0f) {
			continue;
		}

		for (int k = 0; k < 3; ++k) {
			const Vector3 &p0 = cloth->positions[corner[k]];
			const Vector3 &p1 = cloth->positions[corner[(k + 1) % 3]];
			const Vector3 &opposite = cloth->positions[corner[(k + 2) % 3]];
			const float c_sq = (float)p0.distance_squared_to(p1);
			const float a_sq = (float)p1.distance_squared_to(opposite);
			const float b_sq = (float)p0.distance_squared_to(opposite);

			Vector2 &term = terms[edge_key(corner[k], corner[(k + 1) % 3])];
			term.x += area / c_sq;
			term.y += (a_sq + b_sq - c_sq) / (8.0f * area);
		}
	}

	cloth->edge_terms.resize((uint32_t)edges.size());
	for (uint32_t i = 0; i < cloth->edge_terms.size(); ++i) {
		const Vector2 *term = terms.getptr(edge_key(edges[i].mVertex[0], edges[i].mVertex[1]));
		cloth->edge_terms[i] = term != nullptr ? *term : Vector2();
	}

	cloth->bend_terms.resize((uint32_t)settings->mDihedralBendConstraints.size());
	for (uint32_t i = 0; i < cloth->bend_terms.size(); ++i) {
		const JPH::uint32 *v = settings->mDihedralBendConstraints[i].mVertex;
		const Vector3 &e0 = cloth->positions[v[0]];
		const Vector3 &e1 = cloth->positions[v[1]];
		const float areas = triangle_area(e0, e1, cloth->positions[v[2]]) + triangle_area(e0, e1, cloth->positions[v[3]]);
		cloth->bend_terms[i] = areas > 0.0f ? 3.0f * (float)e0.distance_squared_to(e1) / areas : 0.0f;
	}

	cloth->tether_lengths.resize((uint32_t)settings->mLRAConstraints.size());
	for (uint32_t i = 0; i < cloth->tether_lengths.size(); ++i) {
		cloth->tether_lengths[i] = settings->mLRAConstraints[i].mMaxDistance / (1.0f + cloth->built_slack);
	}

	shared_settings = settings;
	_update_cloth_compliance();

	return settings;
}

void JoltSoftBody3D::_update_cloth_compliance() {
	if (cloth == nullptr || shared_settings == nullptr) {
		return;
	}

	const ClothSettings &cs = cloth->settings;
	const float shear = cs.modulus / (1.0f + cs.poisson);
	const float bulk = cs.modulus * cs.poisson / (1.0f - cs.poisson * cs.poisson);

	JPH::Array<JPH::SoftBodySharedSettings::Edge> &edges = shared_settings->mEdgeConstraints;
	for (uint32_t i = 0; i < cloth->edge_terms.size(); ++i) {
		const Vector2 &term = cloth->edge_terms[i];
		const float stiffness = MAX(shear * term.x + bulk * term.y, 0.1f * shear * term.x);
		edges[i].mCompliance = stiffness > 0.0f ? 1.0f / stiffness : FLT_MAX;
	}

	JPH::Array<JPH::SoftBodySharedSettings::DihedralBend> &bends = shared_settings->mDihedralBendConstraints;
	for (uint32_t i = 0; i < cloth->bend_terms.size(); ++i) {
		const float stiffness = cs.bend * cloth->bend_terms[i];
		bends[i].mCompliance = stiffness > 0.0f ? 1.0f / stiffness : FLT_MAX;
	}

	JPH::Array<JPH::SoftBodySharedSettings::LRA> &tethers = shared_settings->mLRAConstraints;
	for (uint32_t i = 0; i < cloth->tether_lengths.size(); ++i) {
		tethers[i].mMaxDistance = cs.tether_slack >= 0.0f ? cloth->tether_lengths[i] * (1.0f + cs.tether_slack) : FLT_MAX;
	}
}

void JoltSoftBody3D::_update_cloth_settings() {
	const ClothSettings &cs = cloth->settings;

	if (!in_space()) {
		jolt_settings->mNumIterations = (JPH::uint32)cs.iterations;
		jolt_settings->mLinearDamping = cs.damping;
		jolt_settings->mFriction = cs.friction;
		jolt_settings->mRestitution = cs.bounce;
		jolt_settings->mVertexRadius = cs.thickness;
		jolt_settings->mGravityFactor = cs.gravity_factor;
		jolt_settings->mUpdatePosition = cs.update_position;
		jolt_settings->mFacesDoubleSided = cs.double_sided;
		return;
	}

	JPH::SoftBodyMotionProperties &motion_properties = static_cast<JPH::SoftBodyMotionProperties &>(*jolt_body->GetMotionPropertiesUnchecked());
	motion_properties.SetNumIterations((JPH::uint32)cs.iterations);
	motion_properties.SetLinearDamping(cs.damping);
	motion_properties.SetVertexRadius(cs.thickness);
	motion_properties.SetGravityFactor(cs.gravity_factor);
	motion_properties.SetUpdatePosition(cs.update_position);
	motion_properties.SetFacesDoubleSided(cs.double_sided);
	motion_properties.SetEnableSkinConstraints(cs.skin_constraints);
	motion_properties.SetSkinnedMaxDistanceMultiplier(cs.skin_multiplier);
	jolt_body->SetFriction(cs.friction);
	jolt_body->SetRestitution(cs.bounce);
}

void JoltSoftBody3D::_update_cloth_mass() {
	if (!in_space()) {
		return;
	}

	JPH::SoftBodyMotionProperties &motion_properties = static_cast<JPH::SoftBodyMotionProperties &>(*jolt_body->GetMotionPropertiesUnchecked());
	JPH::Array<JPH::SoftBodyVertex> &physics_vertices = motion_properties.GetVertices();
	ERR_FAIL_COND(physics_vertices.size() != cloth->masses.size());

	for (uint32_t i = 0; i < cloth->masses.size(); ++i) {
		const bool skinned = !cloth->weights.is_empty() && cloth->weights[i * 4] > 0.0f;
		const bool kinematic = skinned && cloth->max_distances[i] <= 0.0f;
		physics_vertices[i].mInvMass = (cloth->masses[i] > 0.0f && !kinematic) ? 1.0f / cloth->masses[i] : 0.0f;
	}

	pin_vertices(*this, pinned_vertices, mesh_to_physics, physics_vertices);
}

PackedInt32Array JoltSoftBody3D::build(const Transform3D &p_frame, const PackedVector3Array &p_positions, const PackedInt32Array &p_indices, const Dictionary &p_settings) {
	ERR_FAIL_COND_V_MSG(p_indices.size() % 3 != 0, PackedInt32Array(), "A soft body's indices must come in threes.");

	const int given_count = p_positions.size();
	const real_t weld = MAX((real_t)(float)p_settings.get("weld", 0.0001f), (real_t)0.0);
	const real_t cell = MAX(weld, (real_t)1.0e-6);

	Cloth *built = memnew(Cloth);
	if (cloth != nullptr) {
		built->settings = cloth->settings;
	}
	read_cloth_settings(p_settings, built->settings);
	built->placement = p_frame.orthonormalized();
	built->bake = built->placement.basis.transposed() * p_frame.basis;

	LocalVector<int> given_to_welded;
	given_to_welded.resize(given_count);
	LocalVector<Vector3> welded;
	LocalVector<int> welded_first;
	HashMap<Vector3i, LocalVector<int>> grid;

	for (int i = 0; i < given_count; ++i) {
		const Vector3 p = p_positions[i];
		const Vector3i c((int)Math::floor(p.x / cell), (int)Math::floor(p.y / cell), (int)Math::floor(p.z / cell));
		int found = -1;
		for (int dx = -1; dx <= 1 && found < 0; ++dx) {
			for (int dy = -1; dy <= 1 && found < 0; ++dy) {
				for (int dz = -1; dz <= 1 && found < 0; ++dz) {
					const LocalVector<int> *candidates = grid.getptr(c + Vector3i(dx, dy, dz));
					if (candidates == nullptr) {
						continue;
					}
					for (int j : *candidates) {
						if (welded[j].distance_to(p) <= weld) {
							found = j;
							break;
						}
					}
				}
			}
		}

		if (found < 0) {
			found = (int)welded.size();
			welded.push_back(p);
			welded_first.push_back(i);
			grid[c].push_back(found);
		}

		given_to_welded[i] = found;
	}

	LocalVector<int> welded_to_final;
	welded_to_final.resize(welded.size());
	for (int &index : welded_to_final) {
		index = -1;
	}

	LocalVector<int> final_first;
	LocalVector<JPH::uint32> faces;
	for (int t = 0; t < p_indices.size(); t += 3) {
		int corner[3];
		for (int k = 0; k < 3; ++k) {
			const int given = p_indices[t + k];
			ERR_FAIL_INDEX_V_MSG(given, given_count, PackedInt32Array(), vformat("Soft body index %d is out of bounds of its %d positions.", given, given_count));
			corner[k] = given_to_welded[given];
		}
		if (corner[0] == corner[1] || corner[0] == corner[2] || corner[1] == corner[2]) {
			continue;
		}
		if (triangle_area(built->bake.xform(welded[corner[0]]), built->bake.xform(welded[corner[1]]), built->bake.xform(welded[corner[2]])) < 1.0e-12f) {
			continue;
		}

		for (int k = 2; k >= 0; --k) {
			int &final_index = welded_to_final[corner[k]];
			if (final_index < 0) {
				final_index = (int)built->positions.size();
				built->positions.push_back(built->bake.xform(welded[corner[k]]));
				final_first.push_back(welded_first[corner[k]]);
			}
			faces.push_back((JPH::uint32)final_index);
		}
	}

	if (faces.is_empty()) {
		memdelete(built);
		ERR_FAIL_V_MSG(PackedInt32Array(), "A soft body needs at least one triangle with an area.");
	}

	built->faces = faces;
	const uint32_t vertex_count = built->positions.size();

	PackedInt32Array given_to_final;
	given_to_final.resize(given_count);
	for (int i = 0; i < given_count; ++i) {
		given_to_final.write[i] = welded_to_final[given_to_welded[i]];
	}

	built->masses.resize(vertex_count);
	for (float &vertex_mass : built->masses) {
		vertex_mass = 0.0f;
	}

	const PackedFloat32Array given_masses = p_settings.get("masses", PackedFloat32Array());
	if (given_masses.size() == given_count) {
		for (int i = 0; i < given_count; ++i) {
			if (given_to_final[i] >= 0) {
				built->masses[given_to_final[i]] += MAX(given_masses[i], 0.0f);
			}
		}
	} else {
		const float density = MAX((float)p_settings.get("density", 0.15f), 0.0f);
		for (uint32_t f = 0; f < faces.size(); f += 3) {
			const float third = density * triangle_area(built->positions[faces[f]], built->positions[faces[f + 1]], built->positions[faces[f + 2]]) / 3.0f;
			built->masses[faces[f]] += third;
			built->masses[faces[f + 1]] += third;
			built->masses[faces[f + 2]] += third;
		}
	}

	const PackedInt32Array bones = p_settings.get("bones", PackedInt32Array());
	const PackedFloat32Array weights = p_settings.get("weights", PackedFloat32Array());
	if (bones.size() == given_count * 4 && weights.size() == given_count * 4) {
		const Array binds = p_settings.get("binds", Array());
		for (int j = 0; j < binds.size(); ++j) {
			built->binds.push_back(binds[j]);
		}

		const PackedFloat32Array max_distances = p_settings.get("max_distances", PackedFloat32Array());
		const PackedFloat32Array back_stops = p_settings.get("back_stops", PackedFloat32Array());
		built->back_stop_radius = MAX((float)p_settings.get("back_stop_radius", 40.0f), 0.0f);

		built->bones.resize(vertex_count * 4);
		built->weights.resize(vertex_count * 4);
		built->max_distances.resize(vertex_count);
		built->back_stops.resize(vertex_count);

		for (uint32_t i = 0; i < vertex_count; ++i) {
			const int given = final_first[i];

			int order[4] = { 0, 1, 2, 3 };
			for (int a = 0; a < 4; ++a) {
				for (int b = a + 1; b < 4; ++b) {
					if (weights[given * 4 + order[b]] > weights[given * 4 + order[a]]) {
						SWAP(order[a], order[b]);
					}
				}
			}

			float total = 0.0f;
			for (int k = 0; k < 4; ++k) {
				const int bone = bones[given * 4 + k];
				const float weight = weights[given * 4 + k];
				if (weight > 0.0f && (bone < 0 || bone >= (int)built->binds.size())) {
					const int bind_count = (int)built->binds.size();
					memdelete(built);
					ERR_FAIL_V_MSG(PackedInt32Array(), vformat("Soft body vertex %d is weighted to bone %d, past its %d binds.", given, bone, bind_count));
				}
				total += MAX(weight, 0.0f);
			}

			for (int k = 0; k < 4; ++k) {
				const float weight = total > 0.0f ? MAX(weights[given * 4 + order[k]], 0.0f) / total : 0.0f;
				built->bones[i * 4 + k] = weight > 0.0f ? bones[given * 4 + order[k]] : 0;
				built->weights[i * 4 + k] = weight;
			}

			built->max_distances[i] = max_distances.size() == given_count ? max_distances[given] : 0.0f;
			built->back_stops[i] = back_stops.size() == given_count ? back_stops[given] : FLT_MAX;
		}
	}

	float six_volume = 0.0f;
	for (uint32_t f = 0; f < faces.size(); f += 3) {
		six_volume += (float)built->positions[faces[f]].cross(built->positions[faces[f + 1]]).dot(built->positions[faces[f + 2]]);
	}
	built->rest_volume = six_volume / 6.0f;

	if (cloth != nullptr) {
		memdelete(cloth);
	}
	cloth = built;

	mesh = RID();
	pinned_vertices.clear();
	mesh_to_physics.resize(given_count);
	for (int i = 0; i < given_count; ++i) {
		mesh_to_physics[i] = given_to_final[i];
	}

	mass = 0.0f;
	for (float vertex_mass : cloth->masses) {
		mass += vertex_mass;
	}

	if (!in_space()) {
		_update_cloth_settings();
	}

	_try_rebuild();

	return given_to_final;
}

void JoltSoftBody3D::set_cloth_settings(const Dictionary &p_settings) {
	ERR_FAIL_NULL_MSG(cloth, vformat("Soft body '%s' wasn't built from data, so it takes no cloth settings.", to_string()));

	read_cloth_settings(p_settings, cloth->settings);

	_update_cloth_compliance();
	_update_cloth_settings();
	wake_up();
}

Dictionary JoltSoftBody3D::get_cloth_settings() const {
	ERR_FAIL_NULL_V_MSG(cloth, Dictionary(), vformat("Soft body '%s' wasn't built from data, so it has no cloth settings.", to_string()));

	const ClothSettings &cs = cloth->settings;
	Dictionary settings;
	settings["modulus"] = cs.modulus;
	settings["poisson"] = cs.poisson;
	settings["bend"] = cs.bend;
	settings["tether_slack"] = cs.tether_slack;
	settings["damping"] = cs.damping;
	settings["friction"] = cs.friction;
	settings["bounce"] = cs.bounce;
	settings["thickness"] = cs.thickness;
	settings["iterations"] = cs.iterations;
	settings["gravity_factor"] = cs.gravity_factor;
	settings["skin_multiplier"] = cs.skin_multiplier;
	settings["skin_constraints"] = cs.skin_constraints;
	settings["update_position"] = cs.update_position;
	settings["double_sided"] = cs.double_sided;
	return settings;
}

PackedVector3Array JoltSoftBody3D::get_vertices() const {
	ERR_FAIL_COND_V_MSG(!in_space(), PackedVector3Array(), vformat("Failed to read the vertices of '%s'. Doing so without a physics space is not supported when using Jolt Physics.", to_string()));

	const JPH::SoftBodyMotionProperties &motion_properties = static_cast<const JPH::SoftBodyMotionProperties &>(*jolt_body->GetMotionPropertiesUnchecked());
	const JPH::Array<JPH::SoftBodyVertex> &physics_vertices = motion_properties.GetVertices();
	const JPH::RMat44 com = jolt_body->GetCenterOfMassTransform();

	PackedVector3Array vertices;
	vertices.resize((int)physics_vertices.size());
	Vector3 *w = vertices.ptrw();
	for (const JPH::SoftBodyVertex &vertex : physics_vertices) {
		*w++ = to_godot(com * vertex.mPosition);
	}
	return vertices;
}

PackedVector3Array JoltSoftBody3D::get_velocities() const {
	ERR_FAIL_COND_V_MSG(!in_space(), PackedVector3Array(), vformat("Failed to read the velocities of '%s'. Doing so without a physics space is not supported when using Jolt Physics.", to_string()));

	const JPH::SoftBodyMotionProperties &motion_properties = static_cast<const JPH::SoftBodyMotionProperties &>(*jolt_body->GetMotionPropertiesUnchecked());
	const JPH::Array<JPH::SoftBodyVertex> &physics_vertices = motion_properties.GetVertices();
	const JPH::RMat44 com = jolt_body->GetCenterOfMassTransform();

	PackedVector3Array velocities;
	velocities.resize((int)physics_vertices.size());
	Vector3 *w = velocities.ptrw();
	for (const JPH::SoftBodyVertex &vertex : physics_vertices) {
		*w++ = to_godot(com.Multiply3x3(vertex.mVelocity));
	}
	return velocities;
}

float JoltSoftBody3D::get_volume() const {
	ERR_FAIL_COND_V_MSG(!in_space(), 0.0f, vformat("Failed to read the volume of '%s'. Doing so without a physics space is not supported when using Jolt Physics.", to_string()));

	const JPH::SoftBodyMotionProperties &motion_properties = static_cast<const JPH::SoftBodyMotionProperties &>(*jolt_body->GetMotionPropertiesUnchecked());
	return motion_properties.GetVolume();
}

float JoltSoftBody3D::get_rest_volume() const {
	return cloth != nullptr ? cloth->rest_volume : 0.0f;
}

PackedByteArray JoltSoftBody3D::save_state() const {
	ERR_FAIL_COND_V_MSG(!in_space(), PackedByteArray(), vformat("Failed to save the state of '%s'. Doing so without a physics space is not supported when using Jolt Physics.", to_string()));

	const JPH::SoftBodyMotionProperties &motion_properties = static_cast<const JPH::SoftBodyMotionProperties &>(*jolt_body->GetMotionPropertiesUnchecked());

	JPH::StateRecorderImpl recorder;
	recorder.Write(jolt_body->GetPosition());
	recorder.Write(jolt_body->GetRotation());
	motion_properties.SaveState(recorder);

	const std::string data = recorder.GetData();
	PackedByteArray state;
	state.resize((int)data.size());
	memcpy(state.ptrw(), data.data(), data.size());
	return state;
}

bool JoltSoftBody3D::restore_state(const PackedByteArray &p_state) {
	ERR_FAIL_COND_V_MSG(!in_space(), false, vformat("Failed to restore the state of '%s'. Doing so without a physics space is not supported when using Jolt Physics.", to_string()));

	JPH::SoftBodyMotionProperties &motion_properties = static_cast<JPH::SoftBodyMotionProperties &>(*jolt_body->GetMotionPropertiesUnchecked());

	JPH::StateRecorderImpl recorder;
	recorder.WriteBytes(p_state.ptr(), p_state.size());
	recorder.Rewind();

	JPH::RVec3 position;
	JPH::Quat rotation;
	recorder.Read(position);
	recorder.Read(rotation);
	motion_properties.RestoreState(recorder);

	if (recorder.IsFailed()) {
		return false;
	}

	space->get_body_iface().SetPositionAndRotation(jolt_body->GetID(), position, rotation, JPH::EActivation::DontActivate);
	wake_up();
	return true;
}

void JoltSoftBody3D::add_collision_exception(const RID &p_excepted_body) {
	exceptions.push_back(p_excepted_body);

	_exceptions_changed();
}

void JoltSoftBody3D::remove_collision_exception(const RID &p_excepted_body) {
	exceptions.erase(p_excepted_body);

	_exceptions_changed();
}

bool JoltSoftBody3D::has_collision_exception(const RID &p_excepted_body) const {
	return exceptions.find(p_excepted_body) >= 0;
}

void JoltSoftBody3D::add_area(JoltArea3D *p_area) {
	int i = 0;
	for (; i < (int)areas.size(); i++) {
		if (p_area->get_priority() > areas[i]->get_priority()) {
			break;
		}
	}

	areas.insert(i, p_area);

	_areas_changed();
}

void JoltSoftBody3D::remove_area(JoltArea3D *p_area) {
	areas.erase(p_area);

	_areas_changed();
}

bool JoltSoftBody3D::can_interact_with(const JoltBody3D &p_other) const {
	return (can_collide_with(p_other) || p_other.can_collide_with(*this)) && !has_collision_exception(p_other.get_rid()) && !p_other.has_collision_exception(rid);
}

bool JoltSoftBody3D::can_interact_with(const JoltSoftBody3D &p_other) const {
	return (can_collide_with(p_other) || p_other.can_collide_with(*this)) && !has_collision_exception(p_other.get_rid()) && !p_other.has_collision_exception(rid);
}

bool JoltSoftBody3D::can_interact_with(const JoltArea3D &p_other) const {
	return p_other.can_interact_with(*this);
}

Vector3 JoltSoftBody3D::get_velocity_at_position(const Vector3 &p_position) const {
	return Vector3();
}

void JoltSoftBody3D::pre_step(float p_step) {
	if (cloth != nullptr) {
		JPH::SoftBodyMotionProperties &motion_properties = static_cast<JPH::SoftBodyMotionProperties &>(*jolt_body->GetMotionPropertiesUnchecked());
		motion_properties.SetGravityOverride(to_jolt(_compute_gravity()));
		return;
	}

	if (p_step != compliance_step) {
		_update_compliance(p_step);
	}

	_apply_environmental_forces(p_step);
}

void JoltSoftBody3D::set_mesh(const RID &p_mesh) {
	if (unlikely(mesh == p_mesh)) {
		return;
	}

	mesh = p_mesh;

	if (cloth != nullptr) {
		memdelete(cloth);
		cloth = nullptr;
	}

	_mesh_changed();
}

bool JoltSoftBody3D::is_sleeping() const {
	if (!in_space()) {
		return false;
	} else {
		return !jolt_body->IsActive();
	}
}

void JoltSoftBody3D::apply_vertex_impulse(int p_index, const Vector3 &p_impulse) {
	ERR_FAIL_COND_MSG(!in_space(), vformat("Failed to apply impulse to '%s'. Doing so without a physics space is not supported when using Jolt Physics. If this relates to a node, try adding the node to a scene tree first.", to_string()));

	ERR_FAIL_INDEX(p_index, (int)mesh_to_physics.size());
	const int physics_index = mesh_to_physics[p_index];
	ERR_FAIL_COND_MSG(physics_index < 0, vformat("Soft body vertex %d was not used by a face and has been omitted for '%s'. No impulse can be applied.", p_index, to_string()));
	ERR_FAIL_COND_MSG(pinned_vertices.has(p_index), vformat("Failed to apply impulse to point at index %d for '%s'. Point was found to be pinned.", p_index, to_string()));

	JPH::SoftBodyMotionProperties &motion_properties = static_cast<JPH::SoftBodyMotionProperties &>(*jolt_body->GetMotionPropertiesUnchecked());

	JPH::Array<JPH::SoftBodyVertex> &physics_vertices = motion_properties.GetVertices();
	JPH::SoftBodyVertex &physics_vertex = physics_vertices[physics_index];

	physics_vertex.mVelocity += to_jolt(p_impulse) * physics_vertex.mInvMass;

	_motion_changed();
}

void JoltSoftBody3D::apply_vertex_force(int p_index, const Vector3 &p_force) {
	ERR_FAIL_COND_MSG(!in_space(), vformat("Failed to apply force to '%s'. Doing so without a physics space is not supported when using Jolt Physics. If this relates to a node, try adding the node to a scene tree first.", to_string()));

	apply_vertex_impulse(p_index, p_force * space->get_last_step());
}

void JoltSoftBody3D::apply_central_impulse(const Vector3 &p_impulse) {
	ERR_FAIL_COND_MSG(!in_space(), vformat("Failed to apply central impulse to '%s'. Doing so without a physics space is not supported when using Jolt Physics. If this relates to a node, try adding the node to a scene tree first.", to_string()));

	JPH::SoftBodyMotionProperties &motion_properties = static_cast<JPH::SoftBodyMotionProperties &>(*jolt_body->GetMotionPropertiesUnchecked());
	JPH::Array<JPH::SoftBodyVertex> &physics_vertices = motion_properties.GetVertices();

	const JPH::Vec3 impulse = to_jolt(p_impulse) / physics_vertices.size();

	for (JPH::SoftBodyVertex &physics_vertex : physics_vertices) {
		if (physics_vertex.mInvMass > 0.0f) {
			physics_vertex.mVelocity += impulse * physics_vertex.mInvMass;
		}
	}

	_motion_changed();
}

void JoltSoftBody3D::apply_central_force(const Vector3 &p_force) {
	ERR_FAIL_COND_MSG(!in_space(), vformat("Failed to apply central force to '%s'. Doing so without a physics space is not supported when using Jolt Physics. If this relates to a node, try adding the node to a scene tree first.", to_string()));

	jolt_body->AddForce(to_jolt(p_force));

	_motion_changed();
}

void JoltSoftBody3D::set_is_sleeping(bool p_enabled) {
	if (!in_space()) {
		return;
	}

	space->set_is_object_sleeping(jolt_body->GetID(), p_enabled);
}

bool JoltSoftBody3D::is_sleep_allowed() const {
	if (!in_space()) {
		return jolt_settings->mAllowSleeping;
	} else {
		return jolt_body->GetAllowSleeping();
	}
}

void JoltSoftBody3D::set_is_sleep_allowed(bool p_enabled) {
	if (!in_space()) {
		jolt_settings->mAllowSleeping = p_enabled;
	} else {
		jolt_body->SetAllowSleeping(p_enabled);
	}
}

void JoltSoftBody3D::set_simulation_precision(int p_precision) {
	if (unlikely(simulation_precision == p_precision)) {
		return;
	}

	simulation_precision = MAX(p_precision, 0);

	_simulation_precision_changed();
}

void JoltSoftBody3D::set_mass(float p_mass) {
	ERR_FAIL_COND(p_mass <= 0.0); // A mass of zero would result in infinite inverse mass.

	if (unlikely(mass == p_mass)) {
		return;
	}

	mass = p_mass;

	_mass_changed();
}

float JoltSoftBody3D::get_stiffness_coefficient() const {
	return stiffness_coefficient;
}

void JoltSoftBody3D::set_stiffness_coefficient(float p_coefficient) {
	const float coefficient = CLAMP(p_coefficient, 0.0f, 1.0f);
	if (unlikely(stiffness_coefficient == coefficient)) {
		return;
	}

	stiffness_coefficient = coefficient;

	_stiffness_changed();
}

float JoltSoftBody3D::get_shrinking_factor() const {
	return shrinking_factor;
}

void JoltSoftBody3D::set_shrinking_factor(float p_shrinking_factor) {
	if (unlikely(shrinking_factor == p_shrinking_factor)) {
		return;
	}

	shrinking_factor = p_shrinking_factor;

	_shrinking_changed();
}

void JoltSoftBody3D::set_pressure(float p_pressure) {
	if (unlikely(pressure == p_pressure)) {
		return;
	}

	pressure = MAX(p_pressure, 0.0f);

	_pressure_changed();
}

void JoltSoftBody3D::set_linear_damping(float p_damping) {
	if (unlikely(linear_damping == p_damping)) {
		return;
	}

	linear_damping = MAX(p_damping, 0.0f);

	_damping_changed();
}

float JoltSoftBody3D::get_drag() const {
	// Drag is not a thing in Jolt, and not supported by Godot Physics either.
	return 0.0f;
}

void JoltSoftBody3D::set_drag(float p_drag) {
	// Drag is not a thing in Jolt, and not supported by Godot Physics either.
}

Variant JoltSoftBody3D::get_state(PS3DE::BodyState p_state) const {
	switch (p_state) {
		case PS3DE::BODY_STATE_TRANSFORM: {
			return get_transform();
		}
		case PS3DE::BODY_STATE_LINEAR_VELOCITY: {
			ERR_FAIL_V_MSG(Variant(), "Linear velocity is not supported for soft bodies.");
		}
		case PS3DE::BODY_STATE_ANGULAR_VELOCITY: {
			ERR_FAIL_V_MSG(Variant(), "Angular velocity is not supported for soft bodies.");
		}
		case PS3DE::BODY_STATE_SLEEPING: {
			return is_sleeping();
		}
		case PS3DE::BODY_STATE_CAN_SLEEP: {
			return is_sleep_allowed();
		}
		default: {
			ERR_FAIL_V_MSG(Variant(), vformat("Unhandled body state: '%d'. This should not happen. Please report this.", p_state));
		}
	}
}

void JoltSoftBody3D::set_state(PS3DE::BodyState p_state, const Variant &p_value) {
	switch (p_state) {
		case PS3DE::BODY_STATE_TRANSFORM: {
			set_transform(p_value);
		} break;
		case PS3DE::BODY_STATE_LINEAR_VELOCITY: {
			ERR_FAIL_MSG("Linear velocity is not supported for soft bodies.");
		} break;
		case PS3DE::BODY_STATE_ANGULAR_VELOCITY: {
			ERR_FAIL_MSG("Angular velocity is not supported for soft bodies.");
		} break;
		case PS3DE::BODY_STATE_SLEEPING: {
			set_is_sleeping(p_value);
		} break;
		case PS3DE::BODY_STATE_CAN_SLEEP: {
			set_is_sleep_allowed(p_value);
		} break;
		default: {
			ERR_FAIL_MSG(vformat("Unhandled body state: '%d'. This should not happen. Please report this.", p_state));
		} break;
	}
}

Transform3D JoltSoftBody3D::get_transform() const {
	if (cloth != nullptr) {
		if (!in_space()) {
			return cloth->placement;
		}

		return Transform3D(to_godot(jolt_body->GetRotation()), to_godot(jolt_body->GetPosition()));
	}

	// Since any transform gets baked into the vertices anyway we can just return identity here.
	return Transform3D();
}

void JoltSoftBody3D::set_transform(const Transform3D &p_transform) {
	if (cloth != nullptr) {
		const Transform3D placement = p_transform.orthonormalized();
		if (!in_space()) {
			cloth->placement = placement;
			return;
		}

		space->get_body_iface().SetPositionAndRotation(jolt_body->GetID(), to_jolt_r(placement.origin), to_jolt(placement.basis), JPH::EActivation::DontActivate);
		_transform_changed();
		return;
	}

	ERR_FAIL_COND_MSG(!in_space(), vformat("Failed to set transform for '%s'. Doing so without a physics space is not supported when using Jolt Physics. If this relates to a node, try adding the node to a scene tree first.", to_string()));

	// For whatever reason this has to be interpreted as a relative global-space transform rather than an absolute one,
	// because `SoftBody3D` will immediately upon entering the scene tree set itself to be top-level and also set its
	// transform to be identity, while still expecting to stay in its original position.
	//
	// We also discard any scaling, since we have no way of scaling the actual edge lengths.
	const JPH::Mat44 relative_transform = to_jolt(p_transform.orthonormalized());

	// The translation delta goes to the body's position to avoid vertices getting too far away from it.
	JPH::BodyInterface &body_iface = space->get_body_iface();
	body_iface.SetPosition(jolt_body->GetID(), jolt_body->GetPosition() + relative_transform.GetTranslation(), JPH::EActivation::DontActivate);

	// The rotation difference goes to the vertices. We also reset the velocity of these vertices.
	JPH::SoftBodyMotionProperties &motion_properties = static_cast<JPH::SoftBodyMotionProperties &>(*jolt_body->GetMotionPropertiesUnchecked());
	JPH::Array<JPH::SoftBodyVertex> &physics_vertices = motion_properties.GetVertices();

	for (JPH::SoftBodyVertex &vertex : physics_vertices) {
		vertex.mPosition = vertex.mPreviousPosition = relative_transform.Multiply3x3(vertex.mPosition);
		vertex.mVelocity = JPH::Vec3::sZero();
	}

	_transform_changed();
}

AABB JoltSoftBody3D::get_bounds() const {
	ERR_FAIL_COND_V_MSG(!in_space(), AABB(), vformat("Failed to retrieve world bounds of '%s'. Doing so without a physics space is not supported when using Jolt Physics. If this relates to a node, try adding the node to a scene tree first.", to_string()));
	return to_godot(jolt_body->GetWorldSpaceBounds());
}

void JoltSoftBody3D::update_rendering_server(PhysicsServer3DRenderingServerHandler *p_rendering_server_handler) {
	// Ideally we would emit an actual error here, but that would spam the logs to the point where the actual cause will be drowned out.
	if (unlikely(!in_space())) {
		return;
	}

	const JPH::SoftBodyMotionProperties &motion_properties = static_cast<const JPH::SoftBodyMotionProperties &>(*jolt_body->GetMotionPropertiesUnchecked());

	typedef JPH::SoftBodyMotionProperties::Vertex SoftBodyVertex;
	typedef JPH::SoftBodyMotionProperties::Face SoftBodyFace;

	const JPH::Array<SoftBodyVertex> &physics_vertices = motion_properties.GetVertices();
	const JPH::Array<SoftBodyFace> &physics_faces = motion_properties.GetFaces();

	const int physics_vertex_count = (int)physics_vertices.size();

	normals.clear();
	normals.resize(physics_vertex_count);

	// Compute vertex normals using smooth-shading:
	// Each vertex should use the average normal of all faces it is a part of.
	// Iterate over each face, and add the face normal to each of the face vertices.
	// By the end of the loop, each vertex normal will be the sum of all face normals it belongs to.
	for (const SoftBodyFace &physics_face : physics_faces) {
		// Jolt uses a different winding order, so we swap the indices to account for that.

		const uint32_t i0 = physics_face.mVertex[2];
		const uint32_t i1 = physics_face.mVertex[1];
		const uint32_t i2 = physics_face.mVertex[0];

		const Vector3 v0 = to_godot(physics_vertices[i0].mPosition);
		const Vector3 v1 = to_godot(physics_vertices[i1].mPosition);
		const Vector3 v2 = to_godot(physics_vertices[i2].mPosition);

		const Vector3 normal = (v2 - v0).cross(v1 - v0).normalized();

		normals[i0] += normal;
		normals[i1] += normal;
		normals[i2] += normal;
	}
	// Normalize the vertex normals to have length 1.0
	for (Vector3 &n : normals) {
		real_t len = n.length();
		// Some normals may have length 0 if the face was degenerate,
		// so don't divide by zero.
		if (len > CMP_EPSILON) {
			n /= len;
		}
	}

	const int mesh_vertex_count = mesh_to_physics.size();
	const JPH::RVec3 body_position = jolt_body->GetCenterOfMassPosition();

	for (int i = 0; i < mesh_vertex_count; ++i) {
		const int physics_index = mesh_to_physics[i];
		if (physics_index >= 0) {
			const Vector3 vertex = to_godot(body_position + physics_vertices[(size_t)physics_index].mPosition);
			const Vector3 normal = normals[(uint32_t)physics_index];

			p_rendering_server_handler->set_vertex(i, vertex);
			p_rendering_server_handler->set_normal(i, normal);
		}
	}

	p_rendering_server_handler->set_aabb(get_bounds());
}

Vector3 JoltSoftBody3D::get_vertex_position(int p_index) {
	ERR_FAIL_COND_V_MSG(!in_space(), Vector3(), vformat("Failed to retrieve point position for '%s'. Doing so without a physics space is not supported when using Jolt Physics. If this relates to a node, try adding the node to a scene tree first.", to_string()));

	ERR_FAIL_INDEX_V(p_index, (int)mesh_to_physics.size(), Vector3());
	const int physics_index = mesh_to_physics[p_index];
	ERR_FAIL_COND_V_MSG(physics_index < 0, Vector3(), vformat("Soft body vertex %d was not used by a face and has been omitted for '%s'. Position cannot be returned.", p_index, to_string()));

	const JPH::SoftBodyMotionProperties &motion_properties = static_cast<const JPH::SoftBodyMotionProperties &>(*jolt_body->GetMotionPropertiesUnchecked());
	const JPH::Array<JPH::SoftBodyVertex> &physics_vertices = motion_properties.GetVertices();
	const JPH::SoftBodyVertex &physics_vertex = physics_vertices[physics_index];

	return to_godot(jolt_body->GetCenterOfMassPosition() + physics_vertex.mPosition);
}

void JoltSoftBody3D::set_vertex_position(int p_index, const Vector3 &p_position) {
	ERR_FAIL_COND_MSG(!in_space(), vformat("Failed to set point position for '%s'. Doing so without a physics space is not supported when using Jolt Physics. If this relates to a node, try adding the node to a scene tree first.", to_string()));

	ERR_FAIL_INDEX(p_index, (int)mesh_to_physics.size());
	const int physics_index = mesh_to_physics[p_index];
	ERR_FAIL_COND_MSG(physics_index < 0, vformat("Soft body vertex %d was not used by a face and has been omitted for '%s'. Position cannot be set.", p_index, to_string()));

	JPH::SoftBodyMotionProperties &motion_properties = static_cast<JPH::SoftBodyMotionProperties &>(*jolt_body->GetMotionPropertiesUnchecked());
	JPH::Array<JPH::SoftBodyVertex> &physics_vertices = motion_properties.GetVertices();
	JPH::SoftBodyVertex &physics_vertex = physics_vertices[physics_index];

	const JPH::RVec3 center_of_mass = jolt_body->GetCenterOfMassPosition();
	physics_vertex.mPosition = JPH::Vec3(to_jolt_r(p_position) - center_of_mass);

	_vertices_changed();
}

void JoltSoftBody3D::pin_vertex(int p_index) {
	pinned_vertices.insert(p_index);

	_pins_changed();
}

void JoltSoftBody3D::unpin_vertex(int p_index) {
	pinned_vertices.erase(p_index);

	_pins_changed();
}

void JoltSoftBody3D::unpin_all_vertices() {
	pinned_vertices.clear();

	_pins_changed();
}

bool JoltSoftBody3D::is_vertex_pinned(int p_index) const {
	ERR_FAIL_COND_V_MSG(!in_space(), false, vformat("Failed retrieve pin status of point for '%s'. Doing so without a physics space is not supported when using Jolt Physics. If this relates to a node, try adding the node to a scene tree first.", to_string()));

	ERR_FAIL_INDEX_V(p_index, (int)mesh_to_physics.size(), false);

	return pinned_vertices.has(p_index);
}
