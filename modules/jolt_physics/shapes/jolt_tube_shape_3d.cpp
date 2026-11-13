/**************************************************************************/
/*  jolt_tube_shape_3d.cpp                                                */
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

#include "jolt_tube_shape_3d.h"

#include "../misc/jolt_type_conversions.h"

#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>

namespace {

Basis basis_along(const Vector3 &p_axis, int p_column) {
	const Vector3 axis = p_axis.normalized();
	const Vector3 helper = Math::abs(axis.y) < 0.99f ? Vector3(0, 1, 0) : Vector3(1, 0, 0);
	const Vector3 side = helper.cross(axis).normalized();
	const Vector3 other = axis.cross(side);

	Basis basis;
	if (p_column == Vector3::AXIS_Y) {
		basis.set_columns(side, axis, -other);
	} else {
		basis.set_columns(side, other, axis);
	}
	return basis;
}

} // namespace

Vector2 JoltTubeShape3D::cap_profile(float p_roundness, float p_t) {
	const float t = CLAMP(p_t, 0.0f, 1.0f) * Math::PI * 0.5f;
	if (p_roundness <= 0.0f) {
		return Vector2(p_t < 1.0f ? 1.0f : 0.0f, p_t > 0.0f ? 1.0f : 0.0f);
	}

	const float power = p_roundness;
	return Vector2(Math::pow(Math::cos(t), power), Math::pow(Math::sin(t), power));
}

JPH::ShapeRefC JoltTubeShape3D::_build_cap(const Vector3 &p_end, const Vector3 &p_outward, float p_radius, float p_roundness) const {
	const Basis frame = basis_along(p_outward, Vector3::AXIS_Z);

	JPH::Array<JPH::Vec3> hull;
	hull.reserve((CAP_RINGS + 1) * CAP_SIDES + 1);

	for (int ring = 0; ring <= CAP_RINGS; ++ring) {
		const Vector2 profile = cap_profile(p_roundness, (float)ring / CAP_RINGS);
		const float across = profile.x * p_radius;
		const float along = profile.y * p_radius;

		if (across <= 1.0e-5f) {
			hull.push_back(to_jolt(p_end + frame.get_column(Vector3::AXIS_Z) * along));
			continue;
		}

		for (int side = 0; side < CAP_SIDES; ++side) {
			const float angle = Math::TAU * side / CAP_SIDES;
			const Vector3 local(Math::cos(angle) * across, Math::sin(angle) * across, along);
			hull.push_back(to_jolt(p_end + frame.xform(local)));
		}
	}

	JPH::ConvexHullShapeSettings settings(hull, 0.0f);
	const JPH::ShapeSettings::ShapeResult result = settings.Create();
	ERR_FAIL_COND_V_MSG(result.HasError(), nullptr, vformat("Failed to build a tube's cap with %s. Jolt returned: '%s'. This shape belongs to %s.", to_string(), to_godot(result.GetError()), _owners_to_string()));

	return result.Get();
}

JPH::ShapeRefC JoltTubeShape3D::_build() const {
	const int count = points.size();
	ERR_FAIL_COND_V_MSG(count == 0 || radii.size() != count, nullptr, vformat("Failed to build a tube with %s. It needs a radius for each of its points. This shape belongs to %s.", to_string(), _owners_to_string()));

	for (const float radius : radii) {
		ERR_FAIL_COND_V_MSG(!(radius > 0.0f), nullptr, vformat("Failed to build a tube with %s. Each of its radii must be greater than 0. This shape belongs to %s.", to_string(), _owners_to_string()));
	}

	if (count == 1) {
		const JPH::SphereShapeSettings sphere(radii[0]);
		const JPH::RotatedTranslatedShapeSettings placed(to_jolt(points[0]), JPH::Quat::sIdentity(), sphere.Create().Get());
		return placed.Create().Get();
	}

	JPH::StaticCompoundShapeSettings compound;

	for (int i = 0; i + 1 < count; ++i) {
		const Vector3 bottom = points[i];
		const Vector3 top = points[i + 1];
		const float length = bottom.distance_to(top);

		JPH::ShapeRefC span;
		Vector3 center;
		Basis basis;

		if (length <= 1.0e-5f) {
			span = JPH::SphereShapeSettings(MAX(radii[i], radii[i + 1])).Create().Get();
			center = bottom;
		} else {
			const JPH::TaperedCapsuleShapeSettings settings(length * 0.5f, radii[i + 1], radii[i]);
			const JPH::ShapeSettings::ShapeResult result = settings.Create();
			ERR_FAIL_COND_V_MSG(result.HasError(), nullptr, vformat("Failed to build span %d of a tube with %s. Jolt returned: '%s'. This shape belongs to %s.", i, to_string(), to_godot(result.GetError()), _owners_to_string()));
			span = result.Get();
			center = (bottom + top) * 0.5f;
			basis = basis_along(top - bottom, Vector3::AXIS_Y);
		}

		compound.AddShape(to_jolt(center), to_jolt(basis), span);
	}

	const struct {
		int end;
		int inner;
		float roundness;
	} caps[2] = { { 0, 1, start_cap }, { count - 1, count - 2, end_cap } };

	for (const auto &cap : caps) {
		const Vector3 outward = points[cap.end] - points[cap.inner];
		if (cap.roundness >= 1.0f || outward.length() <= 1.0e-5f) {
			continue;
		}

		const JPH::ShapeRefC hull = _build_cap(points[cap.end], outward, radii[cap.end], cap.roundness);
		ERR_FAIL_NULL_V(hull, nullptr);
		compound.AddShape(JPH::Vec3::sZero(), JPH::Quat::sIdentity(), hull);
	}

	const JPH::ShapeSettings::ShapeResult result = compound.Create();
	ERR_FAIL_COND_V_MSG(result.HasError(), nullptr, vformat("Failed to build a tube with %s. Jolt returned: '%s'. This shape belongs to %s.", to_string(), to_godot(result.GetError()), _owners_to_string()));

	return result.Get();
}

Variant JoltTubeShape3D::get_data() const {
	Dictionary data;
	data["points"] = points;
	data["radii"] = radii;
	data["start_cap"] = start_cap;
	data["end_cap"] = end_cap;
	return data;
}

void JoltTubeShape3D::set_data(const Variant &p_data) {
	ERR_FAIL_COND_MSG(p_data.get_type() != Variant::DICTIONARY, "A tube's data is a Dictionary with its points, radii, start_cap and end_cap.");

	const Dictionary data = p_data;

	const Variant maybe_points = data.get("points", Variant());
	ERR_FAIL_COND_MSG(maybe_points.get_type() != Variant::PACKED_VECTOR3_ARRAY, "A tube's points are a PackedVector3Array.");

	const Variant maybe_radii = data.get("radii", Variant());
	ERR_FAIL_COND_MSG(maybe_radii.get_type() != Variant::PACKED_FLOAT32_ARRAY, "A tube's radii are a PackedFloat32Array, one per point.");

	const float new_start_cap = data.get("start_cap", 1.0f);
	const float new_end_cap = data.get("end_cap", 1.0f);
	ERR_FAIL_COND_MSG(!(new_start_cap >= 0.0f && new_start_cap <= 1.0f) || !(new_end_cap >= 0.0f && new_end_cap <= 1.0f), "A tube's caps are from 0, flat, to 1, round.");

	points = maybe_points;
	radii = maybe_radii;
	start_cap = new_start_cap;
	end_cap = new_end_cap;

	destroy();
}

AABB JoltTubeShape3D::get_aabb() const {
	AABB bounds;
	for (int i = 0; i < points.size() && i < radii.size(); ++i) {
		const AABB sphere(points[i] - Vector3(radii[i], radii[i], radii[i]), Vector3(radii[i], radii[i], radii[i]) * 2.0f);
		bounds = i == 0 ? sphere : bounds.merge(sphere);
	}
	return bounds;
}

String JoltTubeShape3D::to_string() const {
	return vformat("{points=%d start_cap=%f end_cap=%f}", points.size(), start_cap, end_cap);
}
