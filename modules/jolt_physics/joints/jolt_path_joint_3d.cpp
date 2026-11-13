/**************************************************************************/
/*  jolt_path_joint_3d.cpp                                                */
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

#include "jolt_path_joint_3d.h"

#include "../misc/jolt_type_conversions.h"
#include "../objects/jolt_body_3d.h"
#include "../spaces/jolt_space_3d.h"

#include <Jolt/Physics/Constraints/PathConstraint.h>

namespace {

Vector3 path_normal_for(const Vector3 &p_tangent) {
	const Vector3 tangent = p_tangent.normalized();
	const Vector3 up = Math::abs(tangent.y) < 0.99f ? Vector3(0, 1, 0) : Vector3(1, 0, 0);
	return (up - tangent * up.dot(tangent)).normalized();
}

} // namespace

JoltPathJoint3D::JoltPathJoint3D(const JoltJoint3D &p_old_joint, JoltBody3D *p_holder, const Transform3D &p_path_frame, const PackedVector3Array &p_points, const PackedVector3Array &p_tangents, const PackedVector3Array &p_normals, bool p_looping, JoltBody3D *p_follower, const Vector3 &p_follower_point, int p_rotation) :
		JoltJoint3D(p_old_joint, p_holder, p_follower, p_path_frame.orthonormalized(), Transform3D({}, p_follower_point)),
		rotation(p_rotation) {
	const int count = p_points.size();

	path = new JPH::PathConstraintPathHermite();
	path->SetIsLooping(p_looping);

	for (int i = 0; i < count; ++i) {
		Vector3 tangent;

		if (i < p_tangents.size()) {
			tangent = p_tangents[i];
		} else if (p_looping) {
			tangent = (p_points[(i + 1) % count] - p_points[(i + count - 1) % count]) * 0.5f;
		} else {
			tangent = p_points[MIN(i + 1, count - 1)] - p_points[MAX(i - 1, 0)];
			tangent *= (i == 0 || i == count - 1) ? 1.0f : 0.5f;
		}

		const Vector3 normal = i < p_normals.size() ? p_normals[i] : path_normal_for(tangent);

		path->AddPoint(to_jolt(p_points[i]), to_jolt(tangent), to_jolt(normal));
	}

	rebuild();
}

Transform3D JoltPathJoint3D::_path_to_world() const {
	return body_a != nullptr ? body_a->get_transform_unscaled() * local_ref_a : local_ref_a;
}

float JoltPathJoint3D::_closest_fraction() const {
	const Vector3 follower = body_b->get_transform_unscaled().xform(local_ref_b.origin);
	return path->GetClosestPoint(to_jolt(_path_to_world().xform_inv(follower)), 0.0f);
}

JPH::Constraint *JoltPathJoint3D::_build_path(JPH::Body *p_jolt_body_a, JPH::Body *p_jolt_body_b) const {
	JPH::PathConstraintSettings constraint_settings;
	constraint_settings.mPath = path;
	constraint_settings.mPathPosition = to_jolt(local_ref_a.origin);
	constraint_settings.mPathRotation = to_jolt(local_ref_a.basis);
	constraint_settings.mPathFraction = fraction;
	constraint_settings.mMaxFrictionForce = friction;
	constraint_settings.mPositionMotorSettings = motor_settings;
	constraint_settings.mRotationConstraintType = (JPH::EPathRotationConstraintType)rotation;

	if (p_jolt_body_a == nullptr) {
		return constraint_settings.Create(JPH::Body::sFixedToWorld, *p_jolt_body_b);
	}

	return constraint_settings.Create(*p_jolt_body_a, *p_jolt_body_b);
}

float JoltPathJoint3D::_length_per_fraction(float p_fraction) const {
	const float max_fraction = path->GetPathMaxFraction();
	const float span = 0.001f;
	const float low = path->IsLooping() ? p_fraction - span : MAX(p_fraction - span, 0.0f);
	const float high = path->IsLooping() ? p_fraction + span : MIN(p_fraction + span, max_fraction);

	JPH::Vec3 point_low, point_high, tangent, normal, binormal;
	path->GetPointOnPath(low, point_low, tangent, normal, binormal);
	path->GetPointOnPath(high, point_high, tangent, normal, binormal);

	return high > low ? (point_high - point_low).Length() / (high - low) : 1.0f;
}

void JoltPathJoint3D::_update_motor() {
	JPH::PathConstraint *constraint = static_cast<JPH::PathConstraint *>(jolt_ref.GetPtr());
	if (constraint == nullptr) {
		return;
	}

	constraint->SetMaxFrictionForce(friction);

	JPH::MotorSettings settings = motor_settings;
	if (motor_mode == 2) {
		settings.mSpringSettings.mFrequency = motor_frequency * Math::sqrt(MAX(_length_per_fraction(motor_target), 1.0e-6f));
	}
	constraint->GetPositionMotorSettings() = settings;

	switch (motor_mode) {
		case 1: {
			constraint->SetTargetVelocity(motor_target);
			constraint->SetPositionMotorState(JPH::EMotorState::Velocity);
		} break;
		case 2: {
			constraint->SetTargetPathFraction(motor_target);
			constraint->SetPositionMotorState(JPH::EMotorState::Position);
		} break;
		default: {
			constraint->SetPositionMotorState(JPH::EMotorState::Off);
		} break;
	}
}

float JoltPathJoint3D::get_fraction() const {
	const JPH::PathConstraint *constraint = static_cast<const JPH::PathConstraint *>(jolt_ref.GetPtr());
	return constraint != nullptr ? constraint->GetPathFraction() : fraction;
}

Transform3D JoltPathJoint3D::get_point(float p_fraction) const {
	JPH::Vec3 point, tangent, normal, binormal;
	path->GetPointOnPath(p_fraction, point, tangent, normal, binormal);

	const Transform3D on_path(Basis(to_godot(tangent), to_godot(binormal), to_godot(normal)), to_godot(point));
	return _path_to_world() * on_path;
}

void JoltPathJoint3D::set_friction(float p_force) {
	ERR_FAIL_COND_MSG(!(p_force >= 0.0f), "A path's friction is a force of zero or more.");

	friction = p_force;
	_update_motor();
	_wake_up_bodies();
}

void JoltPathJoint3D::set_motor(int p_mode, float p_target) {
	ERR_FAIL_COND_MSG(p_mode < 0 || p_mode > 2, "A path's motor is off (0), driving to a speed along it (1) or to a fraction of it (2).");
	ERR_FAIL_COND_MSG(p_mode == 2 && !path->IsLooping() && (p_target < 0.0f || p_target > path->GetPathMaxFraction()), "A path's motor drives to a fraction on the path.");
	ERR_FAIL_COND_MSG(!Math::is_finite(p_target), "A path's motor target is finite.");

	motor_mode = p_mode;
	motor_target = p_target;
	_update_motor();
	_wake_up_bodies();
}

void JoltPathJoint3D::set_motor_spring(float p_frequency, float p_damping, float p_max_force) {
	ERR_FAIL_COND_MSG(!(p_frequency > 0.0f) || !(p_damping >= 0.0f) || !(p_max_force >= 0.0f), "A path's motor spring has a frequency above zero, and a damping and a force limit of zero or more.");

	motor_settings.mSpringSettings.mMode = JPH::ESpringMode::FrequencyAndDamping;
	motor_settings.mSpringSettings.mFrequency = p_frequency;
	motor_frequency = p_frequency;
	motor_settings.mSpringSettings.mDamping = p_damping;
	motor_settings.SetForceLimit(p_max_force);
	_update_motor();
	_wake_up_bodies();
}

float JoltPathJoint3D::get_applied_force() const {
	const JPH::PathConstraint *constraint = static_cast<const JPH::PathConstraint *>(jolt_ref.GetPtr());
	if (constraint == nullptr) {
		return 0.0f;
	}

	JoltSpace3D *space = get_space();
	ERR_FAIL_NULL_V(space, 0.0f);

	const float last_step = space->get_last_step();
	if (unlikely(last_step == 0.0f)) {
		return 0.0f;
	}

	const JPH::Vector<2> position = constraint->GetTotalLambdaPosition();
	const JPH::Vec3 total_lambda(position[0], position[1], constraint->GetTotalLambdaPositionLimits() + constraint->GetTotalLambdaMotor());
	return total_lambda.Length() / last_step;
}

float JoltPathJoint3D::get_applied_torque() const {
	const JPH::PathConstraint *constraint = static_cast<const JPH::PathConstraint *>(jolt_ref.GetPtr());
	if (constraint == nullptr) {
		return 0.0f;
	}

	JoltSpace3D *space = get_space();
	ERR_FAIL_NULL_V(space, 0.0f);

	const float last_step = space->get_last_step();
	if (unlikely(last_step == 0.0f)) {
		return 0.0f;
	}

	const JPH::Vector<2> hinge = constraint->GetTotalLambdaRotationHinge();
	const float hinge_length = Math::sqrt(hinge[0] * hinge[0] + hinge[1] * hinge[1]);
	return (hinge_length + constraint->GetTotalLambdaRotation().Length()) / last_step;
}

void JoltPathJoint3D::rebuild() {
	if (jolt_ref != nullptr) {
		fraction = static_cast<JPH::PathConstraint *>(jolt_ref.GetPtr())->GetPathFraction();
		has_fraction = true;
	}

	destroy();

	JoltSpace3D *space = get_space();
	if (space == nullptr || body_b == nullptr) {
		return;
	}

	JPH::Body *jolt_body_a = body_a != nullptr ? body_a->get_jolt_body() : nullptr;
	JPH::Body *jolt_body_b = body_b->get_jolt_body();
	ERR_FAIL_NULL(jolt_body_b);

	if (!has_fraction) {
		fraction = _closest_fraction();
		has_fraction = true;
	}

	jolt_ref = _build_path(jolt_body_a, jolt_body_b);

	space->add_joint(this);

	_update_joint();
	_update_motor();
}
