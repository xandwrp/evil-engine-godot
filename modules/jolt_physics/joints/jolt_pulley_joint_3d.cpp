/**************************************************************************/
/*  jolt_pulley_joint_3d.cpp                                              */
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

#include "jolt_pulley_joint_3d.h"

#include "../misc/jolt_type_conversions.h"
#include "../objects/jolt_body_3d.h"
#include "../spaces/jolt_space_3d.h"

#include <Jolt/Physics/Constraints/PulleyConstraint.h>

namespace {

Vector3 world_point(const JoltBody3D *p_body, const Vector3 &p_local) {
	return p_body != nullptr ? p_body->get_transform_scaled().xform(p_local) : p_local;
}

} // namespace

JPH::Constraint *JoltPulleyJoint3D::_build_pulley(JPH::Body *p_jolt_body_a, JPH::Body *p_jolt_body_b, const Transform3D &p_shifted_ref_a, const Transform3D &p_shifted_ref_b) const {
	JPH::PulleyConstraintSettings constraint_settings;
	constraint_settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	constraint_settings.mBodyPoint1 = to_jolt_r(p_shifted_ref_a.origin);
	constraint_settings.mBodyPoint2 = to_jolt_r(p_shifted_ref_b.origin);
	constraint_settings.mFixedPoint1 = to_jolt_r(fixed_a);
	constraint_settings.mFixedPoint2 = to_jolt_r(fixed_b);
	constraint_settings.mRatio = ratio;
	constraint_settings.mMinLength = min_length;
	constraint_settings.mMaxLength = max_length;

	if (p_jolt_body_a == nullptr) {
		return constraint_settings.Create(JPH::Body::sFixedToWorld, *p_jolt_body_b);
	} else if (p_jolt_body_b == nullptr) {
		return constraint_settings.Create(*p_jolt_body_a, JPH::Body::sFixedToWorld);
	} else {
		return constraint_settings.Create(*p_jolt_body_a, *p_jolt_body_b);
	}
}

JoltPulleyJoint3D::JoltPulleyJoint3D(const JoltJoint3D &p_old_joint, JoltBody3D *p_body_a, JoltBody3D *p_body_b, const Vector3 &p_local_a, const Vector3 &p_local_b, const Vector3 &p_fixed_a, const Vector3 &p_fixed_b, float p_ratio) :
		JoltJoint3D(p_old_joint, p_body_a, p_body_b, Transform3D({}, p_local_a), Transform3D({}, p_local_b)),
		fixed_a(p_fixed_a),
		fixed_b(p_fixed_b),
		ratio(p_ratio) {
	if (body_a != p_body_a) {
		SWAP(fixed_a, fixed_b);
	}

	const float length = get_current_length();
	min_length = 0.0f;
	max_length = length;

	rebuild();
}

void JoltPulleyJoint3D::set_length(float p_min_length, float p_max_length) {
	ERR_FAIL_COND_MSG(!(p_min_length >= 0.0f) || !(p_min_length <= p_max_length), "A pulley's lengths are from zero up, the least first.");

	min_length = p_min_length;
	max_length = p_max_length;

	if (JPH::PulleyConstraint *constraint = static_cast<JPH::PulleyConstraint *>(jolt_ref.GetPtr())) {
		constraint->SetLength(min_length, max_length);
	}

	_wake_up_bodies();
}

float JoltPulleyJoint3D::get_current_length() const {
	return (float)(world_point(body_a, local_ref_a.origin).distance_to(fixed_a) + ratio * world_point(body_b, local_ref_b.origin).distance_to(fixed_b));
}

float JoltPulleyJoint3D::get_applied_force() const {
	JPH::PulleyConstraint *constraint = static_cast<JPH::PulleyConstraint *>(jolt_ref.GetPtr());
	if (constraint == nullptr) {
		return 0.0f;
	}

	JoltSpace3D *space = get_space();
	ERR_FAIL_NULL_V(space, 0.0f);

	const float last_step = space->get_last_step();
	if (unlikely(last_step == 0.0f)) {
		return 0.0f;
	}

	return Math::abs(constraint->GetTotalLambdaPosition()) / last_step;
}

void JoltPulleyJoint3D::rebuild() {
	destroy();

	JoltSpace3D *space = get_space();
	if (space == nullptr) {
		return;
	}

	JPH::Body *jolt_body_a = body_a != nullptr ? body_a->get_jolt_body() : nullptr;
	JPH::Body *jolt_body_b = body_b != nullptr ? body_b->get_jolt_body() : nullptr;
	ERR_FAIL_COND(jolt_body_a == nullptr && jolt_body_b == nullptr);

	Transform3D shifted_ref_a;
	Transform3D shifted_ref_b;

	_shift_reference_frames(Vector3(), Vector3(), shifted_ref_a, shifted_ref_b);

	jolt_ref = _build_pulley(jolt_body_a, jolt_body_b, shifted_ref_a, shifted_ref_b);

	space->add_joint(this);

	_update_joint();
}
