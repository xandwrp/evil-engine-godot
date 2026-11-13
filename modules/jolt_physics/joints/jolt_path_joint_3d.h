/**************************************************************************/
/*  jolt_path_joint_3d.h                                                  */
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

#include "jolt_joint_3d.h"

#include "core/variant/variant.h"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Constraints/MotorSettings.h>
#include <Jolt/Physics/Constraints/PathConstraintPathHermite.h>

class JoltPathJoint3D final : public JoltJoint3D {
	JPH::Ref<JPH::PathConstraintPathHermite> path;

	int rotation = 0;
	float fraction = 0.0f;
	bool has_fraction = false;
	float friction = 0.0f;

	int motor_mode = 0;
	float motor_target = 0.0f;
	JPH::MotorSettings motor_settings;
	float motor_frequency = 2.0f;

	float _length_per_fraction(float p_fraction) const;

	Transform3D _path_to_world() const;
	float _closest_fraction() const;
	void _update_motor();

	JPH::Constraint *_build_path(JPH::Body *p_jolt_body_a, JPH::Body *p_jolt_body_b) const;

public:
	JoltPathJoint3D(const JoltJoint3D &p_old_joint, JoltBody3D *p_holder, const Transform3D &p_path_frame, const PackedVector3Array &p_points, const PackedVector3Array &p_tangents, const PackedVector3Array &p_normals, bool p_looping, JoltBody3D *p_follower, const Vector3 &p_follower_point, int p_rotation);

	virtual JoltJointType get_jolt_type() const override { return JOLT_JOINT_TYPE_PATH; }

	float get_fraction() const;
	float get_max_fraction() const { return path->GetPathMaxFraction(); }
	bool is_looping() const { return path->IsLooping(); }
	Transform3D get_point(float p_fraction) const;

	float get_friction() const { return friction; }
	void set_friction(float p_force);

	int get_motor_mode() const { return motor_mode; }
	float get_motor_target() const { return motor_target; }
	void set_motor(int p_mode, float p_target);
	void set_motor_spring(float p_frequency, float p_damping, float p_max_force);

	virtual float get_applied_force() const override;
	virtual float get_applied_torque() const override;

	virtual void rebuild() override;
};
