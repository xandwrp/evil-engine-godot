/**************************************************************************/
/*  jolt_distance_joint_3d.h                                              */
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

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Constraints/SpringSettings.h>

class JoltDistanceJoint3D final : public JoltJoint3D {
	float min_distance = 0.0f;
	float max_distance = 0.0f;

	JPH::SpringSettings spring;

	JPH::Constraint *_build_distance(JPH::Body *p_jolt_body_a, JPH::Body *p_jolt_body_b, const Transform3D &p_shifted_ref_a, const Transform3D &p_shifted_ref_b) const;

public:
	JoltDistanceJoint3D(const JoltJoint3D &p_old_joint, JoltBody3D *p_body_a, JoltBody3D *p_body_b, const Vector3 &p_local_a, const Vector3 &p_local_b);

	virtual JoltJointType get_jolt_type() const override { return JOLT_JOINT_TYPE_DISTANCE; }

	float get_min_distance() const { return min_distance; }
	float get_max_distance() const { return max_distance; }
	void set_distance(float p_min_distance, float p_max_distance);

	int get_spring_mode() const { return (int)spring.mMode; }
	float get_spring_value() const { return spring.mFrequency; }
	float get_spring_damping() const { return spring.mDamping; }
	void set_spring(int p_mode, float p_value, float p_damping);

	float get_current_distance() const;

	virtual float get_applied_force() const override;

	virtual void rebuild() override;
};
