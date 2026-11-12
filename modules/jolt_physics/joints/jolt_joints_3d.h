/**************************************************************************/
/*  jolt_joints_3d.h                                                      */
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

#include "core/object/class_db.h"
#include "core/object/object.h"
#include "core/templates/rid.h"

class JoltDistanceJoint3D;
class JoltJoint3D;
class JoltPulleyJoint3D;

class JoltJoints3D : public Object {
	GDCLASS(JoltJoints3D, Object);

	static JoltJoint3D *_get_joint(const RID &p_joint);
	static JoltDistanceJoint3D *_get_distance(const RID &p_joint);
	static JoltPulleyJoint3D *_get_pulley(const RID &p_joint);

protected:
	static void _bind_methods();

public:
	static bool is_enabled(const RID &p_joint);
	static void set_enabled(const RID &p_joint, bool p_enabled);

	static int get_solver_velocity_steps(const RID &p_joint);
	static void set_solver_velocity_steps(const RID &p_joint, int p_steps);

	static int get_solver_position_steps(const RID &p_joint);
	static void set_solver_position_steps(const RID &p_joint, int p_steps);

	static float get_applied_force(const RID &p_joint);
	static float get_applied_torque(const RID &p_joint);

	static double get_param(const RID &p_joint, int p_param);
	static void set_param(const RID &p_joint, int p_param, double p_value);

	static bool get_flag(const RID &p_joint, int p_flag);
	static void set_flag(const RID &p_joint, int p_flag, bool p_enabled);

	static double get_axis_param(const RID &p_joint, Vector3::Axis p_axis, int p_param);
	static void set_axis_param(const RID &p_joint, Vector3::Axis p_axis, int p_param, double p_value);

	static bool get_axis_flag(const RID &p_joint, Vector3::Axis p_axis, int p_flag);
	static void set_axis_flag(const RID &p_joint, Vector3::Axis p_axis, int p_flag, bool p_enabled);

	static void make_fixed(const RID &p_joint, const RID &p_body_a, const Transform3D &p_local_a, const RID &p_body_b, const Transform3D &p_local_b);
	static void make_distance(const RID &p_joint, const RID &p_body_a, const Vector3 &p_local_a, const RID &p_body_b, const Vector3 &p_local_b);
	static void make_pulley(const RID &p_joint, const RID &p_body_a, const Vector3 &p_local_a, const Vector3 &p_fixed_a, const RID &p_body_b, const Vector3 &p_local_b, const Vector3 &p_fixed_b, float p_ratio);

	static int get_jolt_type(const RID &p_joint);

	static Vector2 get_distance_limits(const RID &p_joint);
	static void set_distance_limits(const RID &p_joint, float p_min, float p_max);
	static Vector3 get_distance_spring(const RID &p_joint);
	static void set_distance_spring(const RID &p_joint, int p_mode, float p_value, float p_damping);
	static float get_distance(const RID &p_joint);

	static Vector2 get_pulley_limits(const RID &p_joint);
	static void set_pulley_limits(const RID &p_joint, float p_min, float p_max);
	static float get_pulley_ratio(const RID &p_joint);
	static float get_pulley_length(const RID &p_joint);
};
