/**************************************************************************/
/*  jolt_joints_3d.cpp                                                    */
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

#include "jolt_joints_3d.h"

#include "../jolt_physics_server_3d.h"
#include "../misc/jolt_type_conversions.h"
#include "../objects/jolt_body_3d.h"
#include "jolt_distance_joint_3d.h"
#include "jolt_generic_6dof_joint_3d.h"
#include "jolt_joint_3d.h"
#include "jolt_path_joint_3d.h"
#include "jolt_pulley_joint_3d.h"

using Server = JoltPhysicsServer3D;

JoltJoint3D *JoltJoints3D::_get_joint(const RID &p_joint) {
	Server *server = Server::get_singleton();
	ERR_FAIL_NULL_V_MSG(server, nullptr, "Jolt's joints are reached only while Jolt Physics is the physics server.");

	JoltJoint3D *joint = server->get_joint(p_joint);
	ERR_FAIL_NULL_V_MSG(joint, nullptr, "No Jolt joint has this RID.");

	return joint;
}

JoltDistanceJoint3D *JoltJoints3D::_get_distance(const RID &p_joint) {
	JoltJoint3D *joint = _get_joint(p_joint);
	ERR_FAIL_NULL_V(joint, nullptr);
	ERR_FAIL_COND_V_MSG(joint->get_jolt_type() != JOLT_JOINT_TYPE_DISTANCE, nullptr, "This joint isn't a distance joint.");

	return static_cast<JoltDistanceJoint3D *>(joint);
}

JoltPulleyJoint3D *JoltJoints3D::_get_pulley(const RID &p_joint) {
	JoltJoint3D *joint = _get_joint(p_joint);
	ERR_FAIL_NULL_V(joint, nullptr);
	ERR_FAIL_COND_V_MSG(joint->get_jolt_type() != JOLT_JOINT_TYPE_PULLEY, nullptr, "This joint isn't a pulley.");

	return static_cast<JoltPulleyJoint3D *>(joint);
}

JoltPathJoint3D *JoltJoints3D::_get_path(const RID &p_joint) {
	JoltJoint3D *joint = _get_joint(p_joint);
	ERR_FAIL_NULL_V(joint, nullptr);
	ERR_FAIL_COND_V_MSG(joint->get_jolt_type() != JOLT_JOINT_TYPE_PATH, nullptr, "This joint isn't a path.");

	return static_cast<JoltPathJoint3D *>(joint);
}

JoltGeneric6DOFJoint3D *JoltJoints3D::_get_6dof(const RID &p_joint) {
	JoltJoint3D *joint = _get_joint(p_joint);
	ERR_FAIL_NULL_V(joint, nullptr);
	ERR_FAIL_COND_V_MSG(joint->get_type() != PS3DE::JOINT_TYPE_6DOF, nullptr, "Only a 6DOF joint has a muscle.");

	return static_cast<JoltGeneric6DOFJoint3D *>(joint);
}

bool JoltJoints3D::is_enabled(const RID &p_joint) {
	JoltJoint3D *joint = _get_joint(p_joint);
	return joint != nullptr && joint->is_enabled();
}

void JoltJoints3D::set_enabled(const RID &p_joint, bool p_enabled) {
	if (JoltJoint3D *joint = _get_joint(p_joint)) {
		joint->set_enabled(p_enabled);
	}
}

int JoltJoints3D::get_solver_velocity_steps(const RID &p_joint) {
	JoltJoint3D *joint = _get_joint(p_joint);
	return joint != nullptr ? joint->get_solver_velocity_iterations() : 0;
}

void JoltJoints3D::set_solver_velocity_steps(const RID &p_joint, int p_steps) {
	ERR_FAIL_COND_MSG(p_steps < 0 || p_steps > 255, "A joint's velocity steps are from 0, the space's own, to 255.");

	if (JoltJoint3D *joint = _get_joint(p_joint)) {
		joint->set_solver_velocity_iterations(p_steps);
	}
}

int JoltJoints3D::get_solver_position_steps(const RID &p_joint) {
	JoltJoint3D *joint = _get_joint(p_joint);
	return joint != nullptr ? joint->get_solver_position_iterations() : 0;
}

void JoltJoints3D::set_solver_position_steps(const RID &p_joint, int p_steps) {
	ERR_FAIL_COND_MSG(p_steps < 0 || p_steps > 255, "A joint's position steps are from 0, the space's own, to 255.");

	if (JoltJoint3D *joint = _get_joint(p_joint)) {
		joint->set_solver_position_iterations(p_steps);
	}
}

float JoltJoints3D::get_applied_force(const RID &p_joint) {
	JoltJoint3D *joint = _get_joint(p_joint);
	return joint != nullptr ? joint->get_applied_force() : 0.0f;
}

float JoltJoints3D::get_applied_torque(const RID &p_joint) {
	JoltJoint3D *joint = _get_joint(p_joint);
	return joint != nullptr ? joint->get_applied_torque() : 0.0f;
}

double JoltJoints3D::get_param(const RID &p_joint, int p_param) {
	JoltJoint3D *joint = _get_joint(p_joint);
	ERR_FAIL_NULL_V(joint, 0.0);

	Server *server = Server::get_singleton();

	switch (joint->get_type()) {
		case PS3DE::JOINT_TYPE_HINGE:
			return server->hinge_joint_get_jolt_param(p_joint, (Server::HingeJointParamJolt)p_param);
		case PS3DE::JOINT_TYPE_SLIDER:
			return server->slider_joint_get_jolt_param(p_joint, (Server::SliderJointParamJolt)p_param);
		case PS3DE::JOINT_TYPE_CONE_TWIST:
			return server->cone_twist_joint_get_jolt_param(p_joint, (Server::ConeTwistJointParamJolt)p_param);
		default:
			ERR_FAIL_V_MSG(0.0, "Only a hinge, slider or cone twist joint has Jolt parameters of its own without an axis.");
	}
}

void JoltJoints3D::set_param(const RID &p_joint, int p_param, double p_value) {
	JoltJoint3D *joint = _get_joint(p_joint);
	ERR_FAIL_NULL(joint);

	Server *server = Server::get_singleton();

	switch (joint->get_type()) {
		case PS3DE::JOINT_TYPE_HINGE:
			server->hinge_joint_set_jolt_param(p_joint, (Server::HingeJointParamJolt)p_param, p_value);
			break;
		case PS3DE::JOINT_TYPE_SLIDER:
			server->slider_joint_set_jolt_param(p_joint, (Server::SliderJointParamJolt)p_param, p_value);
			break;
		case PS3DE::JOINT_TYPE_CONE_TWIST:
			server->cone_twist_joint_set_jolt_param(p_joint, (Server::ConeTwistJointParamJolt)p_param, p_value);
			break;
		default:
			ERR_FAIL_MSG("Only a hinge, slider or cone twist joint has Jolt parameters of its own without an axis.");
	}
}

bool JoltJoints3D::get_flag(const RID &p_joint, int p_flag) {
	JoltJoint3D *joint = _get_joint(p_joint);
	ERR_FAIL_NULL_V(joint, false);

	Server *server = Server::get_singleton();

	switch (joint->get_type()) {
		case PS3DE::JOINT_TYPE_HINGE:
			return server->hinge_joint_get_jolt_flag(p_joint, (Server::HingeJointFlagJolt)p_flag);
		case PS3DE::JOINT_TYPE_SLIDER:
			return server->slider_joint_get_jolt_flag(p_joint, (Server::SliderJointFlagJolt)p_flag);
		case PS3DE::JOINT_TYPE_CONE_TWIST:
			return server->cone_twist_joint_get_jolt_flag(p_joint, (Server::ConeTwistJointFlagJolt)p_flag);
		default:
			ERR_FAIL_V_MSG(false, "Only a hinge, slider or cone twist joint has Jolt flags of its own without an axis.");
	}
}

void JoltJoints3D::set_flag(const RID &p_joint, int p_flag, bool p_enabled) {
	JoltJoint3D *joint = _get_joint(p_joint);
	ERR_FAIL_NULL(joint);

	Server *server = Server::get_singleton();

	switch (joint->get_type()) {
		case PS3DE::JOINT_TYPE_HINGE:
			server->hinge_joint_set_jolt_flag(p_joint, (Server::HingeJointFlagJolt)p_flag, p_enabled);
			break;
		case PS3DE::JOINT_TYPE_SLIDER:
			server->slider_joint_set_jolt_flag(p_joint, (Server::SliderJointFlagJolt)p_flag, p_enabled);
			break;
		case PS3DE::JOINT_TYPE_CONE_TWIST:
			server->cone_twist_joint_set_jolt_flag(p_joint, (Server::ConeTwistJointFlagJolt)p_flag, p_enabled);
			break;
		default:
			ERR_FAIL_MSG("Only a hinge, slider or cone twist joint has Jolt flags of its own without an axis.");
	}
}

double JoltJoints3D::get_axis_param(const RID &p_joint, Vector3::Axis p_axis, int p_param) {
	JoltJoint3D *joint = _get_joint(p_joint);
	ERR_FAIL_NULL_V(joint, 0.0);
	ERR_FAIL_COND_V_MSG(joint->get_type() != PS3DE::JOINT_TYPE_6DOF, 0.0, "Only a 6DOF joint has Jolt parameters per axis.");

	return Server::get_singleton()->generic_6dof_joint_get_jolt_param(p_joint, p_axis, (Server::G6DOFJointAxisParamJolt)p_param);
}

void JoltJoints3D::set_axis_param(const RID &p_joint, Vector3::Axis p_axis, int p_param, double p_value) {
	JoltJoint3D *joint = _get_joint(p_joint);
	ERR_FAIL_NULL(joint);
	ERR_FAIL_COND_MSG(joint->get_type() != PS3DE::JOINT_TYPE_6DOF, "Only a 6DOF joint has Jolt parameters per axis.");

	Server::get_singleton()->generic_6dof_joint_set_jolt_param(p_joint, p_axis, (Server::G6DOFJointAxisParamJolt)p_param, p_value);
}

bool JoltJoints3D::get_axis_flag(const RID &p_joint, Vector3::Axis p_axis, int p_flag) {
	JoltJoint3D *joint = _get_joint(p_joint);
	ERR_FAIL_NULL_V(joint, false);
	ERR_FAIL_COND_V_MSG(joint->get_type() != PS3DE::JOINT_TYPE_6DOF, false, "Only a 6DOF joint has Jolt flags per axis.");

	return Server::get_singleton()->generic_6dof_joint_get_jolt_flag(p_joint, p_axis, (Server::G6DOFJointAxisFlagJolt)p_flag);
}

void JoltJoints3D::set_axis_flag(const RID &p_joint, Vector3::Axis p_axis, int p_flag, bool p_enabled) {
	JoltJoint3D *joint = _get_joint(p_joint);
	ERR_FAIL_NULL(joint);
	ERR_FAIL_COND_MSG(joint->get_type() != PS3DE::JOINT_TYPE_6DOF, "Only a 6DOF joint has Jolt flags per axis.");

	Server::get_singleton()->generic_6dof_joint_set_jolt_flag(p_joint, p_axis, (Server::G6DOFJointAxisFlagJolt)p_flag, p_enabled);
}

void JoltJoints3D::make_fixed(const RID &p_joint, const RID &p_body_a, const Transform3D &p_local_a, const RID &p_body_b, const Transform3D &p_local_b) {
	Server *server = Server::get_singleton();
	ERR_FAIL_NULL(server);

	server->joint_make_fixed(p_joint, p_body_a, p_local_a, p_body_b, p_local_b);
}

void JoltJoints3D::make_distance(const RID &p_joint, const RID &p_body_a, const Vector3 &p_local_a, const RID &p_body_b, const Vector3 &p_local_b) {
	Server *server = Server::get_singleton();
	ERR_FAIL_NULL(server);

	server->joint_make_distance(p_joint, p_body_a, p_local_a, p_body_b, p_local_b);
}

void JoltJoints3D::make_pulley(const RID &p_joint, const RID &p_body_a, const Vector3 &p_local_a, const Vector3 &p_fixed_a, const RID &p_body_b, const Vector3 &p_local_b, const Vector3 &p_fixed_b, float p_ratio) {
	ERR_FAIL_COND_MSG(!(p_ratio > 0.0f), "A pulley's ratio is above zero.");

	Server *server = Server::get_singleton();
	ERR_FAIL_NULL(server);

	server->joint_make_pulley(p_joint, p_body_a, p_local_a, p_fixed_a, p_body_b, p_local_b, p_fixed_b, p_ratio);
}

int JoltJoints3D::get_jolt_type(const RID &p_joint) {
	JoltJoint3D *joint = _get_joint(p_joint);
	return joint != nullptr ? (int)joint->get_jolt_type() : (int)JOLT_JOINT_TYPE_NONE;
}

Vector2 JoltJoints3D::get_distance_limits(const RID &p_joint) {
	JoltDistanceJoint3D *joint = _get_distance(p_joint);
	return joint != nullptr ? Vector2(joint->get_min_distance(), joint->get_max_distance()) : Vector2();
}

void JoltJoints3D::set_distance_limits(const RID &p_joint, float p_min, float p_max) {
	if (JoltDistanceJoint3D *joint = _get_distance(p_joint)) {
		joint->set_distance(p_min, p_max);
	}
}

Vector3 JoltJoints3D::get_distance_spring(const RID &p_joint) {
	JoltDistanceJoint3D *joint = _get_distance(p_joint);
	return joint != nullptr ? Vector3(joint->get_spring_mode(), joint->get_spring_value(), joint->get_spring_damping()) : Vector3();
}

void JoltJoints3D::set_distance_spring(const RID &p_joint, int p_mode, float p_value, float p_damping) {
	if (JoltDistanceJoint3D *joint = _get_distance(p_joint)) {
		joint->set_spring(p_mode, p_value, p_damping);
	}
}

float JoltJoints3D::get_distance(const RID &p_joint) {
	JoltDistanceJoint3D *joint = _get_distance(p_joint);
	return joint != nullptr ? joint->get_current_distance() : 0.0f;
}

Vector2 JoltJoints3D::get_pulley_limits(const RID &p_joint) {
	JoltPulleyJoint3D *joint = _get_pulley(p_joint);
	return joint != nullptr ? Vector2(joint->get_min_length(), joint->get_max_length()) : Vector2();
}

void JoltJoints3D::set_pulley_limits(const RID &p_joint, float p_min, float p_max) {
	if (JoltPulleyJoint3D *joint = _get_pulley(p_joint)) {
		joint->set_length(p_min, p_max);
	}
}

float JoltJoints3D::get_pulley_ratio(const RID &p_joint) {
	JoltPulleyJoint3D *joint = _get_pulley(p_joint);
	return joint != nullptr ? joint->get_ratio() : 0.0f;
}

float JoltJoints3D::get_pulley_length(const RID &p_joint) {
	JoltPulleyJoint3D *joint = _get_pulley(p_joint);
	return joint != nullptr ? joint->get_current_length() : 0.0f;
}

void JoltJoints3D::make_path(const RID &p_joint, const RID &p_holder, const Transform3D &p_path_frame, const PackedVector3Array &p_points, const PackedVector3Array &p_tangents, const PackedVector3Array &p_normals, bool p_looping, const RID &p_follower, const Vector3 &p_follower_point, int p_rotation) {
	ERR_FAIL_COND_MSG(p_points.size() < 2, "A path runs through two points or more.");
	ERR_FAIL_COND_MSG(!p_tangents.is_empty() && p_tangents.size() != p_points.size(), "A path's tangents are one per point, or none to take them from its points.");
	ERR_FAIL_COND_MSG(!p_normals.is_empty() && p_normals.size() != p_points.size(), "A path's normals are one per point, or none to keep them upright.");
	ERR_FAIL_COND_MSG(p_rotation < 0 || p_rotation > 5, "A path's follower turns freely (0), about its tangent (1), normal (2) or binormal (3), with it (4), or with its holder (5).");

	Server *server = Server::get_singleton();
	ERR_FAIL_NULL(server);

	server->joint_make_path(p_joint, p_holder, p_path_frame, p_points, p_tangents, p_normals, p_looping, p_follower, p_follower_point, p_rotation);
}

float JoltJoints3D::get_path_fraction(const RID &p_joint) {
	JoltPathJoint3D *joint = _get_path(p_joint);
	return joint != nullptr ? joint->get_fraction() : 0.0f;
}

float JoltJoints3D::get_path_max_fraction(const RID &p_joint) {
	JoltPathJoint3D *joint = _get_path(p_joint);
	return joint != nullptr ? joint->get_max_fraction() : 0.0f;
}

Transform3D JoltJoints3D::get_path_point(const RID &p_joint, float p_fraction) {
	JoltPathJoint3D *joint = _get_path(p_joint);
	return joint != nullptr ? joint->get_point(p_fraction) : Transform3D();
}

void JoltJoints3D::set_path_friction(const RID &p_joint, float p_force) {
	if (JoltPathJoint3D *joint = _get_path(p_joint)) {
		joint->set_friction(p_force);
	}
}

void JoltJoints3D::set_path_motor(const RID &p_joint, int p_mode, float p_target) {
	if (JoltPathJoint3D *joint = _get_path(p_joint)) {
		joint->set_motor(p_mode, p_target);
	}
}

void JoltJoints3D::set_path_motor_spring(const RID &p_joint, float p_frequency, float p_damping, float p_max_force) {
	if (JoltPathJoint3D *joint = _get_path(p_joint)) {
		joint->set_motor_spring(p_frequency, p_damping, p_max_force);
	}
}

void JoltJoints3D::set_muscles(const TypedArray<RID> &p_joints, const PackedFloat32Array &p_settings) {
	const int count = p_joints.size();
	ERR_FAIL_COND_MSG(p_settings.size() != count * 3, "A muscle's settings are three floats per joint: its stiffness in newton metres per radian, its damping in newton metre seconds per radian and its torque limit in newton metres.");

	LocalVector<JoltGeneric6DOFJoint3D *> joints;
	joints.resize(count);

	for (int i = 0; i < count; ++i) {
		joints[i] = _get_6dof(p_joints[i]);
		ERR_FAIL_NULL(joints[i]);

		const float stiffness = p_settings[i * 3 + 0];
		const float damping = p_settings[i * 3 + 1];
		const float torque = p_settings[i * 3 + 2];

		ERR_FAIL_COND_MSG(!(torque >= 0.0f) || !Math::is_finite(torque), vformat("Muscle %d's torque limit is from 0, off and back on its friction, upward.", i));

		if (torque > 0.0f) {
			ERR_FAIL_COND_MSG(!(stiffness > 0.0f) || !Math::is_finite(stiffness), vformat("Muscle %d's stiffness is above 0 while it pulls, since Jolt turns a spring without stiffness off.", i));
			ERR_FAIL_COND_MSG(!(damping >= 0.0f) || !Math::is_finite(damping), vformat("Muscle %d's damping is from 0 upward.", i));
		}
	}

	for (int i = 0; i < count; ++i) {
		joints[i]->set_muscle(p_settings[i * 3 + 0], p_settings[i * 3 + 1], p_settings[i * 3 + 2]);
	}
}

void JoltJoints3D::drive_muscles(const TypedArray<RID> &p_joints, const PackedFloat32Array &p_targets) {
	const int count = p_joints.size();
	ERR_FAIL_COND_MSG(p_targets.size() != count * 7, "A muscle's target is seven floats per joint: its orientation in its constraint space as a quaternion's x, y, z and w, and its angular velocity in body 2's constraint space.");

	LocalVector<JoltGeneric6DOFJoint3D *> joints;
	joints.resize(count);

	for (int i = 0; i < count; ++i) {
		joints[i] = _get_6dof(p_joints[i]);
		ERR_FAIL_NULL(joints[i]);

		const float *target = p_targets.ptr() + i * 7;
		const float length = target[0] * target[0] + target[1] * target[1] + target[2] * target[2] + target[3] * target[3];
		ERR_FAIL_COND_MSG(!(Math::abs(length - 1.0f) < 1e-3f), vformat("Muscle %d's orientation is a unit quaternion.", i));
		ERR_FAIL_COND_MSG(!Math::is_finite(target[4]) || !Math::is_finite(target[5]) || !Math::is_finite(target[6]), vformat("Muscle %d's angular velocity is finite.", i));
	}

	for (int i = 0; i < count; ++i) {
		const float *target = p_targets.ptr() + i * 7;
		const JPH::Quat orientation = JPH::Quat(target[0], target[1], target[2], target[3]).Normalized();
		joints[i]->drive_muscle(orientation, JPH::Vec3(target[4], target[5], target[6]));
	}
}

PackedFloat32Array JoltJoints3D::get_muscles(const TypedArray<RID> &p_joints) {
	const int count = p_joints.size();

	PackedFloat32Array muscles;
	muscles.resize(count * 4);
	float *muscle = muscles.ptrw();

	for (int i = 0; i < count; ++i) {
		JoltGeneric6DOFJoint3D *joint = _get_6dof(p_joints[i]);
		ERR_FAIL_NULL_V(joint, PackedFloat32Array());

		const Vector3 torque = joint->get_muscle_torque();
		muscle[i * 4 + 0] = (float)torque.x;
		muscle[i * 4 + 1] = (float)torque.y;
		muscle[i * 4 + 2] = (float)torque.z;
		muscle[i * 4 + 3] = joint->get_muscle_error();
	}

	return muscles;
}

void JoltJoints3D::make_pin(const RID &p_joint, const RID &p_body, const Transform3D &p_frame) {
	Server *server = Server::get_singleton();
	ERR_FAIL_NULL_MSG(server, "Jolt's joints are reached only while Jolt Physics is the physics server.");

	JoltBody3D *body = server->get_body(p_body);
	ERR_FAIL_NULL_MSG(body, "A pin holds a body.");

	Transform3D world = body->get_transform_unscaled() * p_frame;
	world.basis.orthonormalize();

	server->joint_make_world_pin(p_joint, p_body, p_frame, world);
}

void JoltJoints3D::set_pin(const RID &p_joint, float p_stiffness, float p_damping, float p_force, float p_turn_stiffness, float p_turn_damping, float p_torque) {
	JoltGeneric6DOFJoint3D *joint = _get_6dof(p_joint);
	ERR_FAIL_NULL(joint);
	ERR_FAIL_COND_MSG(!joint->is_held_by_world(), "This 6DOF joint isn't a pin, held by the world on its first side.");
	ERR_FAIL_COND_MSG(!(p_force >= 0.0f) || !Math::is_finite(p_force), "A pin's force limit is from 0, off, upward.");
	ERR_FAIL_COND_MSG(!(p_torque >= 0.0f) || !Math::is_finite(p_torque), "A pin's torque limit is from 0, off, upward.");

	if (p_force > 0.0f) {
		ERR_FAIL_COND_MSG(!(p_stiffness > 0.0f) || !Math::is_finite(p_stiffness), "A pin's stiffness is above 0 while it pulls, since Jolt turns a spring without stiffness off.");
		ERR_FAIL_COND_MSG(!(p_damping >= 0.0f) || !Math::is_finite(p_damping), "A pin's damping is from 0 upward.");
	}

	if (p_torque > 0.0f) {
		ERR_FAIL_COND_MSG(!(p_turn_stiffness > 0.0f) || !Math::is_finite(p_turn_stiffness), "A pin's turning stiffness is above 0 while it turns, since Jolt turns a spring without stiffness off.");
		ERR_FAIL_COND_MSG(!(p_turn_damping >= 0.0f) || !Math::is_finite(p_turn_damping), "A pin's turning damping is from 0 upward.");
	}

	joint->set_pin(p_stiffness, p_damping, p_force);
	joint->set_muscle(p_turn_stiffness, p_turn_damping, p_torque);
}

void JoltJoints3D::drive_pin(const RID &p_joint, const Vector3 &p_position, const Quaternion &p_orientation, const Vector3 &p_velocity, const Vector3 &p_spin) {
	JoltGeneric6DOFJoint3D *joint = _get_6dof(p_joint);
	ERR_FAIL_NULL(joint);
	ERR_FAIL_COND_MSG(!joint->is_held_by_world(), "This 6DOF joint isn't a pin, held by the world on its first side.");
	ERR_FAIL_COND_MSG(!p_orientation.is_normalized(), "A pin's orientation is a unit quaternion.");

	Transform3D world;
	Transform3D held;
	ERR_FAIL_COND(!joint->get_world_frames(world, held));

	const JPH::Vec3 position = to_jolt(world.basis.xform_inv(p_position - world.origin));
	const JPH::Vec3 velocity = to_jolt(world.basis.xform_inv(p_velocity));
	const JPH::Quat orientation = (to_jolt(world.basis).Conjugated() * to_jolt(p_orientation)).Normalized();
	const JPH::Vec3 spin = to_jolt(held.basis.xform_inv(p_spin));

	joint->drive_pin(position, velocity);
	joint->drive_muscle(orientation, spin);
}

PackedFloat32Array JoltJoints3D::get_pin(const RID &p_joint) {
	JoltGeneric6DOFJoint3D *joint = _get_6dof(p_joint);
	ERR_FAIL_NULL_V(joint, PackedFloat32Array());
	ERR_FAIL_COND_V_MSG(!joint->is_held_by_world(), PackedFloat32Array(), "This 6DOF joint isn't a pin, held by the world on its first side.");

	Transform3D world;
	Transform3D held;
	ERR_FAIL_COND_V(!joint->get_world_frames(world, held), PackedFloat32Array());

	const Vector3 force = world.basis.xform(joint->get_pin_force());
	const Vector3 torque = held.basis.xform(joint->get_muscle_torque());

	PackedFloat32Array pin;
	pin.resize(8);
	float *values = pin.ptrw();
	values[0] = (float)force.x;
	values[1] = (float)force.y;
	values[2] = (float)force.z;
	values[3] = (float)torque.x;
	values[4] = (float)torque.y;
	values[5] = (float)torque.z;
	values[6] = joint->get_pin_error();
	values[7] = joint->get_muscle_error();

	return pin;
}

void JoltJoints3D::_bind_methods() {
	const StringName c = get_class_static();

	ClassDB::bind_static_method(c, D_METHOD("is_enabled", "joint"), &JoltJoints3D::is_enabled);
	ClassDB::bind_static_method(c, D_METHOD("set_enabled", "joint", "enabled"), &JoltJoints3D::set_enabled);

	ClassDB::bind_static_method(c, D_METHOD("get_solver_velocity_steps", "joint"), &JoltJoints3D::get_solver_velocity_steps);
	ClassDB::bind_static_method(c, D_METHOD("set_solver_velocity_steps", "joint", "steps"), &JoltJoints3D::set_solver_velocity_steps);
	ClassDB::bind_static_method(c, D_METHOD("get_solver_position_steps", "joint"), &JoltJoints3D::get_solver_position_steps);
	ClassDB::bind_static_method(c, D_METHOD("set_solver_position_steps", "joint", "steps"), &JoltJoints3D::set_solver_position_steps);

	ClassDB::bind_static_method(c, D_METHOD("get_applied_force", "joint"), &JoltJoints3D::get_applied_force);
	ClassDB::bind_static_method(c, D_METHOD("get_applied_torque", "joint"), &JoltJoints3D::get_applied_torque);

	ClassDB::bind_static_method(c, D_METHOD("get_param", "joint", "param"), &JoltJoints3D::get_param);
	ClassDB::bind_static_method(c, D_METHOD("set_param", "joint", "param", "value"), &JoltJoints3D::set_param);
	ClassDB::bind_static_method(c, D_METHOD("get_flag", "joint", "flag"), &JoltJoints3D::get_flag);
	ClassDB::bind_static_method(c, D_METHOD("set_flag", "joint", "flag", "enabled"), &JoltJoints3D::set_flag);

	ClassDB::bind_static_method(c, D_METHOD("get_axis_param", "joint", "axis", "param"), &JoltJoints3D::get_axis_param);
	ClassDB::bind_static_method(c, D_METHOD("set_axis_param", "joint", "axis", "param", "value"), &JoltJoints3D::set_axis_param);
	ClassDB::bind_static_method(c, D_METHOD("get_axis_flag", "joint", "axis", "flag"), &JoltJoints3D::get_axis_flag);
	ClassDB::bind_static_method(c, D_METHOD("set_axis_flag", "joint", "axis", "flag", "enabled"), &JoltJoints3D::set_axis_flag);

	ClassDB::bind_static_method(c, D_METHOD("make_fixed", "joint", "body_a", "local_a", "body_b", "local_b"), &JoltJoints3D::make_fixed);
	ClassDB::bind_static_method(c, D_METHOD("make_distance", "joint", "body_a", "local_a", "body_b", "local_b"), &JoltJoints3D::make_distance);
	ClassDB::bind_static_method(c, D_METHOD("make_pulley", "joint", "body_a", "local_a", "fixed_a", "body_b", "local_b", "fixed_b", "ratio"), &JoltJoints3D::make_pulley);
	ClassDB::bind_static_method(c, D_METHOD("get_jolt_type", "joint"), &JoltJoints3D::get_jolt_type);

	ClassDB::bind_static_method(c, D_METHOD("get_distance_limits", "joint"), &JoltJoints3D::get_distance_limits);
	ClassDB::bind_static_method(c, D_METHOD("set_distance_limits", "joint", "min", "max"), &JoltJoints3D::set_distance_limits);
	ClassDB::bind_static_method(c, D_METHOD("get_distance_spring", "joint"), &JoltJoints3D::get_distance_spring);
	ClassDB::bind_static_method(c, D_METHOD("set_distance_spring", "joint", "mode", "value", "damping"), &JoltJoints3D::set_distance_spring);
	ClassDB::bind_static_method(c, D_METHOD("get_distance", "joint"), &JoltJoints3D::get_distance);

	ClassDB::bind_static_method(c, D_METHOD("get_pulley_limits", "joint"), &JoltJoints3D::get_pulley_limits);
	ClassDB::bind_static_method(c, D_METHOD("set_pulley_limits", "joint", "min", "max"), &JoltJoints3D::set_pulley_limits);
	ClassDB::bind_static_method(c, D_METHOD("get_pulley_ratio", "joint"), &JoltJoints3D::get_pulley_ratio);
	ClassDB::bind_static_method(c, D_METHOD("get_pulley_length", "joint"), &JoltJoints3D::get_pulley_length);

	ClassDB::bind_static_method(c, D_METHOD("make_path", "joint", "holder", "path_frame", "points", "tangents", "normals", "looping", "follower", "follower_point", "rotation"), &JoltJoints3D::make_path);
	ClassDB::bind_static_method(c, D_METHOD("get_path_fraction", "joint"), &JoltJoints3D::get_path_fraction);
	ClassDB::bind_static_method(c, D_METHOD("get_path_max_fraction", "joint"), &JoltJoints3D::get_path_max_fraction);
	ClassDB::bind_static_method(c, D_METHOD("get_path_point", "joint", "fraction"), &JoltJoints3D::get_path_point);
	ClassDB::bind_static_method(c, D_METHOD("set_path_friction", "joint", "force"), &JoltJoints3D::set_path_friction);
	ClassDB::bind_static_method(c, D_METHOD("set_path_motor", "joint", "mode", "target"), &JoltJoints3D::set_path_motor);
	ClassDB::bind_static_method(c, D_METHOD("set_path_motor_spring", "joint", "frequency", "damping", "max_force"), &JoltJoints3D::set_path_motor_spring);

	ClassDB::bind_static_method(c, D_METHOD("set_muscles", "joints", "settings"), &JoltJoints3D::set_muscles);
	ClassDB::bind_static_method(c, D_METHOD("drive_muscles", "joints", "targets"), &JoltJoints3D::drive_muscles);
	ClassDB::bind_static_method(c, D_METHOD("get_muscles", "joints"), &JoltJoints3D::get_muscles);

	ClassDB::bind_static_method(c, D_METHOD("make_pin", "joint", "body", "frame"), &JoltJoints3D::make_pin);
	ClassDB::bind_static_method(c, D_METHOD("set_pin", "joint", "stiffness", "damping", "force", "turn_stiffness", "turn_damping", "torque"), &JoltJoints3D::set_pin);
	ClassDB::bind_static_method(c, D_METHOD("drive_pin", "joint", "position", "orientation", "velocity", "spin"), &JoltJoints3D::drive_pin);
	ClassDB::bind_static_method(c, D_METHOD("get_pin", "joint"), &JoltJoints3D::get_pin);

	ClassDB::bind_integer_constant(c, StringName(), "TYPE_NONE", JOLT_JOINT_TYPE_NONE);
	ClassDB::bind_integer_constant(c, StringName(), "TYPE_FIXED", JOLT_JOINT_TYPE_FIXED);
	ClassDB::bind_integer_constant(c, StringName(), "TYPE_DISTANCE", JOLT_JOINT_TYPE_DISTANCE);
	ClassDB::bind_integer_constant(c, StringName(), "TYPE_PULLEY", JOLT_JOINT_TYPE_PULLEY);
	ClassDB::bind_integer_constant(c, StringName(), "TYPE_PATH", JOLT_JOINT_TYPE_PATH);
	ClassDB::bind_integer_constant(c, StringName(), "PATH_TURN_FREE", 0);
	ClassDB::bind_integer_constant(c, StringName(), "PATH_TURN_ABOUT_TANGENT", 1);
	ClassDB::bind_integer_constant(c, StringName(), "PATH_TURN_ABOUT_NORMAL", 2);
	ClassDB::bind_integer_constant(c, StringName(), "PATH_TURN_ABOUT_BINORMAL", 3);
	ClassDB::bind_integer_constant(c, StringName(), "PATH_TURN_WITH_PATH", 4);
	ClassDB::bind_integer_constant(c, StringName(), "PATH_TURN_WITH_HOLDER", 5);
	ClassDB::bind_integer_constant(c, StringName(), "PATH_MOTOR_OFF", 0);
	ClassDB::bind_integer_constant(c, StringName(), "PATH_MOTOR_SPEED", 1);
	ClassDB::bind_integer_constant(c, StringName(), "PATH_MOTOR_FRACTION", 2);
	ClassDB::bind_integer_constant(c, StringName(), "SPRING_FREQUENCY", 0);
	ClassDB::bind_integer_constant(c, StringName(), "SPRING_STIFFNESS", 1);

	ClassDB::bind_integer_constant(c, StringName(), "HINGE_LIMIT_SPRING_FREQUENCY", Server::HINGE_JOINT_LIMIT_SPRING_FREQUENCY);
	ClassDB::bind_integer_constant(c, StringName(), "HINGE_LIMIT_SPRING_DAMPING", Server::HINGE_JOINT_LIMIT_SPRING_DAMPING);
	ClassDB::bind_integer_constant(c, StringName(), "HINGE_MOTOR_MAX_TORQUE", Server::HINGE_JOINT_MOTOR_MAX_TORQUE);
	ClassDB::bind_integer_constant(c, StringName(), "HINGE_FLAG_USE_LIMIT_SPRING", Server::HINGE_JOINT_FLAG_USE_LIMIT_SPRING);

	ClassDB::bind_integer_constant(c, StringName(), "SLIDER_LIMIT_SPRING_FREQUENCY", Server::SLIDER_JOINT_LIMIT_SPRING_FREQUENCY);
	ClassDB::bind_integer_constant(c, StringName(), "SLIDER_LIMIT_SPRING_DAMPING", Server::SLIDER_JOINT_LIMIT_SPRING_DAMPING);
	ClassDB::bind_integer_constant(c, StringName(), "SLIDER_MOTOR_TARGET_VELOCITY", Server::SLIDER_JOINT_MOTOR_TARGET_VELOCITY);
	ClassDB::bind_integer_constant(c, StringName(), "SLIDER_MOTOR_MAX_FORCE", Server::SLIDER_JOINT_MOTOR_MAX_FORCE);
	ClassDB::bind_integer_constant(c, StringName(), "SLIDER_FLAG_USE_LIMIT", Server::SLIDER_JOINT_FLAG_USE_LIMIT);
	ClassDB::bind_integer_constant(c, StringName(), "SLIDER_FLAG_USE_LIMIT_SPRING", Server::SLIDER_JOINT_FLAG_USE_LIMIT_SPRING);
	ClassDB::bind_integer_constant(c, StringName(), "SLIDER_FLAG_ENABLE_MOTOR", Server::SLIDER_JOINT_FLAG_ENABLE_MOTOR);

	ClassDB::bind_integer_constant(c, StringName(), "CONE_TWIST_SWING_MOTOR_TARGET_VELOCITY_Y", Server::CONE_TWIST_JOINT_SWING_MOTOR_TARGET_VELOCITY_Y);
	ClassDB::bind_integer_constant(c, StringName(), "CONE_TWIST_SWING_MOTOR_TARGET_VELOCITY_Z", Server::CONE_TWIST_JOINT_SWING_MOTOR_TARGET_VELOCITY_Z);
	ClassDB::bind_integer_constant(c, StringName(), "CONE_TWIST_TWIST_MOTOR_TARGET_VELOCITY", Server::CONE_TWIST_JOINT_TWIST_MOTOR_TARGET_VELOCITY);
	ClassDB::bind_integer_constant(c, StringName(), "CONE_TWIST_SWING_MOTOR_MAX_TORQUE", Server::CONE_TWIST_JOINT_SWING_MOTOR_MAX_TORQUE);
	ClassDB::bind_integer_constant(c, StringName(), "CONE_TWIST_TWIST_MOTOR_MAX_TORQUE", Server::CONE_TWIST_JOINT_TWIST_MOTOR_MAX_TORQUE);
	ClassDB::bind_integer_constant(c, StringName(), "CONE_TWIST_FLAG_USE_SWING_LIMIT", Server::CONE_TWIST_JOINT_FLAG_USE_SWING_LIMIT);
	ClassDB::bind_integer_constant(c, StringName(), "CONE_TWIST_FLAG_USE_TWIST_LIMIT", Server::CONE_TWIST_JOINT_FLAG_USE_TWIST_LIMIT);
	ClassDB::bind_integer_constant(c, StringName(), "CONE_TWIST_FLAG_ENABLE_SWING_MOTOR", Server::CONE_TWIST_JOINT_FLAG_ENABLE_SWING_MOTOR);
	ClassDB::bind_integer_constant(c, StringName(), "CONE_TWIST_FLAG_ENABLE_TWIST_MOTOR", Server::CONE_TWIST_JOINT_FLAG_ENABLE_TWIST_MOTOR);

	ClassDB::bind_integer_constant(c, StringName(), "AXIS_LINEAR_SPRING_FREQUENCY", Server::G6DOF_JOINT_LINEAR_SPRING_FREQUENCY);
	ClassDB::bind_integer_constant(c, StringName(), "AXIS_LINEAR_LIMIT_SPRING_FREQUENCY", Server::G6DOF_JOINT_LINEAR_LIMIT_SPRING_FREQUENCY);
	ClassDB::bind_integer_constant(c, StringName(), "AXIS_LINEAR_LIMIT_SPRING_DAMPING", Server::G6DOF_JOINT_LINEAR_LIMIT_SPRING_DAMPING);
	ClassDB::bind_integer_constant(c, StringName(), "AXIS_ANGULAR_SPRING_FREQUENCY", Server::G6DOF_JOINT_ANGULAR_SPRING_FREQUENCY);
	ClassDB::bind_integer_constant(c, StringName(), "AXIS_FLAG_ENABLE_LINEAR_LIMIT_SPRING", Server::G6DOF_JOINT_FLAG_ENABLE_LINEAR_LIMIT_SPRING);
	ClassDB::bind_integer_constant(c, StringName(), "AXIS_FLAG_ENABLE_LINEAR_SPRING_FREQUENCY", Server::G6DOF_JOINT_FLAG_ENABLE_LINEAR_SPRING_FREQUENCY);
	ClassDB::bind_integer_constant(c, StringName(), "AXIS_FLAG_ENABLE_ANGULAR_SPRING_FREQUENCY", Server::G6DOF_JOINT_FLAG_ENABLE_ANGULAR_SPRING_FREQUENCY);
}
