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
#include "jolt_distance_joint_3d.h"
#include "jolt_joint_3d.h"
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

	ClassDB::bind_integer_constant(c, StringName(), "TYPE_NONE", JOLT_JOINT_TYPE_NONE);
	ClassDB::bind_integer_constant(c, StringName(), "TYPE_FIXED", JOLT_JOINT_TYPE_FIXED);
	ClassDB::bind_integer_constant(c, StringName(), "TYPE_DISTANCE", JOLT_JOINT_TYPE_DISTANCE);
	ClassDB::bind_integer_constant(c, StringName(), "TYPE_PULLEY", JOLT_JOINT_TYPE_PULLEY);
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
