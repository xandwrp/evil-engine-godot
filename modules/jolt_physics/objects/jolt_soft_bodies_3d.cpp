/**************************************************************************/
/*  jolt_soft_bodies_3d.cpp                                               */
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

#include "jolt_soft_bodies_3d.h"

#include "../jolt_physics_server_3d.h"
#include "jolt_soft_body_3d.h"

JoltSoftBody3D *JoltSoftBodies3D::_get_soft_body(const RID &p_soft_body) {
	JoltPhysicsServer3D *server = JoltPhysicsServer3D::get_singleton();
	ERR_FAIL_NULL_V_MSG(server, nullptr, "Jolt's soft bodies are reached only while Jolt Physics is the physics server.");

	JoltSoftBody3D *soft_body = server->get_soft_body(p_soft_body);
	ERR_FAIL_NULL_V_MSG(soft_body, nullptr, "No Jolt soft body has this RID.");

	return soft_body;
}

JoltSoftBody3D *JoltSoftBodies3D::_get_built(const RID &p_soft_body) {
	JoltSoftBody3D *soft_body = _get_soft_body(p_soft_body);
	ERR_FAIL_NULL_V(soft_body, nullptr);
	ERR_FAIL_COND_V_MSG(!soft_body->is_built(), nullptr, "This soft body wasn't built from data by JoltSoftBodies3D.build.");

	return soft_body;
}

PackedInt32Array JoltSoftBodies3D::build(const RID &p_soft_body, const Transform3D &p_frame, const PackedVector3Array &p_positions, const PackedInt32Array &p_indices, const Dictionary &p_settings) {
	JoltSoftBody3D *soft_body = _get_soft_body(p_soft_body);
	return soft_body != nullptr ? soft_body->build(p_frame, p_positions, p_indices, p_settings) : PackedInt32Array();
}

bool JoltSoftBodies3D::is_built(const RID &p_soft_body) {
	JoltSoftBody3D *soft_body = _get_soft_body(p_soft_body);
	return soft_body != nullptr && soft_body->is_built();
}

void JoltSoftBodies3D::set_settings(const RID &p_soft_body, const Dictionary &p_settings) {
	if (JoltSoftBody3D *soft_body = _get_built(p_soft_body)) {
		soft_body->set_cloth_settings(p_settings);
	}
}

Dictionary JoltSoftBodies3D::get_settings(const RID &p_soft_body) {
	JoltSoftBody3D *soft_body = _get_built(p_soft_body);
	return soft_body != nullptr ? soft_body->get_cloth_settings() : Dictionary();
}

PackedVector3Array JoltSoftBodies3D::get_vertices(const RID &p_soft_body) {
	JoltSoftBody3D *soft_body = _get_soft_body(p_soft_body);
	return soft_body != nullptr ? soft_body->get_vertices() : PackedVector3Array();
}

PackedVector3Array JoltSoftBodies3D::get_velocities(const RID &p_soft_body) {
	JoltSoftBody3D *soft_body = _get_soft_body(p_soft_body);
	return soft_body != nullptr ? soft_body->get_velocities() : PackedVector3Array();
}

float JoltSoftBodies3D::get_volume(const RID &p_soft_body) {
	JoltSoftBody3D *soft_body = _get_soft_body(p_soft_body);
	return soft_body != nullptr ? soft_body->get_volume() : 0.0f;
}

float JoltSoftBodies3D::get_rest_volume(const RID &p_soft_body) {
	JoltSoftBody3D *soft_body = _get_built(p_soft_body);
	return soft_body != nullptr ? soft_body->get_rest_volume() : 0.0f;
}

void JoltSoftBodies3D::skin(const RID &p_soft_body, const Transform3D &p_frame, const TypedArray<Transform3D> &p_joints, bool p_hard) {
	if (JoltSoftBody3D *soft_body = _get_built(p_soft_body)) {
		soft_body->skin(p_frame, p_joints, p_hard);
	}
}

void JoltSoftBodies3D::update(const RID &p_soft_body, float p_delta) {
	if (JoltSoftBody3D *soft_body = _get_built(p_soft_body)) {
		soft_body->update(p_delta);
	}
}

float JoltSoftBodies3D::get_gas_pressure(const RID &p_soft_body) {
	JoltSoftBody3D *soft_body = _get_built(p_soft_body);
	return soft_body != nullptr ? soft_body->get_gas_pressure() : 0.0f;
}

PackedByteArray JoltSoftBodies3D::save_state(const RID &p_soft_body) {
	JoltSoftBody3D *soft_body = _get_soft_body(p_soft_body);
	return soft_body != nullptr ? soft_body->save_state() : PackedByteArray();
}

bool JoltSoftBodies3D::restore_state(const RID &p_soft_body, const PackedByteArray &p_state) {
	JoltSoftBody3D *soft_body = _get_soft_body(p_soft_body);
	return soft_body != nullptr && soft_body->restore_state(p_state);
}

void JoltSoftBodies3D::_bind_methods() {
	const StringName c = get_class_static();

	ClassDB::bind_static_method(c, D_METHOD("build", "soft_body", "frame", "positions", "indices", "settings"), &JoltSoftBodies3D::build);
	ClassDB::bind_static_method(c, D_METHOD("is_built", "soft_body"), &JoltSoftBodies3D::is_built);

	ClassDB::bind_static_method(c, D_METHOD("set_settings", "soft_body", "settings"), &JoltSoftBodies3D::set_settings);
	ClassDB::bind_static_method(c, D_METHOD("get_settings", "soft_body"), &JoltSoftBodies3D::get_settings);

	ClassDB::bind_static_method(c, D_METHOD("get_vertices", "soft_body"), &JoltSoftBodies3D::get_vertices);
	ClassDB::bind_static_method(c, D_METHOD("get_velocities", "soft_body"), &JoltSoftBodies3D::get_velocities);
	ClassDB::bind_static_method(c, D_METHOD("get_volume", "soft_body"), &JoltSoftBodies3D::get_volume);
	ClassDB::bind_static_method(c, D_METHOD("get_rest_volume", "soft_body"), &JoltSoftBodies3D::get_rest_volume);
	ClassDB::bind_static_method(c, D_METHOD("get_gas_pressure", "soft_body"), &JoltSoftBodies3D::get_gas_pressure);

	ClassDB::bind_static_method(c, D_METHOD("skin", "soft_body", "frame", "joints", "hard"), &JoltSoftBodies3D::skin, DEFVAL(false));
	ClassDB::bind_static_method(c, D_METHOD("update", "soft_body", "delta"), &JoltSoftBodies3D::update);

	ClassDB::bind_static_method(c, D_METHOD("save_state", "soft_body"), &JoltSoftBodies3D::save_state);
	ClassDB::bind_static_method(c, D_METHOD("restore_state", "soft_body", "state"), &JoltSoftBodies3D::restore_state);
}
