/**************************************************************************/
/*  jolt_soft_bodies_3d.h                                                 */
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
#include "core/variant/typed_array.h"

class JoltSoftBody3D;

class JoltSoftBodies3D : public Object {
	GDCLASS(JoltSoftBodies3D, Object);

	static JoltSoftBody3D *_get_soft_body(const RID &p_soft_body);
	static JoltSoftBody3D *_get_built(const RID &p_soft_body);

protected:
	static void _bind_methods();

public:
	static PackedInt32Array build(const RID &p_soft_body, const Transform3D &p_frame, const PackedVector3Array &p_positions, const PackedInt32Array &p_indices, const Dictionary &p_settings);
	static bool is_built(const RID &p_soft_body);

	static void set_settings(const RID &p_soft_body, const Dictionary &p_settings);
	static Dictionary get_settings(const RID &p_soft_body);

	static PackedVector3Array get_vertices(const RID &p_soft_body);
	static PackedVector3Array get_velocities(const RID &p_soft_body);
	static float get_volume(const RID &p_soft_body);
	static float get_rest_volume(const RID &p_soft_body);
	static float get_gas_pressure(const RID &p_soft_body);

	static void skin(const RID &p_soft_body, const Transform3D &p_frame, const TypedArray<Transform3D> &p_joints, bool p_hard);
	static void update(const RID &p_soft_body, float p_delta);

	static PackedByteArray save_state(const RID &p_soft_body);
	static bool restore_state(const RID &p_soft_body, const PackedByteArray &p_state);
};
