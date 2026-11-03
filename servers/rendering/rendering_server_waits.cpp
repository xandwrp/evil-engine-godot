/**************************************************************************/
/*  rendering_server_waits.cpp                                            */
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

#include "rendering_server_waits.h"

#include "core/object/worker_thread_pool.h"
#include "core/variant/typed_array.h"
#include "servers/rendering/rendering_server.h"

BinaryMutex RenderingServerWaits::mutex;
HashMap<String, RenderingServerWaits::Total> RenderingServerWaits::totals;
SafeNumeric<int64_t> RenderingServerWaits::pipelines_compiling;
SafeNumeric<int64_t> RenderingServerWaits::shaders_compiling;

bool RenderingServerWaits::is_waiting_thread() {
	RenderingServer *server = RenderingServer::get_singleton();
	return server != nullptr && !server->is_on_render_thread();
}

void RenderingServerWaits::record(const char *p_server, const char *p_where, uint64_t p_from_usec) {
	uint64_t usec = OS::get_singleton()->get_ticks_usec() - p_from_usec;
	String key = String(p_server) + "." + String(p_where);
	bool pool = WorkerThreadPool::get_singleton() != nullptr && WorkerThreadPool::get_singleton()->get_thread_index() >= 0;
	MutexLock lock(mutex);
	Total &total = totals[key];
	total.count++;
	total.usec += usec;
	if (pool) {
		total.pool_count++;
		total.pool_usec += usec;
	}
}

Dictionary RenderingServerWaits::get_thread_waits() {
	MutexLock lock(mutex);
	Dictionary waits;
	for (const KeyValue<String, Total> &E : totals) {
		PackedInt64Array total;
		total.push_back(int64_t(E.value.count));
		total.push_back(int64_t(E.value.usec));
		total.push_back(int64_t(E.value.pool_count));
		total.push_back(int64_t(E.value.pool_usec));
		waits[E.key] = total;
	}
	return waits;
}

void RenderingServerWaits::clear_thread_waits() {
	MutexLock lock(mutex);
	totals.clear();
}

int64_t RenderingServerWaits::get_pipeline_compilations_in_flight() {
	return pipelines_compiling.get();
}

int64_t RenderingServerWaits::get_shader_compilations_in_flight() {
	return shaders_compiling.get();
}
