/**************************************************************************/
/*  rendering_server_waits.h                                              */
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

#include "core/os/mutex.h"
#include "core/os/os.h"
#include "core/templates/hash_map.h"
#include "core/variant/dictionary.h"

class RenderingServerWaits {
	struct Total {
		uint64_t count = 0;
		uint64_t usec = 0;
		uint64_t pool_count = 0;
		uint64_t pool_usec = 0;
	};

	static BinaryMutex mutex;
	static HashMap<String, Total> totals;

public:
	static bool is_waiting_thread();
	static void record(const char *p_server, const char *p_where, uint64_t p_from_usec);
	static Dictionary get_thread_waits();
	static void clear_thread_waits();

	class Scope {
		const char *server;
		const char *where;
		uint64_t from;

	public:
		_FORCE_INLINE_ Scope(const char *p_server, const char *p_where) :
				server(p_server), where(p_where), from(OS::get_singleton()->get_ticks_usec()) {}
		_FORCE_INLINE_ ~Scope() { record(server, where, from); }
	};

	template <typename M>
	class Lock {
		const M &lock_mutex;

	public:
		_FORCE_INLINE_ Lock(const M &p_mutex, const char *p_where) :
				lock_mutex(p_mutex) {
			if (lock_mutex.try_lock()) {
				return;
			}
			uint64_t from = OS::get_singleton()->get_ticks_usec();
			lock_mutex.lock();
			if (is_waiting_thread()) {
				record("RenderingDevice", p_where, from);
			}
		}
		_FORCE_INLINE_ ~Lock() { lock_mutex.unlock(); }
	};
};
