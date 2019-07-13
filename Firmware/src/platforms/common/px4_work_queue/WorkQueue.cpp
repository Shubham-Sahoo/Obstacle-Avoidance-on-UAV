/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "WorkQueue.hpp"
#include "WorkItem.hpp"

#include <string.h>

#include <px4_tasks.h>
#include <px4_time.h>
#include <drivers/drv_hrt.h>

namespace px4
{

WorkQueue::WorkQueue(const wq_config_t &config) :
	_config(config)
{
	// set the threads name
#ifdef __PX4_DARWIN
	pthread_setname_np(_config.name);
#elif !defined(__PX4_QURT)
	pthread_setname_np(pthread_self(), _config.name);
#endif

#ifndef __PX4_NUTTX
	px4_sem_init(&_qlock, 0, 1);
#endif /* __PX4_NUTTX */

	px4_sem_init(&_process_lock, 0, 0);
	px4_sem_setprotocol(&_process_lock, SEM_PRIO_NONE);
}

WorkQueue::~WorkQueue()
{
	work_lock();
	px4_sem_destroy(&_process_lock);
	work_unlock();

#ifndef __PX4_NUTTX
	px4_sem_destroy(&_qlock);
#endif /* __PX4_NUTTX */
}

void WorkQueue::Add(WorkItem *item)
{
	// TODO: prevent additions when shutting down

	work_lock();
	_q.push(item);
	work_unlock();

	// Wake up the worker thread
	px4_sem_post(&_process_lock);
}

void WorkQueue::Remove(WorkItem *item)
{
	work_lock();
	_q.remove(item);
	work_unlock();
}

void WorkQueue::Clear()
{
	work_lock();

	while (!_q.empty()) {
		_q.pop();
	}

	work_unlock();
}

void WorkQueue::Run()
{
	while (!should_exit()) {
		px4_sem_wait(&_process_lock);

		work_lock();

		// process queued work
		while (!_q.empty()) {
			WorkItem *work = _q.pop();

			work_unlock(); // unlock work queue to run (item may requeue itself)
			work->Run();
			work_lock(); // re-lock
		}

		work_unlock();
	}
}

void WorkQueue::print_status()
{
	PX4_INFO("WorkQueue: %s running", get_name());
}

} // namespace px4
