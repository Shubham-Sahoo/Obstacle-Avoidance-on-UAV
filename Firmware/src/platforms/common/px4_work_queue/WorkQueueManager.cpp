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

#include "WorkQueueManager.hpp"

#include "WorkQueue.hpp"

#include <drivers/drv_hrt.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <px4_atomic.h>
#include <containers/BlockingList.hpp>
#include <containers/BlockingQueue.hpp>
#include <lib/drivers/device/Device.hpp>
#include <lib/mathlib/mathlib.h>

#include <limits.h>
#include <string.h>

using namespace time_literals;

namespace px4
{

// list of current work queues
static BlockingList<WorkQueue *> *_wq_manager_wqs_list{nullptr};

// queue of WorkQueues to be created (as threads in the wq manager task)
static BlockingQueue<const wq_config_t *, 1> *_wq_manager_create_queue{nullptr};

static px4::atomic_bool _wq_manager_should_exit{false};


static WorkQueue *FindWorkQueueByName(const char *name)
{
	if (_wq_manager_wqs_list == nullptr) {
		PX4_ERR("not running");
		return nullptr;
	}

	auto lg = _wq_manager_wqs_list->getLockGuard();

	// search list
	for (WorkQueue *wq : *_wq_manager_wqs_list) {
		if (strcmp(wq->get_name(), name) == 0) {
			return wq;
		}
	}

	return nullptr;
}

WorkQueue *WorkQueueFindOrCreate(const wq_config_t &new_wq)
{
	if (_wq_manager_create_queue == nullptr) {
		PX4_ERR("not running");
		return nullptr;
	}

	// search list for existing work queue
	WorkQueue *wq = FindWorkQueueByName(new_wq.name);

	// create work queue if it doesn't already exist
	if (wq == nullptr) {
		// add WQ config to list
		//  main thread wakes up, creates the thread
		_wq_manager_create_queue->push(&new_wq);

		// we wait until new wq is created, then return
		uint64_t t = 0;

		while (wq == nullptr && t < 10_s) {
			// Wait up to 10 seconds, checking every 1 ms
			t += 1_ms;
			px4_usleep(1_ms);

			wq = FindWorkQueueByName(new_wq.name);
		}

		if (wq == nullptr) {
			PX4_ERR("failed to create %s", new_wq.name);
		}
	}

	return wq;
}

const wq_config_t &device_bus_to_wq(uint32_t device_id_int)
{
	union device::Device::DeviceId device_id;
	device_id.devid = device_id_int;

	const device::Device::DeviceBusType bus_type = device_id.devid_s.bus_type;
	const uint8_t bus = device_id.devid_s.bus;

	if (bus_type == device::Device::DeviceBusType_I2C) {
		switch (bus) {
		case 1: return wq_configurations::I2C1;

		case 2: return wq_configurations::I2C2;

		case 3: return wq_configurations::I2C3;

		case 4: return wq_configurations::I2C4;
		}

	} else if (bus_type == device::Device::DeviceBusType_SPI) {
		switch (bus) {
		case 1: return wq_configurations::SPI1;

		case 2: return wq_configurations::SPI2;

		case 3: return wq_configurations::SPI3;

		case 4: return wq_configurations::SPI4;

		case 5: return wq_configurations::SPI5;

		case 6: return wq_configurations::SPI6;
		}
	}

	// otherwise use high priority
	return wq_configurations::hp_default;
};

static void *WorkQueueRunner(void *context)
{
	wq_config_t *config = static_cast<wq_config_t *>(context);
	WorkQueue wq(*config);

	// add to work queue list
	_wq_manager_wqs_list->add(&wq);

	wq.Run();

	// remove from work queue list
	_wq_manager_wqs_list->remove(&wq);

	return nullptr;
}

static void WorkQueueManagerRun()
{
	_wq_manager_wqs_list = new BlockingList<WorkQueue *>();
	_wq_manager_create_queue = new BlockingQueue<const wq_config_t *, 1>();

	while (!_wq_manager_should_exit.load()) {
		// create new work queues as needed
		const wq_config_t *wq = _wq_manager_create_queue->pop();

		if (wq != nullptr) {
			// create new work queue

			pthread_attr_t attr;
			int ret_attr_init = pthread_attr_init(&attr);

			if (ret_attr_init != 0) {
				PX4_ERR("attr init for %s failed (%i)", wq->name, ret_attr_init);
			}

			sched_param param;
			int ret_getschedparam = pthread_attr_getschedparam(&attr, &param);

			if (ret_getschedparam != 0) {
				PX4_ERR("getting sched param for %s failed (%i)", wq->name, ret_getschedparam);
			}

			// stack size
#ifndef __PX4_QURT
			const size_t stacksize = math::max(PTHREAD_STACK_MIN, PX4_STACK_ADJUSTED(wq->stacksize));
#else
			const size_t stacksize = math::max(8 * 1024, PX4_STACK_ADJUSTED(wq->stacksize));
#endif
			int ret_setstacksize = pthread_attr_setstacksize(&attr, stacksize);

			if (ret_setstacksize != 0) {
				PX4_ERR("setting stack size for %s failed (%i)", wq->name, ret_setstacksize);
			}

#ifndef __PX4_QURT
			// schedule policy FIFO
			int ret_setschedpolicy = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

			if (ret_setschedpolicy != 0) {
				PX4_ERR("failed to set sched policy SCHED_FIFO (%i)", ret_setschedpolicy);
			}

#endif // ! QuRT

			// priority
			param.sched_priority = sched_get_priority_max(SCHED_FIFO) + wq->relative_priority;
			int ret_setschedparam = pthread_attr_setschedparam(&attr, &param);

			if (ret_setschedparam != 0) {
				PX4_ERR("setting sched params for %s failed (%i)", wq->name, ret_setschedparam);
			}

			// create thread
			pthread_t thread;
			int ret_create = pthread_create(&thread, &attr, WorkQueueRunner, (void *)wq);

			if (ret_create == 0) {
				PX4_INFO("creating: %s, priority: %d, stack: %zu bytes", wq->name, param.sched_priority, stacksize);

			} else {
				PX4_ERR("failed to create thread for %s (%i): %s", wq->name, ret_create, strerror(ret_create));
			}

			// destroy thread attributes
			int ret_destroy = pthread_attr_destroy(&attr);

			if (ret_destroy != 0) {
				PX4_ERR("failed to destroy thread attributes for %s (%i)", wq->name, ret_create);
			}
		}
	}
}

int WorkQueueManagerStart()
{
	int task_id = px4_task_spawn_cmd("wq:manager",
					 SCHED_DEFAULT,
					 PX4_WQ_HP_BASE,
					 1200,
					 (px4_main_t)&WorkQueueManagerRun,
					 nullptr);

	if (task_id < 0) {
		PX4_ERR("task start failed (%i)", task_id);
		return -errno;
	}

	return 0;
}

int WorkQueueManagerStop()
{
	if (_wq_manager_wqs_list != nullptr) {
		auto lg = _wq_manager_wqs_list->getLockGuard();

		// ask all work queues (threads) to stop
		// NOTE: not currently safe without all WorkItems stopping first
		for (WorkQueue *wq : *_wq_manager_wqs_list) {
			wq->request_stop();
		}

		delete _wq_manager_wqs_list;
	}

	_wq_manager_should_exit.store(true);

	if (_wq_manager_create_queue != nullptr) {
		// push nullptr to wake the wq manager task
		_wq_manager_create_queue->push(nullptr);

		px4_usleep(1000);

		delete _wq_manager_create_queue;
	}

	return PX4_OK;
}

} // namespace px4
