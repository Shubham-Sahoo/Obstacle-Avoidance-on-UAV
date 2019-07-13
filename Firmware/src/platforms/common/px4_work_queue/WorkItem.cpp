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

#include "WorkItem.hpp"

#include "WorkQueue.hpp"
#include "WorkQueueManager.hpp"

#include <px4_log.h>
#include <drivers/drv_hrt.h>

namespace px4
{

WorkItem::WorkItem(const wq_config_t &config)
{
	if (!Init(config)) {
		PX4_ERR("init failed");
	}
}

WorkItem::~WorkItem()
{
	Deinit();
}

bool WorkItem::Init(const wq_config_t &config)
{
	// clear any existing first
	Deinit();

	px4::WorkQueue *wq = WorkQueueFindOrCreate(config);

	if (wq == nullptr) {
		PX4_ERR("%s not available", config.name);

	} else {
		_wq = wq;
		return true;
	}

	return false;
}

void WorkItem::Deinit()
{
	// remove any currently queued work
	if (_wq != nullptr) {
		// prevent additional insertions
		px4::WorkQueue *wq_temp = _wq;
		_wq = nullptr;

		wq_temp->Remove(this);
	}
}

} // namespace px4
