/****************************************************************************
 *
 *   Copyright (c) 2015-2018 PX4 Development Team. All rights reserved.
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

/**
 * Motor Ordering
 *
 * Determines the motor ordering. This can be used for example in combination with
 * a 4-in-1 ESC that assumes a motor ordering which is different from PX4.
 *
 * ONLY supported for Quads.
 * ONLY supported for fmu output (Pixracer or Omnibus F4).
 *
 * When changing this, make sure to test the motor response without props first.
 *
 * @value 0 PX4
 * @value 1 Betaflight / Cleanflight
 *
 * @min 0
 * @max 1
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(MOT_ORDERING, 0);

/**
 * Run the FMU as a task to reduce latency
 *
 * If true, the FMU will run in a separate task instead of on the work queue.
 * Set this if low latency is required, for example for racing.
 *
 * This is a trade-off between RAM usage and latency: running as a task, it
 * requires a separate stack and directly polls on the control topics, whereas
 * running on the work queue, it runs at a fixed update rate.
 *
 * @boolean
 * @reboot_required true
 * @group System
 */
PARAM_DEFINE_INT32(SYS_FMU_TASK, 1);
