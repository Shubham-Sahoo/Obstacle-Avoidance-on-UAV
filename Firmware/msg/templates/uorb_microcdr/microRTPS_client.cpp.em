@###############################################
@#
@# EmPy template for generating microRTPS_client.cpp file
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - msgs (List) list of all msg files
@#  - multi_topics (List) list of all multi-topic names
@#  - ids (List) list of all RTPS msg ids
@###############################################
@{
import os

import genmsg.msgs
import gencpp
from px_generate_uorb_topic_helper import * # this is in Tools/
from px_generate_uorb_topic_files import MsgScope # this is in Tools/

topic_names = [single_spec.short_name for single_spec in spec]
send_topics = [s.short_name for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND]
recv_topics = [s.short_name for idx, s in enumerate(spec) if scope[idx] == MsgScope.RECEIVE]
}@
/****************************************************************************
 *
 * Copyright (c) 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 * Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "microRTPS_transport.h"
#include "microRTPS_client.h"

#include <inttypes.h>
#include <cstdio>
#include <ctime>
#include <pthread.h>

#include <ucdr/microcdr.h>
#include <px4_time.h>
#include <uORB/uORB.h>

@[for topic in list(set(topic_names))]@
#include <uORB/topics/@(topic).h>
#include <uORB_microcdr/topics/@(topic).h>
@[end for]@

void* send(void *data);

@[if send_topics]@
void* send(void* /*unused*/)
{
    char data_buffer[BUFFER_SIZE] = {};
    uint64_t sent = 0, total_sent = 0;
    int loop = 0, read = 0;
    uint32_t length = 0;
    uint16_t header_length = 0;

    /* subscribe to topics */
    int fds[@(len(send_topics))] = {};

    // orb_set_interval statblish an update interval period in milliseconds.
@[for idx, topic in enumerate(send_topics)]@
    fds[@(idx)] = orb_subscribe(ORB_ID(@(topic)));
    orb_set_interval(fds[@(idx)], _options.update_time_ms);
@[end for]@

    // ucdrBuffer to serialize using the user defined buffer
    ucdrBuffer writer;
    header_length=transport_node->get_header_length();
    ucdr_init_buffer(&writer, (uint8_t*)&data_buffer[header_length], BUFFER_SIZE - header_length);

    struct timespec begin;
    px4_clock_gettime(CLOCK_REALTIME, &begin);

    while (!_should_exit_task)
    {
        bool updated;
@[for idx, topic in enumerate(send_topics)]@
        orb_check(fds[@(idx)], &updated);
        if (updated)
        {
            // obtained data for the file descriptor
            struct @(topic)_s data;
            // copy raw data into local buffer
            if (orb_copy(ORB_ID(@(topic)), fds[@(idx)], &data) == 0) {
                /* payload is shifted by header length to make room for header*/
                serialize_@(topic)(&writer, &data, &data_buffer[header_length], &length);

                if (0 < (read = transport_node->write((char)@(rtps_message_id(ids, topic)), data_buffer, length)))
                {
                    total_sent += read;
                    ++sent;
                }
            }
        }
@[end for]@

        px4_usleep(_options.sleep_ms*1000);
        ++loop;
    }

    struct timespec end;
    px4_clock_gettime(CLOCK_REALTIME, &end);
    double elapsed_secs = double(end.tv_sec - begin.tv_sec) + double(end.tv_nsec - begin.tv_nsec)/double(1000000000);
    PX4_INFO("SENT: %" PRIu64 " messages in %d LOOPS, %" PRIu64 " bytes in %.03f seconds - %.02fKB/s",
            sent, loop, total_sent, elapsed_secs, (double)total_sent/(1000*elapsed_secs));

    return nullptr;
}

static int launch_send_thread(pthread_t &sender_thread)
{
    pthread_attr_t sender_thread_attr;
    pthread_attr_init(&sender_thread_attr);
    pthread_attr_setstacksize(&sender_thread_attr, PX4_STACK_ADJUSTED(4000));
    struct sched_param param;
    (void)pthread_attr_getschedparam(&sender_thread_attr, &param);
    param.sched_priority = SCHED_PRIORITY_DEFAULT;
    (void)pthread_attr_setschedparam(&sender_thread_attr, &param);
    pthread_create(&sender_thread, &sender_thread_attr, send, nullptr);
    pthread_attr_destroy(&sender_thread_attr);

    return 0;
}
@[end if]@

void micrortps_start_topics(struct timespec &begin, int &total_read, uint32_t &received, int &loop)
{
@[if recv_topics]@

    char data_buffer[BUFFER_SIZE] = {};
    int read = 0;
    uint8_t topic_ID = 255;

    // Declare received topics
@[for topic in recv_topics]@
    struct @(topic)_s @(topic)_data;
    orb_advert_t @(topic)_pub = nullptr;
@[end for]@

    // ucdrBuffer to deserialize using the user defined buffer
    ucdrBuffer reader;
    ucdr_init_buffer(&reader, (uint8_t*)data_buffer, BUFFER_SIZE);
@[end if]@

    px4_clock_gettime(CLOCK_REALTIME, &begin);
    _should_exit_task = false;
@[if send_topics]@

    // create a thread for sending data to the simulator
    pthread_t sender_thread;
    launch_send_thread(sender_thread);
@[end if]@

    while (!_should_exit_task)
    {
@[if recv_topics]@
        while (0 < (read = transport_node->read(&topic_ID, data_buffer, BUFFER_SIZE)))
        {
            total_read += read;
            switch (topic_ID)
            {
@[for topic in recv_topics]@

                case @(rtps_message_id(ids, topic)):
                {
                    deserialize_@(topic)(&reader, &@(topic)_data, data_buffer);
                    if (!@(topic)_pub) {
                        @(topic)_pub = orb_advertise(ORB_ID(@(topic)), &@(topic)_data);
                    } else {
                        orb_publish(ORB_ID(@(topic)), @(topic)_pub, &@(topic)_data);
                    }
                    ++received;
                }
                break;
@[end for]@
                default:
                    PX4_WARN("Unexpected topic ID\n");
                break;
            }
        }
@[end if]@

        // loop forever if informed loop number is negative
        if (_options.loops >= 0 && loop >= _options.loops) break;

        px4_usleep(_options.sleep_ms*1000);
        ++loop;
    }
@[if send_topics]@
    _should_exit_task = true;
    pthread_join(sender_thread, nullptr);
@[end if]@
}
