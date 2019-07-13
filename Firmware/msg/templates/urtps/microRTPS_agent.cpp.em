@###############################################
@#
@# EmPy template for generating microRTPS_agent.cpp file
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
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <thread>
#include <atomic>
#include <unistd.h>
#include <poll.h>
#include <chrono>
#include <ctime>
#include <csignal>
#include <termios.h>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>
#include <fastcdr/exceptions/Exception.h>
#include <fastrtps/utils/eClock.h>
#include <fastrtps/Domain.h>

#include "microRTPS_transport.h"
#include "RtpsTopics.h"

#define BUFFER_SIZE 1024

// Default values
#define DEVICE "/dev/ttyACM0"
#define SLEEP_US 1
#define BAUDRATE B460800
#define BAUDRATE_VAL 460800
#define POLL_MS 0
#define WAIT_CNST 2
#define DEFAULT_RECV_PORT 2020
#define DEFAULT_SEND_PORT 2019

using namespace eprosima;
using namespace eprosima::fastrtps;

volatile sig_atomic_t running = 1;
Transport_node *transport_node = nullptr;
RtpsTopics topics;
uint32_t total_sent = 0, sent = 0;

struct baudtype {
    speed_t code;
    uint32_t val;
};

const baudtype baudlist[] = {
    [0] = {.code = B0, .val = 0},
    [1] = {.code = B9600, .val = 9600},
    [2] = {.code = B19200, .val = 19200},
    [3] = {.code = B38400, .val = 38400},
    [4] = {.code = B57600, .val = 57600},
    [5] = {.code = B115200, .val = 115200},
    [6] = {.code = B230400, .val = 230400},
    [7] = {.code = B460800, .val = 460800},
    [8] = {.code = B921600, .val = 921600},
};

struct options {
    enum class eTransports
    {
        UART,
        UDP
    };
    eTransports transport = options::eTransports::UART;
    char device[64] = DEVICE;
    int sleep_us = SLEEP_US;
    baudtype baudrate = {.code=BAUDRATE,.val=BAUDRATE_VAL};
    int poll_ms = POLL_MS;
    uint16_t recv_port = DEFAULT_RECV_PORT;
    uint16_t send_port = DEFAULT_SEND_PORT;
} _options;

static void usage(const char *name)
{
    printf("usage: %s [options]\n\n"
             "  -t <transport>          [UART|UDP] Default UART\n"
             "  -d <device>             UART device. Default /dev/ttyACM0\n"
             "  -w <sleep_time_us>      Time in us for which each iteration sleep. Default 1ms\n"
             "  -b <baudrate>           UART device baudrate. Default 460800\n"
             "  -p <poll_ms>            Time in ms to poll over UART. Default 1ms\n"
             "  -r <reception port>     UDP port for receiving. Default 2019\n"
             "  -s <sending port>       UDP port for sending. Default 2020\n",
             name);
}

baudtype getbaudrate(char *valstr)
{
    uint32_t baudval = strtoul(valstr, nullptr, 10);
    for (unsigned int i=1; i<sizeof(baudlist)/sizeof(baudtype); i++) {
        if (baudlist[i].val==baudval) return baudlist[i];
    }
    return baudlist[0];
}

static int parse_options(int argc, char **argv)
{
    int ch;

    while ((ch = getopt(argc, argv, "t:d:w:b:p:r:s:")) != EOF)
    {
        switch (ch)
        {
            case 't': _options.transport      = strcmp(optarg, "UDP") == 0?
                                                 options::eTransports::UDP
                                                :options::eTransports::UART;  break;
            case 'd': if (nullptr != optarg) strcpy(_options.device, optarg); break;
            case 'w': _options.sleep_us       = strtol(optarg, nullptr, 10);  break;
            case 'b': _options.baudrate       = getbaudrate(optarg);  break;
            case 'p': _options.poll_ms        = strtol(optarg, nullptr, 10);  break;
            case 'r': _options.recv_port      = strtoul(optarg, nullptr, 10); break;
            case 's': _options.send_port      = strtoul(optarg, nullptr, 10); break;
            default:
                usage(argv[0]);
            return -1;
        }
    }

    if (optind < argc)
    {
        usage(argv[0]);
        return -1;
    }

    return 0;
}

void signal_handler(int signum)
{
   printf("Interrupt signal (%d) received.\n", signum);
   running = 0;
   transport_node->close();
}

@[if recv_topics]@
std::atomic<bool> exit_sender_thread(false);
void t_send(void *data)
{
    char data_buffer[BUFFER_SIZE] = {};
    int length = 0;
    uint8_t topic_ID = 255;

    while (running)
    {
        // Send subscribed topics over UART
        while (topics.hasMsg(&topic_ID) && !exit_sender_thread.load())
        {
            uint16_t header_length = transport_node->get_header_length();
            /* make room for the header to fill in later */
            eprosima::fastcdr::FastBuffer cdrbuffer(&data_buffer[header_length], sizeof(data_buffer)-header_length);
            eprosima::fastcdr::Cdr scdr(cdrbuffer);
            if (topics.getMsg(topic_ID, scdr))
            {
                length = scdr.getSerializedDataLength();
                if (0 < (length = transport_node->write(topic_ID, data_buffer, length)))
                {
                    total_sent += length;
                    ++sent;
                }
            }
        }

        usleep(_options.sleep_us);
    }
}
@[end if]@

int main(int argc, char** argv)
{
    if (-1 == parse_options(argc, argv))
    {
        printf("EXITING...\n");
        return -1;
    }

    // register signal SIGINT and signal handler
    signal(SIGINT, signal_handler);

    switch (_options.transport)
    {
        case options::eTransports::UART:
        {
            transport_node = new UART_node(_options.device, _options.baudrate.code, _options.poll_ms);
            printf("\nUART transport: device: %s; baudrate: %d; sleep: %dus; poll: %dms\n\n",
                   _options.device, _options.baudrate.val, _options.sleep_us, _options.poll_ms);
        }
        break;
        case options::eTransports::UDP:
        {
            transport_node = new UDP_node(_options.recv_port, _options.send_port);
            printf("\nUDP transport: recv port: %u; send port: %u; sleep: %dus\n\n",
                    _options.recv_port, _options.send_port, _options.sleep_us);
        }
        break;
        default:
            printf("EXITING...\n");
        return -1;
    }

    if (0 > transport_node->init())
    {
        printf("EXITING...\n");
        return -1;
    }

    sleep(1);

@[if send_topics]@
    char data_buffer[BUFFER_SIZE] = {};
    int received = 0, loop = 0;
    int length = 0, total_read = 0;
    bool receiving = false;
    uint8_t topic_ID = 255;
    std::chrono::time_point<std::chrono::steady_clock> start, end;
@[end if]@

    topics.init();

    running = true;
@[if recv_topics]@
    std::thread sender_thread(t_send, nullptr);
@[end if]@

    while (running)
    {
@[if send_topics]@
        ++loop;
        if (!receiving) start = std::chrono::steady_clock::now();
        // Publish messages received from UART
        while (0 < (length = transport_node->read(&topic_ID, data_buffer, BUFFER_SIZE)))
        {
            topics.publish(topic_ID, data_buffer, sizeof(data_buffer));
            ++received;
            total_read += length;
            receiving = true;
            end = std::chrono::steady_clock::now();
        }

        if ((receiving && std::chrono::duration<double>(std::chrono::steady_clock::now() - end).count() > WAIT_CNST) ||
            (!running  && loop > 1))
        {
            std::chrono::duration<double>  elapsed_secs = end - start;
            printf("\nSENT:     %lu messages - %lu bytes\n",
                    (unsigned long)sent, (unsigned long)total_sent);
            printf("RECEIVED: %d messages - %d bytes; %d LOOPS - %.03f seconds - %.02fKB/s\n\n",
                    received, total_read, loop, elapsed_secs.count(), (double)total_read/(1000*elapsed_secs.count()));
            received = sent = total_read = total_sent = 0;
            receiving = false;
        }

@[end if]@
        usleep(_options.sleep_us);
    }
@[if recv_topics]@
    exit_sender_thread = true;
    sender_thread.join();
@[end if]@
    delete transport_node;
    transport_node = nullptr;

    return 0;
}
