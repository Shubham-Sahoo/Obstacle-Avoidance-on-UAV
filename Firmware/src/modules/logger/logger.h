/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#pragma once

#include "log_writer.h"
#include "messages.h"
#include <containers/Array.hpp>
#include "util.h"
#include <px4_defines.h>
#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <version/version.h>
#include <parameters/param.h>
#include <systemlib/printload.h>
#include <px4_module.h>

#include <uORB/topics/vehicle_command.h>

extern "C" __EXPORT int logger_main(int argc, char *argv[]);

#define TRY_SUBSCRIBE_INTERVAL 1000*1000	// interval in microseconds at which we try to subscribe to a topic
// if we haven't succeeded before

namespace px4
{
namespace logger
{

enum class SDLogProfileMask : int32_t {
	DEFAULT =               1 << 0,
	ESTIMATOR_REPLAY =      1 << 1,
	THERMAL_CALIBRATION =   1 << 2,
	SYSTEM_IDENTIFICATION = 1 << 3,
	HIGH_RATE =             1 << 4,
	DEBUG_TOPICS =          1 << 5,
	SENSOR_COMPARISON =     1 << 6,
	VISION_AND_AVOIDANCE =  1 << 7
};

enum class MissionLogType : int32_t {
	Disabled =               0,
	Complete =               1,
	Geotagging =             2
};

inline bool operator&(SDLogProfileMask a, SDLogProfileMask b)
{
	return static_cast<int32_t>(a) & static_cast<int32_t>(b);
}

struct LoggerSubscription {
	int fd[ORB_MULTI_MAX_INSTANCES]; ///< uorb subscription. The first fd is also used to store the interval if
	/// not subscribed yet (-interval - 1)
	const orb_metadata *metadata = nullptr;
	uint8_t msg_ids[ORB_MULTI_MAX_INSTANCES];

	LoggerSubscription() {}

	LoggerSubscription(int fd_, const orb_metadata *metadata_) :
		metadata(metadata_)
	{
		fd[0] = fd_;

		for (int i = 1; i < ORB_MULTI_MAX_INSTANCES; i++) {
			fd[i] = -1;
		}

		for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
			msg_ids[i] = (uint8_t) - 1;
		}
	}
};

class Logger : public ModuleBase<Logger>
{
public:
	enum class LogMode {
		while_armed = 0,
		boot_until_disarm,
		boot_until_shutdown,
		rc_aux1
	};

	Logger(LogWriter::Backend backend, size_t buffer_size, uint32_t log_interval, const char *poll_topic_name,
	       LogMode log_mode, bool log_name_timestamp, unsigned int queue_size);

	~Logger();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Logger *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/**
	 * Tell the logger that we're in replay mode. This must be called
	 * before starting the logger.
	 * @param file_name file name of the used log replay file. Will be copied.
	 */
	void setReplayFile(const char *file_name);

	/**
	 * Add a topic to be logged. This must be called before start_log()
	 * (because it does not write an ADD_LOGGED_MSG message).
	 * @param name topic name
	 * @param interval limit rate if >0, otherwise log as fast as the topic is updated.
	 * @return true on success
	 */
	bool add_topic(const char *name, unsigned interval = 0);

	/**
	 * add a logged topic (called by add_topic() above).
	 * In addition, it subscribes to the first instance of the topic, if it's advertised,
	 * and sets the file descriptor of LoggerSubscription accordingly
	 * @return the newly added subscription on success, nullptr otherwise
	 */
	LoggerSubscription *add_topic(const orb_metadata *topic);

	/**
	 * request the logger thread to stop (this method does not block).
	 * @return true if the logger is stopped, false if (still) running
	 */
	static bool request_stop_static();

	void print_statistics(LogType type);

	void set_arm_override(bool override) { _manually_logging_override = override; }

private:

	enum class PrintLoadReason {
		Preflight,
		Postflight,
		Watchdog
	};

	static constexpr size_t 	MAX_TOPICS_NUM = 64; /**< Maximum number of logged topics */
	static constexpr int		MAX_MISSION_TOPICS_NUM = 5; /**< Maximum number of mission topics */
	static constexpr unsigned	MAX_NO_LOGFILE = 999;	/**< Maximum number of log files */
	static constexpr const char	*LOG_ROOT[(int)LogType::Count] = {
		PX4_STORAGEDIR "/log",
		PX4_STORAGEDIR "/mission_log"
	};

	struct LogFileName {
		char log_dir[12];           ///< e.g. "2018-01-01" or "sess001"
		int sess_dir_index{1};      ///< search starting index for 'sess<i>' directory name
		char log_file_name[31];     ///< e.g. "log001.ulg" or "12_09_00_replayed.ulg"
		bool has_log_dir{false};
	};

	struct Statistics {
		hrt_abstime start_time_file{0};				///< Time when logging started, file backend (not the logger thread)
		hrt_abstime dropout_start{0};				///< start of current dropout (0 = no dropout)
		float max_dropout_duration{0.0f};			///< max duration of dropout [s]
		size_t write_dropouts{0};				///< failed buffer writes due to buffer overflow
		size_t high_water{0};					///< maximum used write buffer
	};

	struct MissionSubscription {
		unsigned min_delta_ms;        ///< minimum time between 2 topic writes [ms]
		unsigned next_write_time;     ///< next time to write in 0.1 seconds
	};

	/**
	 * Write an ADD_LOGGED_MSG to the log for a all current subscriptions and instances
	 */
	void write_all_add_logged_msg(LogType type);

	/**
	 * Write an ADD_LOGGED_MSG to the log for a given subscription and instance.
	 * _writer.lock() must be held when calling this.
	 */
	void write_add_logged_msg(LogType type, LoggerSubscription &subscription, int instance);

	/**
	 * Create logging directory
	 * @param type
	 * @param tt if not null, use it for the directory name
	 * @param log_dir returned log directory path
	 * @param log_dir_len log_dir buffer length
	 * @return string length of log_dir (excluding terminating null-char), <0 on error
	 */
	int create_log_dir(LogType type, tm *tt, char *log_dir, int log_dir_len);

	/**
	 * Get log file name with directory (create it if necessary)
	 */
	int get_log_file_name(LogType type, char *file_name, size_t file_name_size);

	void start_log_file(LogType type);

	void stop_log_file(LogType type);

	void start_log_mavlink();

	void stop_log_mavlink();

	/** check if mavlink logging can be started */
	bool can_start_mavlink_log() const
	{
		return !_writer.is_started(LogType::Full, LogWriter::BackendMavlink)
		       && (_writer.backend() & LogWriter::BackendMavlink) != 0;
	}

	/** get the configured backend as string */
	const char *configured_backend_mode() const;

	/**
	 * write the file header with file magic and timestamp.
	 */
	void write_header(LogType type);

	/// Array to store written formats (add some more for nested definitions)
	using WrittenFormats = Array < const orb_metadata *, MAX_TOPICS_NUM + 10 >;

	void write_format(LogType type, const orb_metadata &meta, WrittenFormats &written_formats, ulog_message_format_s &msg,
			  int level = 1);
	void write_formats(LogType type);

	/**
	 * write performance counters
	 * @param preflight preflight if true, postflight otherwise
	 */
	void write_perf_data(bool preflight);

	/**
	 * write bootup console output
	 */
	void write_console_output();

	/**
	 * callback to write the performance counters
	 */
	static void perf_iterate_callback(perf_counter_t handle, void *user);

	/**
	 * callback for print_load_buffer() to print the process load
	 */
	static void print_load_callback(void *user);

	void write_version(LogType type);

	void write_info(LogType type, const char *name, const char *value);
	void write_info_multiple(LogType type, const char *name, const char *value, bool is_continued);
	void write_info(LogType type, const char *name, int32_t value);
	void write_info(LogType type, const char *name, uint32_t value);

	/** generic common template method for write_info variants */
	template<typename T>
	void write_info_template(LogType type, const char *name, T value, const char *type_str);

	void write_parameters(LogType type);

	void write_changed_parameters(LogType type);

	inline bool copy_if_updated_multi(int sub_idx, int multi_instance, void *buffer, bool try_to_subscribe);

	/**
	 * Check if a topic instance exists and subscribe to it
	 * @return true when topic exists and subscription successful
	 */
	bool try_to_subscribe_topic(LoggerSubscription &sub, int multi_instance);

	/**
	 * Write exactly one ulog message to the logger and handle dropouts.
	 * Must be called with _writer.lock() held.
	 * @return true if data written, false otherwise (on overflow)
	 */
	bool write_message(LogType type, void *ptr, size_t size);

	/**
	 * Parse a file containing a list of uORB topics to log, calling add_topic for each
	 * @param fname name of file
	 * @return number of topics added
	 */
	int add_topics_from_file(const char *fname);

	/**
	 * Add topic subscriptions based on the configured mission log type
	 */
	void initialize_mission_topics(MissionLogType type);

	/**
	 * Add a topic to be logged for the mission log (it's also added to the full log).
	 * The interval is expected to be 0 or large (in the order of 0.1 seconds or higher).
	 * Must be called before all other topics are added.
	 * @param name topic name
	 * @param interval limit rate if >0 [ms], otherwise log as fast as the topic is updated.
	 */
	void add_mission_topic(const char *name, unsigned interval = 0);

	/**
	 * Add topic subscriptions based on the _sdlog_profile_handle parameter
	 */
	void initialize_configured_topics();

	void add_default_topics();
	void add_estimator_replay_topics();
	void add_thermal_calibration_topics();
	void add_system_identification_topics();
	void add_high_rate_topics();
	void add_debug_topics();
	void add_sensor_comparison_topics();
	void add_vision_and_avoidance_topics();

	/**
	 * check current arming state or aux channel and start/stop logging if state changed and according to
	 * configured params.
	 * @param vehicle_status_sub
	 * @param manual_control_sp_sub
	 * @param mission_log_type
	 * @return true if log started
	 */
	bool start_stop_logging(int vehicle_status_sub, int manual_control_sp_sub, MissionLogType mission_log_type);

	void handle_vehicle_command_update(int vehicle_command_sub, orb_advert_t &vehicle_command_ack_pub);
	void ack_vehicle_command(orb_advert_t &vehicle_command_ack_pub, vehicle_command_s *cmd, uint32_t result);

	/**
	 * initialize the output for the process load, so that ~1 second later it will be written to the log
	 */
	void initialize_load_output(PrintLoadReason reason);

	/**
	 * write the process load, which was previously initialized with initialize_load_output()
	 */
	void write_load_output();

	/**
	 * Regularly print the buffer fill state (only if DBGPRINT is set)
	 * @param total_bytes total written bytes (to the full file), will be reset on each print
	 * @param timer_start time since last print
	 */
	inline void debug_print_buffer(uint32_t &total_bytes, hrt_abstime &timer_start);


	uint8_t						*_msg_buffer{nullptr};
	int						_msg_buffer_len{0};

	LogFileName					_file_name[(int)LogType::Count];

	bool						_prev_state{false}; ///< previous state depending on logging mode (arming or aux1 state)
	bool						_manually_logging_override{false};

	Statistics					_statistics[(int)LogType::Count];
	hrt_abstime					_last_sync_time{0}; ///< last time a sync msg was sent

	LogMode						_log_mode;
	const bool					_log_name_timestamp;

	Array<LoggerSubscription, MAX_TOPICS_NUM>	_subscriptions; ///< all subscriptions for full & mission log (in front)
	MissionSubscription 				_mission_subscriptions[MAX_MISSION_TOPICS_NUM]; ///< additional data for mission subscriptions
	int						_num_mission_subs{0};

	LogWriter					_writer;
	uint32_t					_log_interval{0};
	const orb_metadata				*_polling_topic_meta{nullptr}; ///< if non-null, poll on this topic instead of sleeping
	orb_advert_t					_mavlink_log_pub{nullptr};
	uint8_t						_next_topic_id{0}; ///< id of next subscribed ulog topic
	char						*_replay_file_name{nullptr};
	bool						_should_stop_file_log{false}; /**< if true _next_load_print is set and file logging
											will be stopped after load printing (for the full log) */
	print_load_s					_load{}; ///< process load data
	hrt_abstime					_next_load_print{0}; ///< timestamp when to print the process load
	PrintLoadReason					_print_load_reason {PrintLoadReason::Preflight};

	param_t						_sdlog_profile_handle{PARAM_INVALID};
	param_t						_log_utc_offset{PARAM_INVALID};
	param_t						_log_dirs_max{PARAM_INVALID};
	param_t						_mission_log{PARAM_INVALID};
};

} //namespace logger
} //namespace px4
