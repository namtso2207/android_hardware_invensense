#include <cstdint>
#include <cinttypes>
#include <ctime>
#include <unistd.h>
#include <iostream>
#include <unordered_map>

#include "SensorChannel.h"
#include "SensorDirectChannel.h"
#include "SensorControl.h"

#define INVN_SENSORCONTROL_VERSION_STR		"1.2.2"

// Add missing sensors definitions
#ifndef ASENSOR_TYPE_META_DATA
#  define ASENSOR_TYPE_META_DATA		0
#endif
#ifndef ASENSOR_TYPE_ORIENTATION
#  define ASENSOR_TYPE_ORIENTATION		3
#endif
#ifndef ASENSOR_TYPE_PRESSURE
#  define ASENSOR_TYPE_PRESSURE			6
#endif
#ifndef ASENSOR_TYPE_GRAVITY
#  define ASENSOR_TYPE_GRAVITY			9
#endif
#ifndef ASENSOR_TYPE_LINEAR_ACCELERATION
#  define ASENSOR_TYPE_LINEAR_ACCELERATION	10
#endif
#ifndef ASENSOR_TYPE_ROTATION_VECTOR
#  define ASENSOR_TYPE_ROTATION_VECTOR		11
#endif
#ifndef ASENSOR_TYPE_RELATIVE_HUMIDITY
#  define ASENSOR_TYPE_RELATIVE_HUMIDITY	12
#endif
#ifndef ASENSOR_TYPE_AMBIENT_TEMPERATURE
#  define ASENSOR_TYPE_AMBIENT_TEMPERATURE	13
#endif
#ifndef ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED
#  define ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED	14
#endif
#ifndef ASENSOR_TYPE_GAME_ROTATION_VECTOR
#  define ASENSOR_TYPE_GAME_ROTATION_VECTOR	15
#endif
#ifndef ASENSOR_TYPE_GYROSCOPE_UNCALIBRATED
#  define ASENSOR_TYPE_GYROSCOPE_UNCALIBRATED	16
#endif
#ifndef ASENSOR_TYPE_SIGNIFICANT_MOTION
#  define ASENSOR_TYPE_SIGNIFICANT_MOTION	17
#endif
#ifndef ASENSOR_TYPE_STEP_DETECTOR
#  define ASENSOR_TYPE_STEP_DETECTOR		18
#endif
#ifndef ASENSOR_TYPE_STEP_COUNTER
#  define ASENSOR_TYPE_STEP_COUNTER		19
#endif
#ifndef ASENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR
#  define ASENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR	20
#endif
#ifndef ASENSOR_TYPE_HEART_RATE
#  define ASENSOR_TYPE_HEART_RATE		21
#endif
#ifndef ASENSOR_TYPE_TILT_DETECTOR
#  define ASENSOR_TYPE_TILT_DETECTOR		22
#endif
#ifndef ASENSOR_TYPE_WAKE_GESTURE
#  define ASENSOR_TYPE_WAKE_GESTURE		23
#endif
#ifndef ASENSOR_TYPE_GLANCE_GESTURE
#  define ASENSOR_TYPE_GLANCE_GESTURE		24
#endif
#ifndef ASENSOR_TYPE_PICK_UP_GESTURE
#  define ASENSOR_TYPE_PICK_UP_GESTURE		25
#endif
#ifndef ASENSOR_TYPE_WRIST_TILT_GESTURE
#  define ASENSOR_TYPE_WRIST_TILT_GESTURE	26
#endif
#ifndef ASENSOR_TYPE_DEVICE_ORIENTATION
#  define ASENSOR_TYPE_DEVICE_ORIENTATION	27
#endif
#ifndef ASENSOR_TYPE_POSE_6DOF
#  define ASENSOR_TYPE_POSE_6DOF		28
#endif
#ifndef ASENSOR_TYPE_STATIONARY_DETECT
#  define ASENSOR_TYPE_STATIONARY_DETECT	29
#endif
#ifndef ASENSOR_TYPE_MOTION_DETECT
#  define ASENSOR_TYPE_MOTION_DETECT		30
#endif
#ifndef ASENSOR_TYPE_HEART_BEAT
#  define ASENSOR_TYPE_HEART_BEAT		31
#endif
#ifndef ASENSOR_TYPE_DYNAMIC_SENSOR_META
#  define ASENSOR_TYPE_DYNAMIC_SENSOR_META	32
#endif
#ifndef ASENSOR_TYPE_ADDITIONAL_INFO
#  define ASENSOR_TYPE_ADDITIONAL_INFO		33
#endif
#ifndef ASENSOR_TYPE_LOW_LATENCY_OFFBODY_DETECT
#  define ASENSOR_TYPE_LOW_LATENCY_OFFBODY_DETECT	34
#endif
#ifndef ASENSOR_TYPE_ACCELEROMETER_UNCALIBRATED
#  define ASENSOR_TYPE_ACCELEROMETER_UNCALIBRATED	35
#endif
#ifndef ASENSOR_TYPE_HINGE_ANGLE
#  define ASENSOR_TYPE_HINGE_ANGLE	36
#endif
#ifndef ASENSOR_TYPE_HEAD_TRACKER
#  define ASENSOR_TYPE_HEAD_TRACKER	37
#endif
#ifndef ASENSOR_TYPE_ACCELEROMETER_LIMITED_AXES
#  define ASENSOR_TYPE_ACCELEROMETER_LIMITED_AXES	38
#endif
#ifndef ASENSOR_TYPE_GYROSCOPE_LIMITED_AXES
#  define ASENSOR_TYPE_GYROSCOPE_LIMITED_AXES	39
#endif
#ifndef ASENSOR_TYPE_ACCELEROMETER_LIMITED_AXES_UNCALIBRATED
#  define ASENSOR_TYPE_ACCELEROMETER_LIMITED_AXES_UNCALIBRATED	40
#endif
#ifndef ASENSOR_TYPE_GYROSCOPE_LIMITED_AXES_UNCALIBRATED
#  define ASENSOR_TYPE_GYROSCOPE_LIMITED_AXES_UNCALIBRATED		41
#endif
#ifndef ASENSOR_TYPE_HEADING
#  define ASENSOR_TYPE_HEADING		42
#endif

struct LastTimestamps {
	int64_t lastBatchTs;
	int64_t lastEventTs;
};

struct InvnSensorControl {
	SensorControl *control;
	int64_t startTimestamp;
	std::unordered_map<int, LastTimestamps> lastTimestamps[SensorControl::CHANNEL_ID_NB];
};

static const char *getReportingModeString(int mode)
{
	switch (mode) {
	case AREPORTING_MODE_CONTINUOUS:
		return "continuous";
	case AREPORTING_MODE_ON_CHANGE:
		return "on-change";
	case AREPORTING_MODE_ONE_SHOT:
		return "one-shot";
	case AREPORTING_MODE_SPECIAL_TRIGGER:
		return "special-trigger";
	case AREPORTING_MODE_INVALID:
	default:
		return "invalid";
	}
}

static inline int64_t getTime()
{
	struct timespec ts;
	clock_gettime(CLOCK_BOOTTIME, &ts);

	int64_t time = static_cast<int64_t>(ts.tv_sec) * static_cast<int64_t>(1000000000LL) + static_cast<int64_t>(ts.tv_nsec);

	return time;
}

static void printUsage(const char *name)
{
	std::cout << "\nusage: " << name << " [-hVplv] [-t timeout] [id1,rate1[,latency1[,type1]] id2,rate2[,latency2[,type2]] ...]\n\n";

	std::cout << "parameters: id,rate[,latency[,type]] ...\n"
		"\tid: sensor id found in the displayed sensors list\n"
		"\trate: sensor rate in Hz integer\n"
		"\t      or direct report rate 1 (normal), 2 (fast), 3 (very-fast)\n"
		"\tlatency: sensor latency in ms decimal (default 0)\n"
		"\ttype: sensor data report type\n"
			"\t\t0 => normal (default)\n"
			"\t\t1 => direct report using shared memory\n"
			"\t\t2 => direct report using hardware buffer\n"
		"\tBeware that you need to specify latency if you want to set report type!\n";

	std::cout << "\noptions:\n"
		"-h\tdisplay this help\n"
		"-V\tdisplay version\n"
		"-p\tprint sensor data\n"
		"-l\tlist available sensors\n"
		"-v\tverbose mode\n"
		"-t\ttimeout in ms\n";

	std::cout << std::endl;
}

static void printVersion(const char *name)
{
	std::cout << name << " version " << INVN_SENSORCONTROL_VERSION_STR << std::endl;
}

static void printSensorList(const ASensorList sensorList, size_t sensorNb, bool verbose)
{
	std::cout << "\nSensors' list:\n";

	for (size_t i = 0; i < sensorNb; ++i) {
		ASensorRef sensor = sensorList[i];
		bool directChannelSharedMemory = ASensor_isDirectChannelTypeSupported(sensor, ASENSOR_DIRECT_CHANNEL_TYPE_SHARED_MEMORY);
		bool directChannelHardwareBuffer = ASensor_isDirectChannelTypeSupported(sensor, ASENSOR_DIRECT_CHANNEL_TYPE_HARDWARE_BUFFER);
		printf("\t%2zu -> type %2d, name '%s'\n", (i + 1), ASensor_getType(sensor), ASensor_getName(sensor));
		if (!verbose) {
			continue;
		}
		printf("\t\tvendor: '%s', type: '%s'\n", ASensor_getVendor(sensor), ASensor_getStringType(sensor));
		printf("\t\treporting: %s", getReportingModeString(ASensor_getReportingMode(sensor)));
		if (ASensor_isWakeUpSensor(sensor)) {
			printf(", wake-up");
		}
		printf("\n");
		printf("\t\tdirectReport: ");
		if (!directChannelSharedMemory && !directChannelHardwareBuffer) {
			printf("none\n");
		} else {
			if (directChannelSharedMemory) {
				printf("sharedMemory - ");
			}
			if (directChannelHardwareBuffer) {
				printf("hardwareBuffer - ");
			}
			printf("maxRate=%s\n", SensorDirectChannel::getRateString(ASensor_getHighestDirectReportRateLevel(sensor)));
		}
		printf("\t\tresolution: %f, minDelay: %dus\n", ASensor_getResolution(sensor), ASensor_getMinDelay(sensor));
		printf("\t\tfifo: max %d, reserved %d\n", ASensor_getFifoMaxEventCount(sensor), ASensor_getFifoReservedEventCount(sensor));
	}
}

static ASensorRef getSensor(const InvnSensorControl *st, int32_t sensor)
{
	ASensorList sensorList = st->control->getSensorList();
	size_t sensorNb = st->control->getSensorNb();
	int32_t handle;

	for (size_t i = 0; i < sensorNb; ++i) {
		handle = ASensor_getHandle(sensorList[i]);
		if (handle == sensor) {
			return sensorList[i];
		}
	}

	return nullptr;
}

static void printEvents(const SensorChannel &channel, const ASensorEvent *events, size_t eventsNb, void *priv)
{
	const int64_t now = getTime();
	InvnSensorControl *st = reinterpret_cast<InvnSensorControl *>(priv);
	std::unordered_map<int, size_t> sensorEventsNb;
	int channelId;

	// exit if no events
	if (eventsNb == 0) {
		return;
	}

	// find channel id
	channelId = st->control->getChannelId(channel.getId());
	if (channelId == SensorControl::CHANNEL_ID_INVALID) {
		return;
	}
	std::unordered_map<int, LastTimestamps> &lastTimestamps = st->lastTimestamps[channelId];

	for (size_t i = 0; i < eventsNb; ++i) {
		const ASensorEvent *event = &events[i];
		int32_t token, handle;
		ASensorRef sensor;

		// filter out incorrect event
		if (event->version != sizeof(*event)) {
			std::cout << "ERROR:" << channelId << " invalid event" << std::endl;
			continue;
		}

		// handle invalid type
		if (event->type == ASENSOR_TYPE_INVALID) {
			std::cout << "ERROR:" << channelId << " invalid type" << std::endl;
			continue;
		}

		// get sensor
		if (event->type == ASENSOR_TYPE_META_DATA) {
			token = event->meta_data.sensor;
		} else {
			token = event->sensor;
		}
		handle = channel.getDataSensorHandle(token);
		sensor = getSensor(st, handle);
		if (sensor == nullptr) {
			std::cout << "ERROR:" << channelId << " invalid handle " << handle << std::endl;
			continue;
		}

		// print event header
		if (event->type == ASENSOR_TYPE_META_DATA) {
			std::cout << "METADATA";
		} else if (event->type == ASENSOR_TYPE_ADDITIONAL_INFO) {
			std::cout << "ADDINFO";
		} else {
			std::cout << "DATALOG";
			// increment event counter
			size_t &nb = sensorEventsNb[event->sensor];
			++nb;
		}
		std::cout << ':' << channelId << " '" << ASensor_getName(sensor) << "'(" << handle << ")\t| ";

		// print event data
		switch (event->type) {
		case ASENSOR_TYPE_META_DATA:
			switch (event->meta_data.what) {
			case 0x01:
				printf("flush");
				break;
			default:
				printf("%#" PRIx32, event->meta_data.what);
				break;
			}
			break;
		case ASENSOR_TYPE_ADDITIONAL_INFO:
		{
			const AAdditionalInfoEvent &addInfoEvent = event->additional_info;
			switch (addInfoEvent.type) {
			case ASENSOR_ADDITIONAL_INFO_BEGIN:
				std::cout << "begin";
				break;
			case ASENSOR_ADDITIONAL_INFO_END:
				std::cout << "end";
				break;
			case ASENSOR_ADDITIONAL_INFO_UNTRACKED_DELAY:
				printf("untracked_delay: %+f ; %+f", addInfoEvent.data_float[0], addInfoEvent.data_float[1]);
				break;
			case ASENSOR_ADDITIONAL_INFO_INTERNAL_TEMPERATURE:
				printf("internal_temperature: %+10f", addInfoEvent.data_float[0]);
				break;
			case ASENSOR_ADDITIONAL_INFO_VEC3_CALIBRATION:
				printf("vec3_calibration: %+f, %+f, %+f, %+f, %+f, %+f, %+f, %+f, %+f, %+f, %+f, %+f",
						addInfoEvent.data_float[0], addInfoEvent.data_float[1], addInfoEvent.data_float[2], addInfoEvent.data_float[3],
						addInfoEvent.data_float[4], addInfoEvent.data_float[5], addInfoEvent.data_float[6], addInfoEvent.data_float[7],
						addInfoEvent.data_float[8], addInfoEvent.data_float[9], addInfoEvent.data_float[10], addInfoEvent.data_float[11]);
				break;
			case ASENSOR_ADDITIONAL_INFO_SENSOR_PLACEMENT:
				printf("sensor_placement: %+9f, %+9f, %+9f, %+9f, %+9f, %+9f, %+9f, %+9f, %+9f ; %+f, %+f, %+f",
						addInfoEvent.data_float[0], addInfoEvent.data_float[1], addInfoEvent.data_float[2],
						addInfoEvent.data_float[4], addInfoEvent.data_float[5], addInfoEvent.data_float[6],
						addInfoEvent.data_float[8], addInfoEvent.data_float[9], addInfoEvent.data_float[10],
						addInfoEvent.data_float[3], addInfoEvent.data_float[7], addInfoEvent.data_float[11]);
				break;
			case ASENSOR_ADDITIONAL_INFO_SAMPLING:
				printf("sampling: %+f ; %+f", addInfoEvent.data_float[0], addInfoEvent.data_float[1]);
				break;
			default:
				break;
			}
			break;
		}
		case ASENSOR_TYPE_ACCELEROMETER:
		case ASENSOR_TYPE_MAGNETIC_FIELD:
		case ASENSOR_TYPE_GYROSCOPE:
		case ASENSOR_TYPE_ORIENTATION:
			printf("%+13f, %+13f, %+13f ; %2d", event->vector.x, event->vector.y, event->vector.z,
					event->vector.status);
			break;
		case ASENSOR_TYPE_GRAVITY:
		case ASENSOR_TYPE_LINEAR_ACCELERATION:
			printf("%+13f, %+13f, %+13f", event->vector.x, event->vector.y, event->vector.z);
			break;
		case ASENSOR_TYPE_LIGHT:
		case ASENSOR_TYPE_PRESSURE:
		case ASENSOR_TYPE_PROXIMITY:
		case ASENSOR_TYPE_RELATIVE_HUMIDITY:
		case ASENSOR_TYPE_AMBIENT_TEMPERATURE:
		case ASENSOR_TYPE_SIGNIFICANT_MOTION:
		case ASENSOR_TYPE_STEP_DETECTOR:
		case ASENSOR_TYPE_STATIONARY_DETECT:
		case ASENSOR_TYPE_MOTION_DETECT:
		case ASENSOR_TYPE_HEART_BEAT:
		case ASENSOR_TYPE_TILT_DETECTOR:
		case ASENSOR_TYPE_WAKE_GESTURE:
		case ASENSOR_TYPE_GLANCE_GESTURE:
		case ASENSOR_TYPE_PICK_UP_GESTURE:
		case ASENSOR_TYPE_WRIST_TILT_GESTURE:
		case ASENSOR_TYPE_DEVICE_ORIENTATION:
		case ASENSOR_TYPE_LOW_LATENCY_OFFBODY_DETECT:
		case ASENSOR_TYPE_HINGE_ANGLE:
			printf("%+13f", event->data[0]);
			break;
		case ASENSOR_TYPE_ROTATION_VECTOR:
		case ASENSOR_TYPE_GAME_ROTATION_VECTOR:
		case ASENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR:
			printf("%+13f, %+13f, %+13f, %+13f ; %f", event->data[0], event->data[1],
					event->data[2], event->data[3], event->data[4]);
			break;
		case ASENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
		case ASENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
		case ASENSOR_TYPE_ACCELEROMETER_UNCALIBRATED:
		case ASENSOR_TYPE_ACCELEROMETER_LIMITED_AXES:
		case ASENSOR_TYPE_GYROSCOPE_LIMITED_AXES:
			printf("%+13f, %+13f, %+13f ; %+13f, %+13f, %+13f", event->data[0], event->data[1], event->data[2],
					event->data[3], event->data[4], event->data[5]);
			break;
		case ASENSOR_TYPE_STEP_COUNTER:
			printf("%13" PRIu64, event->u64.data[0]);
			break;
		case ASENSOR_TYPE_HEART_RATE:
			printf("%+13f ; %2d", event->heart_rate.bpm, event->heart_rate.status);
			break;
		case ASENSOR_TYPE_POSE_6DOF:
			printf("%+13f, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f",
					event->data[0], event->data[1], event->data[2], event->data[3],
					event->data[4], event->data[5], event->data[6], event->data[7],
					event->data[8], event->data[9], event->data[10], event->data[11],
					event->data[12], event->data[13], event->data[14]);
			break;
		case ASENSOR_TYPE_HEAD_TRACKER:
		{
			const int32_t *discontinuity_count = reinterpret_cast<const int32_t *>(&event->data[6]);
			printf("%+13f, %+13f, %+13f ; %+13f, %+13f, %+13f ; %d", event->data[0], event->data[1], event->data[2],
					event->data[3], event->data[4], event->data[5], *discontinuity_count);
			break;
		}
		case ASENSOR_TYPE_ACCELEROMETER_LIMITED_AXES_UNCALIBRATED:
		case ASENSOR_TYPE_GYROSCOPE_LIMITED_AXES_UNCALIBRATED:
			printf("%+13f, %+13f, %+13f ; %+13f, %+13f, %+13f ; %+13f, %+13f, %+13f",
					event->data[0], event->data[1], event->data[2],
					event->data[3], event->data[4], event->data[5],
					event->data[6], event->data[7], event->data[8]);
			break;
		case ASENSOR_TYPE_HEADING:
			printf("%+13f ; %+13f", event->data[0], event->data[1]);
			break;
		default:
			printf("%+13f, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f",
					event->data[0], event->data[1], event->data[2], event->data[3],
					event->data[4], event->data[5], event->data[6], event->data[7],
					event->data[8], event->data[9], event->data[10], event->data[11],
					event->data[12], event->data[13], event->data[14], event->data[15]);
			break;
		}

		// print timestamp
		printf("\t| %16" PRId64, event->timestamp);

		// skip delta timestamp
		if (event->type == ASENSOR_TYPE_META_DATA || event->type == ASENSOR_TYPE_ADDITIONAL_INFO) {
			std::cout << std::endl;
			continue;
		}

		// compute and print delta timestamp
		LastTimestamps &lastTs = lastTimestamps[event->sensor];
		if (lastTs.lastEventTs == 0) {
			lastTs.lastEventTs = st->startTimestamp;
		}
		if (lastTs.lastBatchTs == 0) {
			lastTs.lastBatchTs = st->startTimestamp;
		}
		double deltaEvent = static_cast<double>(event->timestamp - lastTs.lastEventTs) / 1000000.0;
		double deltaBatch = static_cast<double>(now - lastTs.lastBatchTs) / 1000000.0;
		printf(", %8.3f, %10.3f\n", deltaEvent, deltaBatch);
		lastTs.lastEventTs = event->timestamp;
	}

	for (auto &p : sensorEventsNb) {
		LastTimestamps &lastTs = lastTimestamps[p.first];
		if (p.second > 1) {
			double delta = static_cast<double>(now - lastTs.lastBatchTs) / 1000000.0;
			std::cout << "INFO:" << channelId << " batch sensor #" << p.first << " nb " << p.second << " duration " << delta << "ms\n";
		}
		lastTs.lastBatchTs = now;
	}

	std::cout.flush();
}

int main(int argc, char *argv[])
{
	static InvnSensorControl st;
	int opt;
	bool listSensors = false;
	bool verbose = false;
	SensorChannel::EventsCallback eventsCallback = nullptr;
	uint32_t timeoutMs = 0;
	int status, ret;

	// parse command line args
	while ((opt = getopt(argc, argv, "hVplvt:")) != -1) {
		switch (opt) {
		// help
		case 'h':
			printUsage(argv[0]);
			exit(EXIT_SUCCESS);
			break;
		case 'V':
			printVersion(argv[0]);
			exit(EXIT_SUCCESS);
			break;
		// print sensor events
		case 'p':
			eventsCallback = printEvents;
			break;
		// list sensors
		case 'l':
			listSensors = true;
			break;
		// verbose mode
		case 'v':
			verbose = true;
			break;
		// timeout mode
		case 't':
			ret = sscanf(optarg, "%" SCNu32, &timeoutMs);
			if (ret != 1) {
				std::cout << "error: -t timeout syntax error" << std::endl;
				printUsage(argv[0]);
				exit(EXIT_FAILURE);
			}
			break;
		default:
			break;
		}
	}

	// instantiate main SensorControl
	SensorControl *control = new(std::nothrow) SensorControl();
	if (control == nullptr) {
		std::cout << "error: not enough memory" << std::endl;
		exit(EXIT_FAILURE);
	}
	ret = control->getError();
	if (ret) {
		status = EXIT_FAILURE;
		goto exit;
	}

	// print sensors list
	if (listSensors) {
		printSensorList(control->getSensorList(), control->getSensorNb(), verbose);
		std::cout << std::endl;
	}

	// parse sensor parameters for init
	ret = control->parseSensorsParameters(const_cast<const char **>(&argv[optind]), argc - optind);
	if (ret) {
		status = EXIT_FAILURE;
		goto exit;
	}
	std::cout << std::endl;

	// start SensorControl
	ret = control->start(eventsCallback, &st);
	if (ret) {
		status = EXIT_FAILURE;
		goto exit;
	}

	// run SensorControl
	st.control = control;
	st.startTimestamp = getTime();
	control->run(timeoutMs);

	// stop SensorControl
	control->stop();

	std::cout << "\nexit" << std::endl;
	status = EXIT_SUCCESS;
exit:
	delete control;
	exit(status);
}
