#include <cstdlib>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <csignal>
#include <ctime>
#include <string>
#include <iostream>
#include <list>
#include <sys/signalfd.h>

#include <android/looper.h>
#include <android/sensor.h>

#include "SensorEventQueue.h"
#include "SharedMemoryDirectChannel.h"
#include "HardwareBufferDirectChannel.h"
#include "SensorControl.h"

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(array)		(sizeof(array) / sizeof(array[0]))
#endif

#define BUFFER_SIZE		(1024 * sizeof(ASensorEvent))

int SensorControl::exitCallback(int fd, int events, void *priv)
{
	SensorControl *control = reinterpret_cast<SensorControl *>(priv);

	(void)fd, (void)events;
	control->mRun = false;

	return 1;
}

int SensorControl::parseSensorId(const char *str, ASensorRef &sensor)
{
	int id;
	int ret;

	ret = sscanf(str, "%d", &id);
	if (ret != 1) {
		return -EINVAL;
	}
	if (id <= 0 || static_cast<size_t>(id - 1) >= mSensorNb) {
		return -EINVAL;
	}

	sensor = mSensorList[id - 1];
	return 0;
}

int SensorControl::parseSensorName(const char *str, ASensorRef &sensor)
{
	std::string string(str);

	size_t len = string.find_first_of(',');
	if (len == std::string::npos) {
		return -EINVAL;
	}

	for (size_t i = 0; i < mSensorNb; ++i) {
		if (string.compare(0, len, ASensor_getName(mSensorList[i])) == 0) {
			sensor = mSensorList[i];
			return 0;
		}
	}

	return -EINVAL;
}

int SensorControl::parseSensorArgs(const char *str, int &rate, double &latencyMs, int &type)
{
	int ret;
	const char *args;

	args = strchr(str, ',');
	if (args == nullptr) {
		return -EINVAL;
	}
	++args;

	rate = -1;
	latencyMs = 0;
	type = SensorChannel::TYPE_NORMAL;
	ret = sscanf(args, "%d,%lf,%d", &rate, &latencyMs, &type);
	if (ret < 1) {
		return -EINVAL;
	}

	if (rate <= 0 || latencyMs < 0 || type < SensorChannel::TYPE_FIRST || type > SensorChannel::TYPE_LAST) {
		return -EINVAL;
	}

	return 0;
}

SensorControl::SensorControl()
	: mRun(false)
{
	int ret;

	// create event loop
	mLooper = ALooper_prepare(0);
	if (mLooper == nullptr) {
		std::cout << "error: cannot create event loop" << std::endl;
		mError = -1;
		return;
	}

	// get sensor manager
	mManager = ASensorManager_getInstanceForPackage("com.invensense.sensorcontrol");
	if (mManager == nullptr) {
		std::cout << "error: cannot get sensor manager" << std::endl;
		mError = -1;
		return;
	}

	// retrieve sensors list
	ret = ASensorManager_getSensorList(mManager, &mSensorList);
	if (ret < 0) {
		std::cout << "error: " << ret << ": cannot retrieve sensor list" << std::endl;
		mError = -1;
		return;
	}
	mSensorNb = ret;

	mError = 0;
	return;
}

int SensorControl::parseSensorsParameters(const char *strings[], size_t nb)
{
	int ret;

	// parse sensor parameters
	for (size_t i = 0; i < nb; ++i) {
		ASensorRef sensor;
		int rate;
		double latencyMs;
		int type;
		int channelId;

		ret = parseSensorId(strings[i], sensor);
		if (ret != 0) {
			ret = parseSensorName(strings[i], sensor);
			if (ret != 0) {
				std::cout << "error: cannot parse sensor in '" << strings[i] << '\'' << std::endl;
				return -EINVAL;
			}
		}

		ret = parseSensorArgs(strings[i], rate, latencyMs, type);
		if (ret != 0) {
			std::cout << "error: invalid sensor arguments syntax '" << strings[i] << '\'' << std::endl;
			return -EINVAL;
		}

		// Compute channel id
		switch (type) {
		case SensorChannel::TYPE_NORMAL:
			channelId = CHANNEL_ID_NORMAL;
			break;
		case SensorChannel::TYPE_SHARED_MEMORY:
			channelId = CHANNEL_ID_SHARED_MEMORY;
			break;
		case SensorChannel::TYPE_HARDWARE_BUFFER:
			channelId = CHANNEL_ID_HARDWARE_BUFFER;
			break;
		default:
			return -EINVAL;
		}

		SensorParam param = {
			.sensor = sensor,
			.reportType = type,
			.rate = rate,
			.latencyUs = static_cast<int64_t>(latencyMs * 1000.0),
			.channelId = channelId,
		};
		mParams.push_back(param);
	}

	// verify sensor parameters and allocate channels
	bool init[CHANNEL_ID_NB];
	for (int i = CHANNEL_ID_FIRST; i < CHANNEL_ID_NB; ++i) {
		init[i] = false;
	}

	for (auto &p : mParams) {
		switch (p.channelId) {
		case CHANNEL_ID_NORMAL:
			ret = SensorEventQueue::checkSensorParameters(p.sensor, p.rate, p.latencyUs);
			if (ret) {
				return ret;
			}
			if (!init[CHANNEL_ID_NORMAL]) {
				mChannels[CHANNEL_ID_NORMAL] = std::unique_ptr<SensorChannel>(new(std::nothrow) SensorEventQueue(mManager, mLooper, 16 * BUFFER_SIZE));
			}
			break;
		case CHANNEL_ID_SHARED_MEMORY:
			ret = SharedMemoryDirectChannel::checkSensorParameters(p.sensor, p.rate, p.latencyUs);
			if (ret) {
				return ret;
			}
			if (!init[CHANNEL_ID_SHARED_MEMORY]) {
				mChannels[CHANNEL_ID_SHARED_MEMORY] = std::unique_ptr<SensorChannel>(new(std::nothrow) SharedMemoryDirectChannel(mManager, mLooper, BUFFER_SIZE));
			}
			break;
		case CHANNEL_ID_HARDWARE_BUFFER:
			ret = HardwareBufferDirectChannel::checkSensorParameters(p.sensor, p.rate, p.latencyUs);
			if (ret) {
				return ret;
			}
			if (!init[CHANNEL_ID_HARDWARE_BUFFER]) {
				mChannels[CHANNEL_ID_HARDWARE_BUFFER] = std::unique_ptr<SensorChannel>(new(std::nothrow) HardwareBufferDirectChannel(mManager, mLooper, BUFFER_SIZE));
			}
			break;
		default:
			std::cout << "error: invalid report type value " << p.reportType << std::endl;
			return -EINVAL;
		}
		if (!init[p.channelId]) {
			if (mChannels[p.channelId] == nullptr) {
				return -ENOMEM;
			}
			ret = mChannels[p.channelId]->getError();
			if (ret) {
				return ret;
			}
			init[p.channelId] = true;
		}
	}

	return 0;
}

int SensorControl::start(SensorChannel::EventsCallback cb, void *priv)
{
	int ret;

	// turn sensors on
	for (auto &p : mParams) {
		double latencyMs = static_cast<double>(p.latencyUs) / 1000.0;
		int32_t rateHz;
		switch (p.reportType) {
		case SensorChannel::TYPE_SHARED_MEMORY:
		case SensorChannel::TYPE_HARDWARE_BUFFER:
			rateHz = SensorDirectChannel::getRateHz(p.rate);
			break;
		case SensorChannel::TYPE_NORMAL:
		default:
			rateHz = p.rate;
			break;
		}
		std::cout << "## enabling '" << ASensor_getName(p.sensor) << "' @ " << rateHz << "Hz ("
				<< 1000.0 / static_cast<double>(rateHz) << "ms) - batch " << latencyMs << "ms - report "
				<< SensorChannel::getReportTypeString(p.reportType) << "(" << p.channelId << ")" << std::endl;
		mChannels[p.channelId]->setEventsCallback(cb, priv);
		ret = mChannels[p.channelId]->registerSensor(p.sensor, p.rate, p.latencyUs);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

void SensorControl::stop()
{
	// turn all sensors off
	for (auto &p : mParams) {
		mChannels[p.channelId]->disableSensor(p.sensor);
	}
}

void SensorControl::run(uint32_t timeoutMs)
{
	sigset_t sigs;
	int exitSigs[] = { SIGINT, SIGQUIT, SIGTERM };
	int sigFd;
	timer_t timeoutId;
	bool timeout = timeoutMs > 0 ? true : false;
	int ret;

	// exit if there are no sensors parameters
	if (mParams.empty()) {
		return;
	}

	mRun = true;

	// block all signals
	sigfillset(&sigs);
	ret = sigprocmask(SIG_BLOCK, &sigs, NULL);
	if (ret) {
		std::cout << "warning: '" << strerror(errno) << "'(" << errno << "): cannot block all signals" << std::endl;
	}

	// create signal fd for exit
	sigemptyset(&sigs);
	for (size_t i = 0; i < ARRAY_SIZE(exitSigs); ++i) {
		sigaddset(&sigs, exitSigs[i]);
	}
	sigFd = signalfd(-1, &sigs, SFD_NONBLOCK | SFD_CLOEXEC);
	if (sigFd == -1) {
		std::cout << "warning: '" << strerror(errno) << "'(" << errno << "): cannot create exit signals fd" << std::endl;
	} else {
		// add signal fd to event loop
		ret = ALooper_addFd(mLooper, sigFd, ALOOPER_POLL_CALLBACK, ALOOPER_EVENT_INPUT, exitCallback, this);
		if (ret == -1) {
			std::cout << "warning: cannot add exit signals fd to loop" << std::endl;
		}
	}

	if (timeout) {
		// create timeout timer
		struct sigevent sevp = {
			.sigev_value = {
				.sival_int = 0,
			},
			.sigev_signo = SIGTERM,
			.sigev_notify = SIGEV_SIGNAL,
		};
		ret = timer_create(CLOCK_BOOTTIME, &sevp, &timeoutId);
		if (ret == -1) {
			std::cout << "warning: '" << strerror(errno) << "'(" << errno << "): cannot create timeout timer" << std::endl;
			timeout = false;
		} else {
			const struct itimerspec timerVal = {
				.it_interval = { 0, 0 },
				.it_value = { static_cast<long>(timeoutMs / 1000), static_cast<long>((timeoutMs % 1000) * 1000000L) },
			};
			ret = timer_settime(timeoutId, 0, &timerVal, nullptr);
			if (ret == -1) {
				std::cout << "warning: '" << strerror(errno) << "'(" << errno << "): cannot arm timeout timer" << std::endl;
			}
		}
	}

	// loop for events
	while (mRun) {
		int outFd, outEvents;
		void *outData;
		ret = ALooper_pollOnce(-1, &outFd, &outEvents, &outData);
		if (ret == ALOOPER_POLL_ERROR) {
			std::cout << "error: looper poll error" << std::endl;
			mRun = false;
		}
	}

	if (timeout) {
		timer_delete(timeoutId);
	}
	ALooper_removeFd(mLooper, sigFd);
	close(sigFd);
}
