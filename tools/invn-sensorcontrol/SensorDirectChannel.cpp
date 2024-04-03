#include <iostream>
#include <ctime>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <sys/timerfd.h>

#include <android/looper.h>
#include <android/sensor.h>

#include "SensorChannel.h"
#include "SensorDirectChannel.h"

int SensorDirectChannel::checkSensorParameters(ASensorRef sensor, int rateHz, int64_t latencyUs)
{
	// check if rate parameter is valid
	switch (rateHz) {
	case ASENSOR_DIRECT_RATE_NORMAL:
	case ASENSOR_DIRECT_RATE_FAST:
	case ASENSOR_DIRECT_RATE_VERY_FAST:
		// valid
		break;
	default:
		// invalid
		std::cout << "error: invalid direct report rate " << rateHz << std::endl;
		return -EINVAL;
	}

	// check if requested rate is supported
	if (rateHz > ASensor_getHighestDirectReportRateLevel(sensor)) {
		std::cout << "error: direct report rate '" << getRateString(rateHz) << "' not supported by '" << ASensor_getName(sensor)
				<< "' (max '" << getRateString(ASensor_getHighestDirectReportRateLevel(sensor)) << "')" << std::endl;
		return -EINVAL;
	}

	// print warning if batch mode requested
	if (latencyUs > 0) {
		std::cout << "warning: batch mode not supported in direct report mode" << std::endl;
	}

	return 0;
}

SensorDirectChannel::SensorDirectChannel(ASensorManager *manager, ALooper *looper, size_t size)
	: SensorChannel(manager, looper, size),
	  mTimer(-1),
	  mTimerVal({{0, 0}, {0, 0}}),
	  mRegisteredSensorsNb(0),
	  mBufferCurrent(0),
	  mBufferCounter(0)
{
	int ret;

	mTimer = timerfd_create(CLOCK_BOOTTIME, TFD_NONBLOCK | TFD_CLOEXEC);
	if (mTimer == -1) {
		std::cout << "error: '" << strerror(errno) << "' (" << errno << "): cannot create timerfd" << std::endl;
		mError = -errno;
		return;
	}

	ret = ALooper_addFd(mLooper, mTimer, ALOOPER_POLL_CALLBACK, ALOOPER_EVENT_INPUT, dataCallbackWrapper, this);
	if (ret == -1) {
		std::cout << "error: cannot add timer to event loop" << std::endl;
		mError = -EINVAL;
		return;
	}

	mError = 0;
	return;
}

SensorDirectChannel::~SensorDirectChannel()
{
	if (mTimer >= 0) {
		ALooper_removeFd(mLooper, mTimer);
		close(mTimer);
	}
}

int32_t SensorDirectChannel::getDataSensorHandle(int32_t token) const
{
	int32_t handle = -1;

	auto search = mSensorsToken.find(token);
	if (search != mSensorsToken.end()) {
		ASensorRef sensor = search->second;
		handle = ASensor_getHandle(sensor);
	}

	return handle;
}

int SensorDirectChannel::registerSensor(ASensorRef sensor, int rateHz, int64_t latencyUs)
{
	int ret;

	(void)latencyUs;

	ret = ASensorManager_configureDirectReport(mManager, sensor, mId, rateHz);
	if (ret < 0) {
		std::cout << "error: " << ret << ": cannot register direct channem sensor '" << ASensor_getName(sensor) << "'" << std::endl;
		return ret;
	}

	// register sensor token
	mSensorsToken[ret] = sensor;

	long period = 1000000000L / getRateHz(rateHz);
	if (mRegisteredSensorsNb == 0 || mTimerVal.it_interval.tv_nsec > period) {
		mTimerVal.it_interval.tv_nsec = period;
		mTimerVal.it_value = mTimerVal.it_interval;
		ret = timerfd_settime(mTimer, 0, &mTimerVal, NULL);
		if (ret) {
			std::cout << "error: '" << strerror(errno) << "' (" << errno << "): cannot arm memory timer" << std::endl;
			ret = -errno;
			goto error_unregister;
		}
	}

	++mRegisteredSensorsNb;
	return 0;

error_unregister:
	ASensorManager_configureDirectReport(mManager, sensor, mId, ASENSOR_DIRECT_RATE_STOP);
	return ret;
}

int SensorDirectChannel::disableSensor(ASensorRef sensor)
{
	int ret;

	if (mRegisteredSensorsNb == 0) {
		return -EINVAL;
	}

	ret = ASensorManager_configureDirectReport(mManager, sensor, mId, ASENSOR_DIRECT_RATE_STOP);
	if (ret) {
		return ret;
	}

	// unregister sensor token
	for (auto it = mSensorsToken.begin(); it != mSensorsToken.end(); ++it) {
		if (it->second == sensor) {
			mSensorsToken.erase(it);
			break;
		}
	}

	if (--mRegisteredSensorsNb == 0) {
		struct itimerspec stopTimer = {{0, 0}, {0, 0}};
		timerfd_settime(mTimer, 0, &stopTimer, NULL);
	}

	return 0;
}
