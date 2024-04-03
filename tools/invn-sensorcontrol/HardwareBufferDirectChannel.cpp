#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <iostream>

#include <android/sensor.h>
#include <android/looper.h>
#include <android/hardware_buffer.h>

#include "SensorDirectChannel.h"
#include "HardwareBufferDirectChannel.h"

int HardwareBufferDirectChannel::checkSensorParameters(ASensorRef sensor, int rateHz, int64_t latencyUs)
{
	// check direct report type support
	if (!ASensor_isDirectChannelTypeSupported(sensor, ASENSOR_DIRECT_CHANNEL_TYPE_HARDWARE_BUFFER)) {
		std::cout << "error: hardware buffer direct report type not supported by '" << ASensor_getName(sensor) << std::endl;
		return -EINVAL;
	}

	return SensorDirectChannel::checkSensorParameters(sensor, rateHz, latencyUs);
}

HardwareBufferDirectChannel::HardwareBufferDirectChannel(ASensorManager *manager, ALooper *looper, size_t size)
	: SensorDirectChannel(manager, looper, size),
	  mHardwareBuffer(nullptr)
{
	const AHardwareBuffer_Desc hardwareBufferDesc = {
		.width = static_cast<uint32_t>(mSize),
		.height = 1,
		.layers = 1,
		.format = AHARDWAREBUFFER_FORMAT_BLOB,
		.usage = AHARDWAREBUFFER_USAGE_CPU_WRITE_OFTEN | AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN |
			 AHARDWAREBUFFER_USAGE_SENSOR_DIRECT_DATA,
		.stride = 0,
		.rfu0 = 0,
		.rfu1 = 0,
	};
	int ret;

	mType = SensorChannel::TYPE_HARDWARE_BUFFER;

	ret = AHardwareBuffer_allocate(&hardwareBufferDesc, &mHardwareBuffer);
	if (ret < 0) {
		std::cout << "error: " << ret << ": hardware buffer allocation error" << std::endl;
		mError = ret;
		return;
	}

	mId = ASensorManager_createHardwareBufferDirectChannel(mManager, mHardwareBuffer, mSize);
	if (mId <= 0) {
		std::cout << "error: " << mId << ": cannot create hardware buffer channel" << std::endl;
		mError = mId;
		return;
	}

	mError = 0;
	return;
}

HardwareBufferDirectChannel::~HardwareBufferDirectChannel()
{
	if (mId > 0) {
		ASensorManager_destroyDirectChannel(mManager, mId);
	}
	if (mHardwareBuffer != nullptr) {
		AHardwareBuffer_release(mHardwareBuffer);
	}
}

int HardwareBufferDirectChannel::dataCallback(int fd, int events)
{
	int ret;

	if (events & (ALOOPER_EVENT_ERROR | ALOOPER_EVENT_HANGUP | ALOOPER_EVENT_INVALID)) {
		std::cout << "error: event error " << std::hex << events << std::endl;
		return 0;
	}
	if (!(events & ALOOPER_EVENT_INPUT)) {
		std::cout << "error: nothing to read" << std::endl;
		return 1;
	}

	// read timer expiration number
	uint64_t timerNb;
	read(fd, &timerNb, sizeof(timerNb));

	const size_t bufferNb = mSize / sizeof(mBuffer[0]);
	uint32_t begin = mBufferCurrent;
	uint32_t end = mBufferCurrent;

	ret = AHardwareBuffer_lock(mHardwareBuffer, AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN, -1, NULL, reinterpret_cast<void **>(&mBuffer));
	if (ret < 0) {
		std::cout << "error: hardware buffer lock error " << ret << std::endl;
		return 1;
	}

	while ((uint32_t)mBuffer[end].reserved0 > mBufferCounter) {
		++end;
		if (end >= bufferNb) {
			end = 0;
		}
	}

	if (end >= begin) {
		eventsCallback(&mBuffer[begin], end - begin);
		mBufferCounter += end - begin;
	} else {
		eventsCallback(&mBuffer[begin], bufferNb - begin);
		eventsCallback(&mBuffer[0], end);
		mBufferCounter += bufferNb - begin + end;
	}

	ret = AHardwareBuffer_unlock(mHardwareBuffer, NULL);
	if (ret < 0) {
		std::cout << "error: hardware buffer unlock error " << ret << std::endl;
	}

	mBufferCurrent = end;

	return 1;
}
