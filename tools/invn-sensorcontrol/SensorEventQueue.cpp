#include <cerrno>
#include <iostream>

#include <android/sensor.h>
#include <android/looper.h>

#include "SensorChannel.h"
#include "SensorEventQueue.h"

int SensorEventQueue::checkSensorParameters(ASensorRef sensor, int rateHz, int64_t latencyUs)
{
	if (rateHz == 0) {
		return -EINVAL;
	}

	// check if rate is supported, warning only
	uint32_t period = 1000000L / rateHz;
	if (period < (uint32_t)ASensor_getMinDelay(sensor)) {
		std::cout << "warning: '" << ASensor_getName(sensor) << "' maximal rate is " << (1000000UL / ASensor_getMinDelay(sensor)) << std::endl;
	}

	// check if latency is supported, warning only
	int64_t maxLatency = (int64_t)ASensor_getFifoMaxEventCount(sensor) * 1000000LL / (int64_t)rateHz;
	if (latencyUs > maxLatency) {
		std::cout << "warning: '" << ASensor_getName(sensor) << "' maximal latency is " << maxLatency << "us" << std::endl;
	}

	return 0;
}

SensorEventQueue::SensorEventQueue(ASensorManager *manager, ALooper *looper, size_t size)
	: SensorChannel(manager, looper, size),
	  mQueue(nullptr)
{
	mType = SensorChannel::TYPE_NORMAL;
	mId = 0;
	int ret;

	mBuffer = new(std::nothrow) ASensorEvent[mSize / sizeof(mBuffer[0])];
	if (mBuffer == nullptr) {
		mError = -ENOMEM;
		return;
	}

	mQueue = ASensorManager_createEventQueue(mManager, mLooper, ALOOPER_POLL_CALLBACK, dataCallbackWrapper, this);
	if (mQueue == nullptr) {
		mError = -EINVAL;
		return;
	}

	ret = ASensorEventQueue_requestAdditionalInfoEvents(mQueue, true);
	if (ret != 0) {
		std::cout << "error: " << ret << " cannot request additional info" << std::endl;
	}

	mError = 0;
}

SensorEventQueue::~SensorEventQueue()
{
	if (mQueue != nullptr) {
		ASensorManager_destroyEventQueue(mManager, mQueue);
	}
	if (mBuffer != nullptr) {
		delete[] mBuffer;
	}
}

int SensorEventQueue::registerSensor(ASensorRef sensor, int rateHz, int64_t latencyUs)
{
	if (rateHz == 0) {
		return -EINVAL;
	}

	return ASensorEventQueue_registerSensor(mQueue, sensor, 1000000L / rateHz, latencyUs);
}

int SensorEventQueue::disableSensor(ASensorRef sensor)
{
	return ASensorEventQueue_disableSensor(mQueue, sensor);
}

int SensorEventQueue::dataCallback(int fd, int events)
{
	int ret;

	(void)fd;

	if (events & (ALOOPER_EVENT_ERROR | ALOOPER_EVENT_HANGUP | ALOOPER_EVENT_INVALID)) {
		std::cout << "error: queue event error " << std::hex << events << std::endl;
		return 0;
	}
	if (!(events & ALOOPER_EVENT_INPUT)) {
		std::cout << "error: queue nothing to read" << std::endl;
		return 1;
	}

	const size_t bufferNb = mSize / sizeof(mBuffer[0]);
	size_t nb = 0;
	while (ASensorEventQueue_hasEvents(mQueue) == 1) {
		ret = ASensorEventQueue_getEvents(mQueue, &mBuffer[nb], bufferNb - nb);
		if (ret < 0) {
			std::cout << "error: queue reading error " << ret << std::endl;
			return 0;
		}
		nb += ret;
		if (nb >= bufferNb) {
			eventsCallback(mBuffer, nb);
			nb = 0;
		}
	}
	eventsCallback(mBuffer, nb);

	return 1;
}
