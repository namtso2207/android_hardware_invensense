#include <iostream>
#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <sys/mman.h>

#include <android/sensor.h>
#include <android/looper.h>
#include <android/sharedmem.h>

#include "SensorDirectChannel.h"
#include "SharedMemoryDirectChannel.h"

int SharedMemoryDirectChannel::checkSensorParameters(ASensorRef sensor, int rateHz, int64_t latencyUs)
{
	// check direct report type support
	if (!ASensor_isDirectChannelTypeSupported(sensor, ASENSOR_DIRECT_CHANNEL_TYPE_SHARED_MEMORY)) {
		std::cout << "error: shared memory direct report type not supported by '" << ASensor_getName(sensor) << "'" << std::endl;
		return -EINVAL;
	}

	return SensorDirectChannel::checkSensorParameters(sensor, rateHz, latencyUs);
}

SharedMemoryDirectChannel::SharedMemoryDirectChannel(ASensorManager *manager, ALooper *looper, size_t size)
	: SensorDirectChannel(manager, looper, size),
	  mFd(-1)
{
	mType = SensorChannel::TYPE_SHARED_MEMORY;

	mFd = ASharedMemory_create("com.invensense.sensorcontrol", mSize);
	if (mFd < 0) {
		std::cout << "error: " << mFd << ": cannot create shared memory" << std::endl;
		mError = mFd;
		return;
	}

	mBuffer = reinterpret_cast<ASensorEvent *>(mmap(NULL, mSize, PROT_READ, MAP_SHARED, mFd, 0));
	if (mBuffer == MAP_FAILED) {
		std::cout << "error: '" << strerror(errno) << "' (" << errno << "): shared memory mmap error" << std::endl;
		mError = -errno;
		return;
	}

	mId = ASensorManager_createSharedMemoryDirectChannel(mManager, mFd, mSize);
	if (mId <= 0) {
		std::cout << "error: " << mId << ": cannot create shared memory channel" << std::endl;
		mError = mId;
		return;
	}

	mError = 0;
	return;
}

SharedMemoryDirectChannel::~SharedMemoryDirectChannel()
{
	if (mId > 0) {
		ASensorManager_destroyDirectChannel(mManager, mId);
	}
	if (mBuffer != MAP_FAILED) {
		munmap(mBuffer, mSize);
	}
	if (mFd >= 0) {
		close(mFd);
	}
}

int SharedMemoryDirectChannel::dataCallback(int fd, int events)
{
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

	mBufferCurrent = end;

	return 1;
}
