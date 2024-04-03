#ifndef SHAREDMEMORYDIRECTCHANNEL_H_
#define SHAREDMEMORYDIRECTCHANNEL_H_

#include <android/sensor.h>
#include <android/looper.h>

#include "SensorDirectChannel.h"

class SharedMemoryDirectChannel : public SensorDirectChannel {
public:
	static int checkSensorParameters(ASensorRef sensor, int rateHz, int64_t latencyUs);
	SharedMemoryDirectChannel(ASensorManager *manager, ALooper *looper, size_t size);
	virtual ~SharedMemoryDirectChannel() override;

protected:
	int mFd;
	virtual int dataCallback(int fd, int events) override;
};

#endif
