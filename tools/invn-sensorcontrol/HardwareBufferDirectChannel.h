#ifndef HARDWAREBUFFERDIRECTCHANNEL_H_
#define HARDWAREBUFFERDIRECTCHANNEL_H_

#include <android/sensor.h>
#include <android/looper.h>
#include <android/hardware_buffer.h>

#include "SensorDirectChannel.h"

class HardwareBufferDirectChannel : public SensorDirectChannel {
public:
	static int checkSensorParameters(ASensorRef sensor, int rateHz, int64_t latencyUs);
	HardwareBufferDirectChannel(ASensorManager *manager, ALooper *looper, size_t size);
	virtual ~HardwareBufferDirectChannel() override;

protected:
	AHardwareBuffer *mHardwareBuffer;
	virtual int dataCallback(int fd, int events) override;
};

#endif
