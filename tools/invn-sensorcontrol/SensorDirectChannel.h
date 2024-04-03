#ifndef SENSORDIRECTCHANNEL_H_
#define SENSORDIRECTCHANNEL_H_

#include <unordered_map>

#include <android/sensor.h>
#include <android/looper.h>

#include "SensorChannel.h"

class SensorDirectChannel : public SensorChannel {
public:
	static int checkSensorParameters(ASensorRef sensor, int rateHz, int64_t latencyUs);
	static const char *getRateString(int rate) {
		switch (rate) {
		case ASENSOR_DIRECT_RATE_STOP:
			return "stop";
		case ASENSOR_DIRECT_RATE_NORMAL:
			return "normal";
		case ASENSOR_DIRECT_RATE_FAST:
			return "fast";
		case ASENSOR_DIRECT_RATE_VERY_FAST:
			return "very-fast";
		default:
			return "invalid";
		}
	}
	static int getRateHz(int rate) {
		switch (rate) {
		case ASENSOR_DIRECT_RATE_NORMAL:
			return 50;
		case ASENSOR_DIRECT_RATE_FAST:
			return 200;
		case ASENSOR_DIRECT_RATE_VERY_FAST:
			return 800;
		case ASENSOR_DIRECT_RATE_STOP:
		default:
			return 0;
		}
	}
	SensorDirectChannel(ASensorManager *manager, ALooper *looper, size_t size);
	virtual ~SensorDirectChannel();
	virtual int32_t getDataSensorHandle(int32_t token) const override;
	virtual int registerSensor(ASensorRef sensor, int rateHz, int64_t latencyUs) override;
	virtual int disableSensor(ASensorRef sensor) override;

protected:
	int mTimer;
	struct itimerspec mTimerVal;
	size_t mRegisteredSensorsNb;
	std::unordered_map<int32_t, ASensorRef> mSensorsToken;
	uint32_t mBufferCurrent;
	uint32_t mBufferCounter;
	virtual int dataCallback(int fd, int events) override = 0;
};

#endif
