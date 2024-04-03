#ifndef SENSOREVENTQUEUE_H_
#define SENSOREVENTQUEUE_H_

#include <android/sensor.h>
#include <android/looper.h>

#include "SensorChannel.h"

class SensorEventQueue : public SensorChannel {
public:
	static int checkSensorParameters(ASensorRef sensor, int rateHz, int64_t latencyUs);
	SensorEventQueue(ASensorManager *manager, ALooper *looper, size_t size);
	virtual ~SensorEventQueue() override;
	virtual int32_t getDataSensorHandle(int32_t token) const override { return token; };
	virtual int registerSensor(ASensorRef sensor, int rateHz, int64_t latencyUs) override;
	virtual int disableSensor(ASensorRef sensor) override;

protected:
	virtual int dataCallback(int fd, int events) override;

private:
	ASensorEventQueue *mQueue;
};

#endif
