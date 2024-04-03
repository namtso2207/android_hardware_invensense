#ifndef SENSORCONTROL_H_
#define SENSORCONTROL_H_

#include <list>
#include <memory>

#include <android/looper.h>
#include <android/sensor.h>

#include "SensorChannel.h"

class SensorControl {
public:
	// Sensor channel ids
	enum {
		CHANNEL_ID_INVALID = -1,
		CHANNEL_ID_NORMAL = 0,
		CHANNEL_ID_SHARED_MEMORY,
		CHANNEL_ID_HARDWARE_BUFFER,
		CHANNEL_ID_NB,

		CHANNEL_ID_FIRST = CHANNEL_ID_NORMAL,
		CHANNEL_ID_LAST = CHANNEL_ID_NB - 1,
	};
	SensorControl();
	virtual ~SensorControl() { }
	int getError() const { return mError; }
	ASensorManager *getSensorManager() const { return mManager; }
	ASensorList getSensorList() const { return mSensorList; }
	size_t getSensorNb() const { return mSensorNb; }
	int getChannelId(int id) const {
		for (int i = CHANNEL_ID_FIRST; i < CHANNEL_ID_NB; ++i) {
			if (mChannels[i] == nullptr) {
				continue;
			}
			if (mChannels[i]->getId() == id) {
				return i;
			}
		}
		return CHANNEL_ID_INVALID;
	}
	int parseSensorsParameters(const char *strings[], size_t nb);
	int start(SensorChannel::EventsCallback cb, void *priv);
	void stop();
	void run(uint32_t timeoutMs);

private:
	// Sensor parameters
	struct SensorParam {
		ASensorRef sensor;
		int reportType;
		int rate;
		int64_t latencyUs;
		int channelId;
	};
	int mError;
	ALooper *mLooper;
	ASensorManager *mManager;
	ASensorList mSensorList;
	size_t mSensorNb;
	std::list<SensorParam> mParams;
	std::unique_ptr<SensorChannel> mChannels[CHANNEL_ID_NB];
	bool mRun;
	static int exitCallback(int fd, int events, void *priv);
	int parseSensorId(const char *str, ASensorRef &sensor);
	int parseSensorName(const char *str, ASensorRef &sensor);
	int parseSensorArgs(const char *str, int &rate, double &latencyMs, int &type);
};

#endif
