#ifndef SENSORCHANNEL_H_
#define SENSORCHANNEL_H_

#include <android/sensor.h>
#include <android/looper.h>

class SensorChannel {
public:
	// Sensor channel type
	enum {
		TYPE_INVALID = ASENSOR_DIRECT_CHANNEL_TYPE_SHARED_MEMORY - 2,
		TYPE_NORMAL = ASENSOR_DIRECT_CHANNEL_TYPE_SHARED_MEMORY - 1,
		TYPE_SHARED_MEMORY = ASENSOR_DIRECT_CHANNEL_TYPE_SHARED_MEMORY,
		TYPE_HARDWARE_BUFFER = ASENSOR_DIRECT_CHANNEL_TYPE_HARDWARE_BUFFER,

		TYPE_FIRST = TYPE_NORMAL,
		TYPE_LAST = TYPE_HARDWARE_BUFFER,
		TYPE_NB = TYPE_LAST - TYPE_FIRST + 1,
	};
	static const char *getReportTypeString(int reportType) {
		switch (reportType) {
		case TYPE_NORMAL:
			return "normal";
		case TYPE_SHARED_MEMORY:
			return "shared memory";
		case TYPE_HARDWARE_BUFFER:
			return "hardware buffer";
		default:
			return "invalid";
		}
	}
	// Sensor channel events callback
	typedef void (*EventsCallback)(const SensorChannel &channel, const ASensorEvent *events, size_t eventsNb, void *priv);
	SensorChannel(ASensorManager *manager, ALooper *looper, size_t size)
		: mError(-1), mType(TYPE_INVALID), mId(-1), mManager(manager), mLooper(looper), mBuffer(nullptr),
		  mSize(size), mEventsCallback(nullptr), mEventsCallbackData(nullptr) { }
	virtual ~SensorChannel() { }
	int getError() const { return mError; }
	int getType() const { return mType; }
	int getId() const { return mId; }
	void setEventsCallback(EventsCallback cb, void *priv) { mEventsCallback = cb; mEventsCallbackData = priv; }
	virtual int32_t getDataSensorHandle(int32_t token) const = 0;
	virtual int registerSensor(ASensorRef sensor, int rateHz, int64_t latencyUs) = 0;
	virtual int disableSensor(ASensorRef sensor) = 0;

protected:
	int mError;
	int mType;
	int mId;
	ASensorManager *mManager;
	ALooper *mLooper;
	ASensorEvent *mBuffer;
	size_t mSize;
	EventsCallback mEventsCallback;
	void *mEventsCallbackData;
	void eventsCallback(const ASensorEvent *buffer, size_t size) {
		if (mEventsCallback != nullptr) {
			mEventsCallback(*this, buffer, size, mEventsCallbackData);
		}
	}
	virtual int dataCallback(int fd, int events) = 0;
	static int dataCallbackWrapper(int fd, int events, void *priv) {
		SensorChannel *chan = reinterpret_cast<SensorChannel *>(priv);
		return chan->dataCallback(fd, events);
	}
};

#endif
