#ifndef MPLLOGGER_H_
#define MPLLOGGER_H_

#include <string>
#include <unordered_map>
#include <ctime>

class MPLLogger {
public:
	enum sensor_id {
		SENSOR_ACCELEROMETER = 0x01,
		SENSOR_MAGNETOMETER = 0x02,
		SENSOR_ORIENTATION = 0x03,
		SENSOR_GYROSCOPE = 0x04,
		SENSOR_LIGHT = 0x05,
		SENSOR_PRESSURE = 0x06,
		SENSOR_TEMPERATURE = 0x07,
		SENSOR_PROXIMITY = 0x08,
		SENSOR_GRAVITY = 0x09,
		SENSOR_LINEAR_ACCELERATION = 0x0A,
		SENSOR_ROTATION_VECTOR = 0x0B,
		SENSOR_HUMIDITY = 0x0C,
		SENSOR_AMBIENT_TEMPERATURE = 0x0D,
		SENSOR_UNCAL_MAGNETOMETER = 0x0E,
		SENSOR_GAME_ROTATION_VECTOR = 0x0F,
		SENSOR_UNCAL_GYROSCOPE = 0x10,
		SENSOR_SIGNIFICANT_MOTION = 0x11,
		SENSOR_STEP_DETECTOR = 0x12,
		SENSOR_STEP_COUNTER = 0x13,
		SENSOR_GEOMAGNETIC_ROTATION_VECTOR = 0x14,
		SENSOR_HEART_RATE = 0x15,
		SENSOR_TILT_DETECTOR = 0x16,
		SENSOR_WAKE_GESTURE = 0x17,
		SENSOR_GLANCE_GESTURE = 0x18,
		SENSOR_PICK_UP_GESTURE = 0x19,
		SENSOR_RAW_ACCELEROMETER = 0x20,
		SENSOR_RAW_GYROSCOPE = 0x21,
		SENSOR_RAW_MAGNETOMETER = 0x22,
		SENSOR_RAW_TEMPERATURE = 0x23,
		SENSOR_DOUBLE_TAP = 0x30,
	};

	MPLLogger(std::string path, std::string prefix);
	~MPLLogger();

	void logEvents(const sensor_id sensor_type, const float *values, int accuracy, int64_t timestamp);
	void logEvents(const sensor_id sensor_type, const int32_t *values, int64_t timestamp);
	void logEvents(const sensor_id sensor_type, uint64_t value, int64_t timestamp);


private:
	bool mEnabled;
	FILE* mLogFile;
	std::string mPath;
	std::string mPrefix;
	std::string mFileName;
	bool mLogProp;
	time_t mLogPropTime;
	std::unordered_map<enum sensor_id, std::string> mSensorStrings = {
		{SENSOR_ACCELEROMETER, "SENSOR_ACCELEROMETER"},
		{SENSOR_MAGNETOMETER, "SENSOR_MAGNETOMETER"},
		{SENSOR_ORIENTATION, "SENSOR_ORIENTATION"},
		{SENSOR_GYROSCOPE, "SENSOR_GYROSCOPE"},
		{SENSOR_LIGHT, "SENSOR_LIGHT"},
		{SENSOR_PRESSURE, "SENSOR_PRESSURE"},
		{SENSOR_TEMPERATURE, "SENSOR_TEMPERATURE"},
		{SENSOR_PROXIMITY, "SENSOR_PROXIMITY"},
		{SENSOR_GRAVITY, "SENSOR_GRAVITY"},
		{SENSOR_LINEAR_ACCELERATION, "SENSOR_LINEAR_ACCELERATION"},
		{SENSOR_ROTATION_VECTOR, "SENSOR_ROTATION_VECTOR"},
		{SENSOR_HUMIDITY, "SENSOR_HUMIDITY"},
		{SENSOR_AMBIENT_TEMPERATURE, "SENSOR_AMBIENT_TEMPERATURE"},
		{SENSOR_UNCAL_MAGNETOMETER, "SENSOR_UNCAL_MAGNETOMETER"},
		{SENSOR_GAME_ROTATION_VECTOR, "SENSOR_GAME_ROTATION_VECTOR"},
		{SENSOR_UNCAL_GYROSCOPE, "SENSOR_UNCAL_GYROSCOPE"},
		{SENSOR_SIGNIFICANT_MOTION, "SENSOR_SIGNIFICANT_MOTION"},
		{SENSOR_STEP_DETECTOR, "SENSOR_STEP_DETECTOR"},
		{SENSOR_STEP_COUNTER, "SENSOR_STEP_COUNTER"},
		{SENSOR_GEOMAGNETIC_ROTATION_VECTOR, "SENSOR_GEOMAGNETIC_ROTATION_VECTOR"},
		{SENSOR_HEART_RATE, "SENSOR_HEART_RATE"},
		{SENSOR_TILT_DETECTOR, "SENSOR_TILT_DETECTOR"},
		{SENSOR_WAKE_GESTURE, "SENSOR_WAKE_GESTURE"},
		{SENSOR_GLANCE_GESTURE, "SENSOR_GLANCE_GESTURE"},
		{SENSOR_PICK_UP_GESTURE, "SENSOR_PICK_UP_GESTURE"},
		{SENSOR_RAW_ACCELEROMETER, "SENSOR_RAW_ACCELEROMETER"},
		{SENSOR_RAW_GYROSCOPE, "SENSOR_RAW_GYROSCOPE"},
		{SENSOR_RAW_MAGNETOMETER, "SENSOR_RAW_MAGNETOMETER"},
		{SENSOR_RAW_TEMPERATURE, "SENSOR_RAW_TEMPERATURE"},
		{SENSOR_DOUBLE_TAP, "SENSOR_DOUBLE_TAP"},
	};

	void createLogFileName();
	int startLog(const sensor_id sensor_type, int64_t timestamp);
};

#endif