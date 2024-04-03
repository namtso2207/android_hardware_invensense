/*
 * Copyright (C) 2019 InvenSense, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SENSOR_CHANNEL_MUX_H
#define SENSOR_CHANNEL_MUX_H

#include <unordered_map>

class SensorChannelMux {
public:
	struct SensorParams {
		int64_t periodNs;
		int64_t latencyNs;
	};
	static int32_t getSensorToken(int32_t handle) {
		// to ensure report token is > 0
		return (handle + 1);
	}
	SensorChannelMux() : mEnabled(false), mParams({INT64_MAX, INT64_MAX}) {};
	virtual ~SensorChannelMux() {};
	bool getEnable() const {
		return mEnabled;
	}
	const struct SensorParams getParams() const {
		return mParams;
	}
	bool isRegistered(int32_t channel) const {
		return mRequests.count(channel) == 0 ? false : true;
	}
	int registerChannel(int32_t channel, const struct SensorParams &params);
	int disableChannel(int32_t channel);

private:
	bool mEnabled;
	struct SensorParams mParams;
	std::unordered_map<int32_t, SensorParams> mRequests;
};

#endif
