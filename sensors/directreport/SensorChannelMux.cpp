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

#include <cerrno>
#include <unordered_map>

#include "SensorChannelMux.h"

static int computeMuxParams(const std::unordered_map<int32_t, SensorChannelMux::SensorParams> &requests, SensorChannelMux::SensorParams &mux)
{
	constexpr SensorChannelMux::SensorParams init = { INT64_MAX, INT64_MAX };

	if (requests.empty()) {
		return -EINVAL;
	}

	mux = init;
	for (auto &p : requests) {
		const SensorChannelMux::SensorParams &s = p.second;
		if (s.periodNs < mux.periodNs) {
			mux.periodNs = s.periodNs;
		}
		if (s.latencyNs < mux.latencyNs) {
			mux.latencyNs = s.latencyNs;
		}
	}

	return 0;
}

int SensorChannelMux::registerChannel(int32_t channel, const struct SensorParams &params)
{
	int ret;

	// Find current setting or create a new element if not existing
	// Fill it with the sensor settings
	struct SensorParams &par = mRequests[channel];
	par = params;

	// if not enable, take the parameters as is and enable
	if (!mEnabled) {
		mParams = params;
		mEnabled = true;
		return 0;
	}

	// compute and use multiplexed values of all requests
	struct SensorParams mux;
	ret = computeMuxParams(mRequests, mux);
	if (ret < 0) {
		return ret;
	}
	mParams = mux;

	return 0;
}

int SensorChannelMux::disableChannel(int32_t channel)
{
	int ret;

	// error if already disabled
	if (!mEnabled) {
		return -EINVAL;
	}

	// erase element, return error if it doesn't exist
	size_t nb = mRequests.erase(channel);
	if (nb == 0) {
		return -EINVAL;
	}

	// if empy, disable and return
	if (mRequests.empty()) {
		mEnabled = false;
		mParams = { INT64_MAX, INT64_MAX };
		return 0;
	}

	// compute and use multiplexed values of all requests
	struct SensorParams mux;
	ret = computeMuxParams(mRequests, mux);
	if (ret < 0) {
		return ret;
	}
	mParams = mux;

	return 0;
}
