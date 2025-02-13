/*
 * Copyright (C) 2014-2017 InvenSense, Inc.
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

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>
#include <pthread.h>
#include <errno.h>
#include <sys/eventfd.h>

#include <hardware/sensors.h>

#include "SensorBase.h"
#include "MPLSensor.h"
#include "Log.h"

/*****************************************************************************/
/* The SENSORS Module */

#define LOCAL_SENSORS (TotalNumSensors)

static MPLSensor *gMPLSensor;
static int data_injection_mode;
static int data_injection_supported;

static struct sensor_t sSensorList[LOCAL_SENSORS];
static int sensors = (sizeof(sSensorList) / sizeof(sensor_t));

static int open_sensors(const struct hw_module_t* module, const char* id,
                        struct hw_device_t** device);

static int sensors__get_sensors_list(struct sensors_module_t* module,
                                     struct sensor_t const** list)
{
    (void) module;
    *list = sSensorList;
    return sensors;
}

static int sensors__set_operation_mode(unsigned int mode)
{
    data_injection_mode = mode;
    gMPLSensor->setDataInjectionMode(mode);
    return data_injection_supported;
}

static struct hw_module_methods_t sensors_module_methods = {
        .open = open_sensors,
};

struct sensors_module_t HAL_MODULE_INFO_SYM = {
        .common = {
                .tag = HARDWARE_MODULE_TAG,
                .module_api_version = SENSORS_MODULE_API_VERSION_0_1,
                .hal_api_version = HARDWARE_HAL_API_VERSION,
                .id = SENSORS_HARDWARE_MODULE_ID,
                .name = "Invensense module",
                .author = "Invensense Inc.",
                .methods = &sensors_module_methods,
                .dso = NULL,
                .reserved = {0},
        },
        .get_sensors_list = sensors__get_sensors_list,
        .set_operation_mode = sensors__set_operation_mode,
};

struct sensors_poll_context_t {
    sensors_poll_device_1_t device; // must be first

    sensors_poll_context_t();
    ~sensors_poll_context_t();
    int activate(int handle, int enabled);
    int pollEvents(sensors_event_t* data, int count);
    int batch(int handle, int flags, int64_t period_ns, int64_t timeout);
    int flush(int handle);
    int inject_sensor_data(const sensors_event_t* data);
#ifdef DIRECT_REPORT
    int register_direct_channel(const struct sensors_direct_mem_t* mem, int channel_handle);
    int config_direct_report(int sensor_handle, int channel_handle, const struct sensors_direct_cfg_t *config);
#endif

private:
    enum {
        exitEvent = 0,
        mpl,
        compass,
        pressure,
        dmpSign,
        dmpPed,
        dmpTilt,
        dmpPickup,
        numFds,
    };

    struct pollfd mPollFds[numFds];
    int exitFd;
    MPLSensor *mSensor;
    CompassSensor *mCompassSensor;
    PressureSensor *mPressureSensor;
};

/******************************************************************************/

sensors_poll_context_t::sensors_poll_context_t() {
    VFUNC_LOG;

    mCompassSensor = new CompassSensor();
    mPressureSensor = new PressureSensor();
    mSensor = new MPLSensor(mCompassSensor, mPressureSensor);

    gMPLSensor = mSensor;
    data_injection_supported = mSensor->isDataInjectionSupported();

    // populate the sensor list
    sensors = mSensor->populateSensorList(sSensorList, sizeof(sSensorList));

    mPollFds[exitEvent].fd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    mPollFds[exitEvent].events = POLLIN;
    mPollFds[exitEvent].revents = 0;

    mPollFds[mpl].fd = mSensor->getFd();
    mPollFds[mpl].events = POLLIN;
    mPollFds[mpl].revents = 0;

    mPollFds[compass].fd = mCompassSensor->getFd();
    mPollFds[compass].events = POLLIN;
    mPollFds[compass].revents = 0;

    mPollFds[pressure].fd = mPressureSensor->getFd();
    mPollFds[pressure].events = POLLIN;
    mPollFds[pressure].revents = 0;

    mPollFds[dmpSign].fd = mSensor->getDmpSignificantMotionFd();
    mPollFds[dmpSign].events = POLLPRI;
    mPollFds[dmpSign].revents = 0;

    mPollFds[dmpPed].fd = mSensor->getDmpPedometerFd();
    mPollFds[dmpPed].events = POLLPRI;
    mPollFds[dmpPed].revents = 0;

    mPollFds[dmpTilt].fd = mSensor->getDmpTiltFd();
    mPollFds[dmpTilt].events = POLLPRI;
    mPollFds[dmpTilt].revents = 0;

    mPollFds[dmpPickup].fd = mSensor->getDmpPickupFd();
    mPollFds[dmpPickup].events = POLLPRI;
    mPollFds[dmpPickup].revents = 0;

    exitFd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
}

sensors_poll_context_t::~sensors_poll_context_t() {
    VFUNC_LOG;

    struct pollfd pollExit = {
        .fd = exitFd,
        .events = POLLIN,
        .revents = 0,
    };
    int64_t exitVal = 1;
    int ret;

    // disable all sensors
    for (int i = 0; i < sensors; ++i) {
        mSensor->enable(sSensorList[i].handle, 0);
    }

    // exit poll thread
    ret = write(mPollFds[exitEvent].fd, &exitVal, sizeof(exitVal));
    LOGE_IF(ret <= 0, "write to poll exitEvent failed error %d", ret);
    if (ret > 0) {
        ret = poll(&pollExit, 1, 3 * 1000);
        LOGE_IF(ret <= 0, "exit poll error %d", ret);
        if (ret == 1) {
            ret = read(exitFd, &exitVal, sizeof(exitVal));
        }
    }

    close(mPollFds[exitEvent].fd);
    close(exitFd);
    delete mSensor;
    delete mCompassSensor;
    delete mPressureSensor;
}

int sensors_poll_context_t::activate(int handle, int enabled) {
    VFUNC_LOG;

    int err;
    err = mSensor->enable(handle, enabled);
    return err;
}

int sensors_poll_context_t::pollEvents(sensors_event_t *data, int count)
{
    VHANDLER_LOG;

    int nbEvents = 0;
    int nb, polltime;
    int ret;

    polltime = mSensor->getPollTime();

    // look for new events
    do {
        nb = poll(mPollFds, numFds, polltime);
        LOGI_IF(0, "poll nb=%d, count=%d, pt=%d", nb, count, polltime);
        if (nb < 0)
            return -errno;
        if (nb > 0) {
            for (int i = 0; count && i < numFds; i++) {
                if (mPollFds[i].revents & (POLLIN | POLLPRI)) {
                    nb = 0;
                    if (i == exitEvent) {
                        int64_t exitVal = 1;
                        ret = write(exitFd, &exitVal, sizeof(exitVal));
                        LOGE_IF(ret <= 0, "poll thread exitFd write error %d", errno);
                        return 0;
                    } else if (i == mpl) {
                        mSensor->buildMpuEvent();
                        mPollFds[i].revents = 0;
                    } else if (i == compass) {
                        mSensor->buildCompassEvent();
                        mPollFds[i].revents = 0;
                    } else if (i == pressure) {
                        mSensor->buildPressureEvent();
                        mPollFds[i].revents = 0;
                    } else if (i == dmpSign) {
                        nb = mSensor->readDmpSignificantMotionEvents(data, count);
                        mPollFds[i].revents = 0;
                        count -= nb;
                        nbEvents += nb;
                        data += nb;
                    } else if (i == dmpPed) {
                        nb = mSensor->readDmpPedometerEvents(data, count, ID_P);
                        mPollFds[i].revents = 0;
                        count -= nb;
                        nbEvents += nb;
                        data += nb;
                    }
                    else if (i == dmpTilt) {
                        nb = mSensor->readDmpTiltEvents(data, count);
                        mPollFds[i].revents = 0;
                        count -= nb;
                        nbEvents += nb;
                        data += nb;
                    }
                    else if (i == dmpPickup) {
                        nb = mSensor->readDmpPickupEvents(data, count);
                        mPollFds[i].revents = 0;
                        count -= nb;
                        nbEvents += nb;
                        data += nb;
                    }

                    if(nb == 0) {
                        nb = mSensor->readEvents(data, count);
                        LOGI_IF(0, "sensors_mpl:readEvents() - "
                                "i=%d, nb=%d, count=%d, nbEvents=%d, "
                                "data->timestamp=%lld, data->data[0]=%f,",
                                i, nb, count, nbEvents, (long long)(data->timestamp),
                                data->data[0]);
                        if (nb > 0) {
                            count -= nb;
                            nbEvents += nb;
                            data += nb;
                        }
                    }
                }
            }
        }
        /* poll step count */
        mSensor->trigDmpPedometerCountRead();
    } while (nbEvents == 0);

    return nbEvents;
}

int sensors_poll_context_t::batch(int handle, int flags, int64_t period_ns,
                                  int64_t timeout)
{
    VFUNC_LOG;
    return mSensor->batch(handle, flags, period_ns, timeout);
}

int sensors_poll_context_t::flush(int handle)
{
    VFUNC_LOG;
    return mSensor->flush(handle);
}

int sensors_poll_context_t::inject_sensor_data(const sensors_event_t *data)
{
    VFUNC_LOG;
    return mSensor->inject_sensor_data(data);
}

#ifdef DIRECT_REPORT
int sensors_poll_context_t::register_direct_channel(const struct sensors_direct_mem_t* mem,
                                                    int channel_handle)
{
    VFUNC_LOG;
    return mSensor->register_direct_channel(mem, channel_handle);
}

int sensors_poll_context_t::config_direct_report(int sensor_handle, int channel_handle,
                                                 const struct sensors_direct_cfg_t *config)
{
    VFUNC_LOG;
    return mSensor->config_direct_report(sensor_handle, channel_handle, config);
}
#endif

/******************************************************************************/

static int poll__close(struct hw_device_t *dev)
{
    VFUNC_LOG;
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    if (ctx) {
        delete ctx;
    }
    return 0;
}

static int poll__activate(struct sensors_poll_device_t *dev,
                          int handle, int enabled)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->activate(handle, enabled);
}

static int poll__poll(struct sensors_poll_device_t *dev,
                      sensors_event_t* data, int count)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->pollEvents(data, count);
}

static int poll__setDelay(struct sensors_poll_device_t *dev,
                      int handle, int64_t period_ns)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->batch(handle, 0, period_ns, 0);
}

static int poll__batch(struct sensors_poll_device_1 *dev,
                      int handle, int flags, int64_t period_ns, int64_t timeout)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->batch(handle, flags, period_ns, timeout);
}

static int poll__flush(struct sensors_poll_device_1 *dev,
                      int handle)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->flush(handle);
}

static int poll__inject_sensor_data(struct sensors_poll_device_1 *dev,
                      const sensors_event_t *data)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->inject_sensor_data(data);
}

#ifdef DIRECT_REPORT
static int poll__register_direct_channel(struct sensors_poll_device_1 *dev,
                      const struct sensors_direct_mem_t* mem, int channel_handle)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->register_direct_channel(mem, channel_handle);
}

static int poll__config_direct_report(struct sensors_poll_device_1 *dev,
                      int sensor_handle, int channel_handle, const struct sensors_direct_cfg_t *config)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->config_direct_report(sensor_handle, channel_handle, config);
}
#endif

/******************************************************************************/

/** Open a new instance of a sensor device using name */
static int open_sensors(const struct hw_module_t* module, const char* id,
                        struct hw_device_t** device)
{
    VFUNC_LOG;
    int status = -EINVAL;
    (void) id;

    sensors_poll_context_t *dev = new sensors_poll_context_t();

    memset(&dev->device, 0, sizeof(sensors_poll_device_1));

    dev->device.common.tag          = HARDWARE_DEVICE_TAG;
    dev->device.common.version      = SENSORS_DEVICE_API_VERSION_1_4;
    dev->device.common.module       = const_cast<hw_module_t*>(module);
    dev->device.common.close        = poll__close;
    dev->device.activate            = poll__activate;
    dev->device.poll                = poll__poll;
    dev->device.setDelay            = poll__setDelay;
    dev->device.batch               = poll__batch;
    dev->device.flush               = poll__flush;
    dev->device.inject_sensor_data  = poll__inject_sensor_data;
#ifdef DIRECT_REPORT
    dev->device.register_direct_channel = poll__register_direct_channel;
    dev->device.config_direct_report    = poll__config_direct_report;
#endif

    *device = &dev->device.common;
    status = 0;

    return status;
}
