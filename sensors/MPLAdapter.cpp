/*
 * Copyright (C) 2014-2021 InvenSense, Inc.
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

#include <dirent.h>
#include <string.h>
#include <stdio.h>
#include "SensorBase.h"
#include <fcntl.h>
#include "Log.h"
#include "data_builder.h"
#include "hal_outputs.h"
#include "sensors.h"
#include "MPLAdapter.h"
#ifdef INV_USE_ALGO_LIB
#include "MPLBuilder.h"
#endif

static int adapter_mode;

void adapter_set_mode(int mode)
{
    adapter_mode = mode;
}

void adapter_init(void (*cb)(int32_t rw, uint32_t* data), const char *chip)
{
    (void)cb;
    (void)chip;

#ifdef INV_USE_ALGO_LIB
    if (adapter_mode != dmp) {
        mpl_init_cal_lib();
        LOGI("Initialize MPL calibration library");
    }
#endif
}

int adapter_gyro_reset_timestamp(void)
{
#ifdef INV_USE_ALGO_LIB
    /* call the same API for all cases for now */
    mpl_gyro_reset_timestamp();
#endif

    return 0;
}

int adapter_accel_reset_timestamp(void)
{
#ifdef INV_USE_ALGO_LIB
    /* call the same API for all cases for now */
    if (adapter_mode & mpl_accel_cal)
        mpl_accel_reset_timestamp();
    else
        mpl_accel_reset_timestamp();
#endif

    return 0;
}

int adapter_compass_reset_timestamp(void)
{
#ifdef INV_USE_ALGO_LIB
    /* call the same API for all cases for now */
    if (adapter_mode & mpl_compass_cal)
        mpl_compass_reset_timestamp();
    else
        mpl_compass_reset_timestamp();
#endif

    return 0;
}

int adapter_build_temp(int *gyro, int temp)
{
    (void)gyro, (void)temp;
    return 0;
}

int adapter_build_pressure(int pressure, int status, long long timestamp)
{
    inv_build_pressure(pressure, status, timestamp);
    return 0;
}

int adapter_build_gyro_dmp(int *gyro, int temperature, int status, long long timestamp)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_gyro_cal)
        mpl_build_gyro(gyro, temperature, timestamp);
    else
#else
    (void)temperature;
#endif
        inv_build_gyro_dmp(gyro, status, timestamp);

    return 0;
}

int adapter_build_eis_gyro(int *eis_gyro, int temperature, int status, long long timestamp)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_gyro_cal)
        mpl_build_eis_gyro(eis_gyro, temperature, timestamp);
    else
#else
    (void)temperature;
#endif
        inv_build_eis_gyro(eis_gyro, status, timestamp);

    return 0;
}

int adapter_build_eis_authentication(int *eis_authentication, int status, long long timestamp)
{
    inv_build_eis_authentication(eis_authentication, status, timestamp);

    return 0;
}

int adapter_build_accel_dmp(int *accel, int temperature, int status, long long timestamp)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_accel_cal)
        mpl_build_accel(accel, temperature, timestamp);
    else
#else
    (void)temperature;
#endif
        inv_build_accel_dmp(accel, status, timestamp);

    return 0;
}

int adapter_build_compass(int *compass, int status, long long timestamp)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_compass_cal)
        mpl_build_compass(compass, timestamp);
    else
#endif
        inv_build_compass(compass, status, timestamp);

    return 0;
}

int adapter_build_quat(int *quat, int status, long long timestamp)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_quat)
        return 0;
    else
#endif
        inv_build_quat(quat, status, timestamp);

    return 0;
}

int adapter_get_sensor_type_orientation(float *values, int8_t *accuracy, int64_t *timestamp)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_quat)
        return mpl_get_sensor_type_orientation(values, accuracy, timestamp);
    else
#endif
        return inv_get_sensor_type_orientation(values, accuracy, timestamp);
}

int adapter_get_sensor_type_accelerometer(float *values, int8_t *accuracy, int64_t *timestamp)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_accel_cal)
        return mpl_get_sensor_type_accelerometer(values, accuracy, timestamp);
    else
#endif
        return inv_get_sensor_type_accelerometer(values, accuracy, timestamp);
}

int adapter_get_sensor_type_accelerometer_raw(float *values, int8_t *accuracy, int64_t *timestamp)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_accel_cal)
        return mpl_get_sensor_type_accelerometer_raw(values, accuracy, timestamp);
    else
#else
    (void)values, (void)accuracy, (void)timestamp;
#endif
        return 0;
}

int adapter_get_sensor_type_accelerometer_custom(float *values, int8_t *accuracy, int64_t *timestamp)
{
    return inv_get_sensor_type_accelerometer_custom(values, accuracy, timestamp);
}

int adapter_get_sensor_type_gyroscope(float *values, int8_t *accuracy, int64_t *timestamp)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_gyro_cal)
        return mpl_get_sensor_type_gyroscope(values, accuracy, timestamp);
    else
#endif
        return inv_get_sensor_type_gyroscope(values, accuracy, timestamp);
}

int adapter_get_sensor_type_gyroscope_raw(float *values, int8_t *accuracy, int64_t *timestamp)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_gyro_cal)
        return mpl_get_sensor_type_gyroscope_raw(values, accuracy, timestamp);
    else
#endif
        return inv_get_sensor_type_gyroscope_raw(values, accuracy, timestamp);
}

int adapter_get_sensor_type_gyroscope_raw_custom(float *values, int8_t *accuracy, int64_t *timestamp)
{
    return inv_get_sensor_type_gyroscope_raw_custom(values, accuracy, timestamp);
}

int adapter_get_sensor_type_eis_gyroscope(float *values, int8_t *accuracy, int64_t *timestamp)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_gyro_cal)
        return mpl_get_sensor_type_eis_gyroscope(values, accuracy, timestamp);
    else
#endif
        return inv_get_sensor_type_eis_gyroscope(values, accuracy, timestamp);
}

int adapter_get_sensor_type_eis_authentication(float *values, int8_t *accuracy, int64_t *timestamp)
{
    return inv_get_sensor_type_eis_authentication(values, accuracy, timestamp);
}

int adapter_get_sensor_type_magnetic_field(float *values, int8_t *accuracy, int64_t *timestamp)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_compass_cal)
        return mpl_get_sensor_type_magnetic_field(values, accuracy, timestamp);
    else
#endif
        return inv_get_sensor_type_magnetic_field(values, accuracy, timestamp);
}

int adapter_get_sensor_type_magnetic_field_raw(float *values, int8_t *accuracy, int64_t *timestamp)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_compass_cal)
        return mpl_get_sensor_type_magnetic_field_raw(values, accuracy, timestamp);
    else
#endif
        return inv_get_sensor_type_magnetic_field_raw(values, accuracy, timestamp);
}

int adapter_get_sensor_type_magnetic_field_raw_custom(float *values, int8_t *accuracy, int64_t *timestamp)
{
    return inv_get_sensor_type_magnetic_field_raw_custom(values, accuracy, timestamp);
}

int adapter_get_sensor_type_rotation_vector(float *values, int8_t *accuracy, int64_t *timestamp)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_quat)
        return mpl_get_sensor_type_rotation_vector(values, accuracy, timestamp);
    else
#endif
        return inv_get_sensor_type_rotation_vector(values, accuracy, timestamp);
}

int adapter_get_sensor_type_linear_acceleration(float *values, int8_t *accuracy, int64_t *timestamp)
{
    int accel_cal[3] = {0, };

    if (adapter_mode & mpl_accel_cal) {
#ifdef INV_USE_ALGO_LIB
        inv_get_cal_accel(accel_cal);
#endif
        inv_build_accel_dmp(accel_cal, INV_CALIBRATED, (long long)timestamp);
    }
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_quat)
        return mpl_get_sensor_type_linear_acceleration(values, accuracy, timestamp);
    else
#endif
        return inv_get_sensor_type_linear_acceleration(values, accuracy, timestamp);
}

int adapter_get_sensor_type_game_rotation_vector(float *values, int8_t *accuracy, int64_t *timestamp)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_quat)
        return mpl_get_sensor_type_game_rotation_vector(values, accuracy, timestamp);
    else
#endif
        return inv_get_sensor_type_rotation_vector_6_axis(values, accuracy, timestamp);
}

int adapter_get_sensor_type_lpq(float *values, int8_t *accuracy, int64_t *timestamp)
{
    return inv_get_sensor_type_lpq(values, accuracy, timestamp);
}

int adapter_get_sensor_type_geomagnetic_rotation_vector(float *values, int8_t *accuracy, int64_t *timestamp)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_quat)
        return mpl_get_sensor_type_geomagnetic_rotation_vector(values, accuracy, timestamp);
    else
#endif
        return inv_get_sensor_type_geomagnetic_rotation_vector(values, accuracy, timestamp);
}

int adapter_get_sensor_type_gravity(float *values, int8_t *accuracy, int64_t *timestamp)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_quat)
        return mpl_get_sensor_type_gravity(values, accuracy, timestamp);
    else
#endif
        return inv_get_sensor_type_gravity(values, accuracy, timestamp);
}

int adapter_get_sensor_type_heading(float *values, int8_t *accuracy, int64_t *timestamp)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_quat)
        return mpl_get_sensor_type_heading(values, accuracy, timestamp);
    else
#endif
        return inv_get_sensor_type_orientation(values, accuracy, timestamp);
}

void adapter_set_sample_rate(int sample_rate_us, int id)
{
    switch(id) {
        case ID_GY:
        case ID_RG:
        case ID_GYW:
        case ID_RGW:
#ifdef INV_USE_ALGO_LIB
            if (adapter_mode & mpl_gyro_cal)
                mpl_set_sample_rate(sample_rate_us, id);
            else
#endif
                inv_set_gyro_sample_rate(sample_rate_us);
            break;
        case ID_A:
        case ID_RA:
        case ID_AW:
        case ID_RAW:
#ifdef INV_USE_ALGO_LIB
            if (adapter_mode & mpl_accel_cal)
                mpl_set_sample_rate(sample_rate_us, id);
            else
#endif
                inv_set_accel_sample_rate(sample_rate_us);
            break;
        case ID_M:
        case ID_RM:
        case ID_MW:
        case ID_RMW:
#ifdef INV_USE_ALGO_LIB
            if (adapter_mode & mpl_compass_cal)
                mpl_set_sample_rate(sample_rate_us, id);
            else
#endif
                inv_set_compass_sample_rate(sample_rate_us);
            break;
        case ID_RV:
        case ID_RVW:
        case ID_GRV:
        case ID_GRVW:
        case ID_GMRV:
        case ID_GMRVW:
        case ID_HEADING:
        case ID_HEADINGW:
#ifdef INV_USE_ALGO_LIB
            if (adapter_mode & mpl_quat)
                mpl_set_sample_rate(sample_rate_us, id);
            else
#endif
                inv_set_quat_sample_rate(sample_rate_us);
            break;
        case ID_P:
        case ID_SC:
        case ID_T:
        case ID_PICK:
        case ID_PW:
        case ID_SCW:
#ifdef INV_USE_ALGO_LIB
            if (adapter_mode & (mpl_smd | mpl_pedo | mpl_pickup | mpl_tilt | mpl_tap | mpl_md))
                mpl_set_sample_rate(sample_rate_us, id);
#endif
            break;
    }
}

void adapter_set_gyro_sample_rate(int sample_rate_us, int id)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_gyro_cal)
        return mpl_set_sample_rate(sample_rate_us, id);
    else
#else
    (void)id;
#endif
        return inv_set_compass_sample_rate(sample_rate_us);
}

void adapter_set_compass_sample_rate(int sample_rate_us, int id)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_compass_cal)
        return mpl_set_sample_rate(sample_rate_us, id);
    else
#else
    (void)id;
#endif
        return inv_set_compass_sample_rate(sample_rate_us);
}

void adapter_set_accel_sample_rate(int sample_rate_us, int id)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_accel_cal)
        return mpl_set_sample_rate(sample_rate_us, id);
    else
#else
    (void)id;
#endif
        return inv_set_accel_sample_rate(sample_rate_us);
}

void adapter_set_linear_acceleration_sample_rate(int sample_rate_us, int id)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_quat)
        return mpl_set_sample_rate(sample_rate_us, id);
    else
#else
    (void)id;
#endif
        return inv_set_linear_acceleration_sample_rate(sample_rate_us);
}

void adapter_set_orientation_sample_rate(int sample_rate_us, int id)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_quat)
        return mpl_set_sample_rate(sample_rate_us, id);
    else
#else
    (void)id;
#endif
        return inv_set_linear_acceleration_sample_rate(sample_rate_us);
}

void adapter_set_gravity_sample_rate(int sample_rate_us, int id)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_quat)
        return mpl_set_sample_rate(sample_rate_us, id);
    else
#else
    (void)id;
#endif
        return inv_set_gravity_sample_rate(sample_rate_us);
}

void adapter_set_gyro_orientation_and_scale(int orientation, int scale)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_gyro_cal)
        return mpl_set_gyro_orientation_and_scale(orientation, scale >> 15);
    else
#endif
        return inv_set_gyro_orientation_and_scale(orientation, scale);
}

void adapter_set_accel_orientation_and_scale(int orientation, int scale)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_accel_cal) {
        if (!(adapter_mode & mpl_quat))
            inv_set_accel_orientation_and_scale(orientation, scale);
        return mpl_set_accel_orientation_and_scale(orientation, scale >> 15);
    } else
#endif
        return inv_set_accel_orientation_and_scale(orientation, scale);
}

void adapter_set_compass_orientation_and_scale(int orientation, int scale, int *sensitivity, int *softIron)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_compass_cal)
        return mpl_set_compass_orientation_and_scale(orientation, scale >> 14, sensitivity, softIron);
    else
#else
    (void)sensitivity, (void)softIron;
#endif
        return inv_set_compass_orientation_and_scale(orientation, scale);
}

void adapter_get_mpl_gyro_bias(int *bias, int *temperature, int *accuracy, int *updated)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_gyro_cal)
        mpl_get_mpl_gyro_bias(bias, accuracy, updated);
    else
#else
    (void)accuracy, (void)updated;
#endif
        inv_get_mpl_gyro_bias(bias, temperature);
}

void adapter_set_mpl_gyro_bias(int *bias, int accuracy)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_gyro_cal)
        mpl_set_mpl_gyro_bias(bias, accuracy);
    else
#endif
        inv_set_mpl_gyro_bias(bias, accuracy);
}

void adapter_get_mpl_accel_bias(int *bias, int *temperature, int *accuracy, int *updated)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_accel_cal)
        mpl_get_mpl_accel_bias(bias, accuracy, updated);
    else
#else
    (void)accuracy, (void)updated;
#endif
        inv_get_mpl_accel_bias(bias, temperature);
}

void adapter_set_mpl_accel_bias(int *bias, int accuracy)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_accel_cal)
        mpl_set_mpl_accel_bias(bias, accuracy);
    else
#else
    (void)accuracy;
#endif
        inv_set_accel_bias_mask(bias, 3,7);
}

void adapter_get_mpl_compass_bias(int *bias)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_compass_cal)
        mpl_get_mpl_compass_bias(bias);
#else
    (void)bias;
#endif
}

void adapter_set_mpl_compass_bias(int *bias, int accuracy)
{
#ifdef INV_USE_ALGO_LIB
    if (adapter_mode & mpl_compass_cal)
        mpl_set_mpl_compass_bias(bias, accuracy);
#else
    (void)bias, (void)accuracy;
#endif
}

void adapter_set_version(int version_number)
{
#ifdef INV_USE_ALGO_LIB
    mpl_set_lib_version(version_number);
#else
    (void)version_number;
#endif
}

int adapter_get_version()
{
#ifdef INV_USE_ALGO_LIB
    return mpl_get_lib_version();
#else
    return 0;
#endif
}

int adapter_get_calibration_mode(char *ChipID)
{
    if (!strcmp(ChipID, "ICM20648"))
       return dmp;
    else if (!strcmp(ChipID, "ICM20608D"))
        return dmp | mpl_accel_cal | mpl_compass_cal;
    else if (!strcmp(ChipID, "ICM20602"))
        return mpl_all;
    else if (!strcmp(ChipID, "ICM20690"))
        return mpl_all;
    else if (!strcmp(ChipID, "IAM20680"))
        return mpl_all;
    else if (!strcmp(ChipID, "ICM42600"))
        return dmp | mpl_accel_cal | mpl_gyro_cal | mpl_compass_cal | mpl_quat | mpl_md;
    else if (!strcmp(ChipID, "ICM43600"))
        return dmp | mpl_accel_cal | mpl_gyro_cal | mpl_compass_cal | mpl_quat | mpl_md | mpl_pickup;
    else if (!strcmp(ChipID, "ICM45600"))
        return dmp | mpl_accel_cal | mpl_gyro_cal | mpl_compass_cal | mpl_quat | mpl_md;
    else
        return dmp;
}

int adapter_get_internal_sampling_rate(const char *ChipID)
{
    if (!strcmp(ChipID, "ICM20648"))
        return 1125;
    else
        return 1000;
}
