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

#ifndef ANDROID_MPL_BUILDER_H
#define ANDROID_MPL_BUILDER_H

#include <stdint.h>
#include "invn_algo_agm.h"
#include "MPLLogger.h"

struct mpl_sensor_data {
    InvnAlgoAGMStruct algo;
    InvnAlgoAGMConfig algo_conf;

    int gyro_data[3];
    int gyro_accuracy;
    int accel_data[3];
    int accel_accuracy;
    int compass_data[3];
    int compass_accuracy;
    int eis_calibrated[4];
    int eis_gyro_bias[3];

    int gyro_bias_updated;
    int accel_bias_updated;

    int six_axis_data[4];
    int nine_axis_data[4];
    int six_axis_compass_data[4];
    int linear_acceleration[3];
    int gravity[3];

    short gyro_raw_data[3];
    short accel_raw_data[3];
    short compass_raw_data[3];

    int gyro_scale;
    int accel_scale;
    int compass_scale;

    float am_accuracy_rad;      // 3-sigma accuracy
    float agm_accuracy_rad;     // 3-sigma accuracy
    float heading_accuracy_rad; // 1-sigma accuracy

    int gyro_matrix_scalar;
    int accel_matrix_scalar;
    int compass_matrix_scalar;

    int compass_soft_iron[9];
    int compass_sens[9];

    int gyro_bias[3];
    int accel_bias[3];
    int compass_bias[3];

    int gyro_sample_rate;
    int accel_sample_rate;
    int compass_sample_rate;

    int64_t gyro_timestamp;
    int64_t eis_gyro_timestamp;
    int64_t accel_timestamp;
    int64_t compass_timestamp;
    int64_t quat6_timestamp;
    int64_t quat6_compass_timestamp;
    int64_t quat9_timestamp;
    int64_t la_timestamp;
    int64_t gravity_timestamp;
    int64_t or_timestamp;

    MPLLogger MPLLog{"/data/vendor/sensor", "AlgoSensors"};
};

void mpl_init_cal_lib();
int mpl_gyro_reset_timestamp();
int mpl_accel_reset_timestamp();
int mpl_compass_reset_timestamp();
int mpl_build_accel(int *accel, int temperature, long long timestamp);
int mpl_build_compass(int *compass, long long timestamp);
int mpl_build_gyro(int *gyro, int temperature, long long timestamp);
int mpl_build_eis_gyro(int *gyro, int temperature, long long timestamp);

int mpl_get_sensor_type_magnetic_field(float *values, int8_t *accuracy, int64_t *timestamp);
int mpl_get_sensor_type_rotation_vector(float *values, int8_t *accuracy, int64_t *timestamp);
int mpl_get_sensor_type_accelerometer(float *values, int8_t *accuracy, int64_t * timestamp);
int mpl_get_sensor_type_accelerometer_raw(float *values, int8_t *accuracy, int64_t * timestamp);
int mpl_get_sensor_type_gyroscope(float *values, int8_t *accuracy, int64_t *timestamp);
int mpl_get_sensor_type_eis_gyroscope(float *values, int8_t *accuracy, int64_t *timestamp);
int mpl_get_sensor_type_gyroscope_raw(float *values, int8_t *accuracy, int64_t *timestamp);
int mpl_get_sensor_type_magnetic_field(float *values, int8_t *accuracy, int64_t *timestamp);
int mpl_get_sensor_type_magnetic_field_raw(float *values, int8_t *accuracy, int64_t *timestamp);
int mpl_get_sensor_type_game_rotation_vector(float *values, int8_t *accuracy, int64_t *timestamp);
int mpl_get_sensor_type_geomagnetic_rotation_vector(float *values, int8_t *accuracy, int64_t *timestamp);
int mpl_get_sensor_type_linear_acceleration(float *values, int8_t *accuracy, int64_t *timestamp);
int mpl_get_sensor_type_gravity(float *values, int8_t *accuracy, int64_t *timestamp);
int mpl_get_sensor_type_orientation(float *values, int8_t *accuracy, int64_t *timestamp);
int mpl_get_sensor_type_heading(float *values, int8_t *accuracy, int64_t *timestamp);

void mpl_set_sample_rate(int sample_rate_us, int id);
void mpl_set_accel_orientation_and_scale(int orientation, int scale);
void mpl_set_gyro_orientation_and_scale(int orientation, int scale);
void mpl_set_compass_orientation_and_scale(int orientation, int scale, int *sensitivity, int *softIron);

int inv_get_cal_accel(int *data);

int mpl_get_mpl_gyro_bias(int *bias, int *accuracy, int *updated);
int mpl_set_mpl_gyro_bias(int *bias, int accuracy);
int mpl_get_mpl_accel_bias(int *bias, int *accuracy, int *updated);
int mpl_set_mpl_accel_bias(int *bias, int accuracy);
int mpl_get_mpl_compass_bias(int *bias);
int mpl_set_mpl_compass_bias(int *bias, int accuracy);

void mpl_set_lib_version(int version_number);
int mpl_get_lib_version();

#endif //  ANDROID_MPL_BUILDER_H
