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

#include <dirent.h>
#include <string.h>
#include <fcntl.h>
#include <math.h>

#include "invn_algo_agm.h"
#include "ml_math_func.h"
#include "Log.h"
#include "sensors.h"
#include "MPLLogger.h"
#include "MPLBuilder.h"

#define BUILDER_VERBOSE 0

// uT from Q16
#define COMPASS_CONVERSION 1.52587890625e-005f
// m/s² from g on Q16
#define ACCEL_CONVERSION 1.49637603759765625e-004f
// rad/s from dps on Q16
#define GYRO_CONVERSION 2.66316109007923824604e-007f
// quaternion in Q30
#define INV_TWO_POWER_NEG_30 9.313225746154785e-010f

// convert Q16 to g/uT
#define CONVERSION_Q16 0.0000152587890625f

static struct mpl_sensor_data sensor_data;

static int inv_init_lib()
{
    InvnAlgoAGMConfig * const algo_conf = &sensor_data.algo_conf;
    const InvnAlgoAGM_ModeId_t algo_mode_id =
#ifdef INV_FUSION_AGM_AUTOMOTIVE
            INVN_ALGO_AGM_AUTOMOTIVE;
#else
            INVN_ALGO_AGM_MOBILE;
#endif

    invn_algo_agm_generate_config(algo_conf, algo_mode_id);
    algo_conf->acc_bias_q16 = sensor_data.accel_bias;
    algo_conf->gyr_bias_q16 = sensor_data.gyro_bias;
    algo_conf->mag_bias_q16 = sensor_data.compass_bias;
    algo_conf->acc_fsr = sensor_data.accel_scale > 0 ? sensor_data.accel_scale : 8;
    algo_conf->gyr_fsr = sensor_data.gyro_scale > 0 ? sensor_data.gyro_scale : 2000;
    algo_conf->acc_odr_us = sensor_data.accel_sample_rate > 0 ? sensor_data.accel_sample_rate : 20000;
    algo_conf->gyr_odr_us = sensor_data.gyro_sample_rate > 0 ? sensor_data.gyro_sample_rate : 20000;
    algo_conf->mag_sc_q16 = sensor_data.compass_scale > 0 ? sensor_data.compass_scale : 1 << 16;
    algo_conf->mag_odr_us = sensor_data.compass_sample_rate > 0 ? sensor_data.compass_sample_rate : 20000;
    algo_conf->temp_sensitivity = (1L << 30) / 100;
    algo_conf->temp_offset = 0;
    algo_conf->acc_accuracy = sensor_data.accel_accuracy;
    algo_conf->gyr_accuracy = sensor_data.gyro_accuracy;
    algo_conf->mag_accuracy = sensor_data.compass_accuracy;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: fusion config: "
            "acc_bias(%d, %d, %d - %d), gyr_bias(%d, %d, %d - %d), mag_bias(%d, %d, %d - %d) "
            "acc_fsr=%d, gyr_fsr=%d, acc_odr=%d, gyr_odr=%d, mag_odr=%d",
            algo_conf->acc_bias_q16[0], algo_conf->acc_bias_q16[1], algo_conf->acc_bias_q16[2], algo_conf->acc_accuracy,
            algo_conf->gyr_bias_q16[0], algo_conf->gyr_bias_q16[1], algo_conf->gyr_bias_q16[2], algo_conf->gyr_accuracy,
            algo_conf->mag_bias_q16[0], algo_conf->mag_bias_q16[1], algo_conf->mag_bias_q16[2], algo_conf->mag_accuracy,
            algo_conf->acc_fsr, algo_conf->gyr_fsr, algo_conf->acc_odr_us, algo_conf->gyr_odr_us, algo_conf->mag_odr_us);

    return invn_algo_agm_init(&sensor_data.algo, algo_conf);
}

static void inv_set_sample_rate()
{
    InvnAlgoAGMConfig *algo_conf = &sensor_data.algo_conf;

    algo_conf->acc_odr_us = sensor_data.accel_sample_rate > 0 ? sensor_data.accel_sample_rate : 20000;
    algo_conf->gyr_odr_us = sensor_data.gyro_sample_rate > 0 ? sensor_data.gyro_sample_rate : 20000;
    algo_conf->mag_odr_us = sensor_data.compass_sample_rate > 0 ? sensor_data.compass_sample_rate : 20000;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: fusion sample rate: accel_rate=%d, gyro_rate=%d, compass_rate=%d",
            algo_conf->acc_odr_us, algo_conf->gyr_odr_us, algo_conf->mag_odr_us);

    invn_algo_agm_set_config(&sensor_data.algo, algo_conf);
}

static void inv_get_rotation(float r[3][3])
{
    int rot[9];

    inv_quaternion_to_rotation(sensor_data.nine_axis_data, rot);
    r[0][0] = rot[0] * INV_TWO_POWER_NEG_30;
    r[0][1] = rot[1] * INV_TWO_POWER_NEG_30;
    r[0][2] = rot[2] * INV_TWO_POWER_NEG_30;
    r[1][0] = rot[3] * INV_TWO_POWER_NEG_30;
    r[1][1] = rot[4] * INV_TWO_POWER_NEG_30;
    r[1][2] = rot[5] * INV_TWO_POWER_NEG_30;
    r[2][0] = rot[6] * INV_TWO_POWER_NEG_30;
    r[2][1] = rot[7] * INV_TWO_POWER_NEG_30;
    r[2][2] = rot[8] * INV_TWO_POWER_NEG_30;
}

static void google_orientation(float *g)
{
    const float rad2deg = (float)(180.0 / M_PI);
    float R[3][3];

    inv_get_rotation(R);

    g[0] = atan2f(-R[1][0], R[0][0]) * rad2deg;
    g[1] = atan2f(-R[2][1], R[2][2]) * rad2deg;
    g[2] = asinf ( R[2][0])          * rad2deg;
    if (g[0] < 0)
        g[0] += 360;
}

void mpl_init_cal_lib()
{
    int ret;

    LOGI("Algorithm fusion_AGM version %s", invn_algo_agm_version());

    ret = inv_init_lib();
    LOGE_IF(ret != 0, "MPL Builder: algo init error %d", ret);

    sensor_data.gyro_bias_updated = 0;
    sensor_data.accel_bias_updated = 0;
}

int mpl_gyro_reset_timestamp(void)
{
    sensor_data.gyro_timestamp = 0;
    sensor_data.quat9_timestamp = 0;
    sensor_data.or_timestamp = 0;
    sensor_data.quat6_timestamp = 0;

    return 0;
}

int mpl_accel_reset_timestamp(void)
{
    sensor_data.accel_timestamp = 0;
    sensor_data.la_timestamp = 0;
    sensor_data.gravity_timestamp = 0;
    sensor_data.quat6_compass_timestamp = 0;

    return 0;
}

int mpl_compass_reset_timestamp(void)
{
    sensor_data.compass_timestamp = 0;

    return 0;
}

static void mpl_log(MPLLogger::sensor_id sensor_type, const InvnAlgoAGMInput *input, const InvnAlgoAGMOutput *output)
{
    int32_t idata[1];
    float fdata[6];
    int64_t timestamp;

    if (input->mask & INVN_ALGO_AGM_INPUT_MASK_MAG) {
        timestamp = input->sRmag_time_us;
    } else {
        timestamp = input->sRimu_time_us;
    }

    switch (sensor_type) {
    case MPLLogger::SENSOR_RAW_ACCELEROMETER:
        sensor_data.MPLLog.logEvents(sensor_type, input->sRacc_data, timestamp);
        break;
    case MPLLogger::SENSOR_RAW_GYROSCOPE:
        sensor_data.MPLLog.logEvents(sensor_type, input->sRgyr_data, timestamp);
        break;
    case MPLLogger::SENSOR_RAW_MAGNETOMETER:
        sensor_data.MPLLog.logEvents(sensor_type, input->sRmag_data, timestamp);
        break;
    case MPLLogger::SENSOR_RAW_TEMPERATURE:
        idata[0] = input->sRtemp_data;
        sensor_data.MPLLog.logEvents(sensor_type, idata, timestamp);
        break;
    case MPLLogger::SENSOR_ACCELEROMETER:
        fdata[0] = (float)output->acc_cal_q16[0] * CONVERSION_Q16;
        fdata[1] = (float)output->acc_cal_q16[1] * CONVERSION_Q16;
        fdata[2] = (float)output->acc_cal_q16[2] * CONVERSION_Q16;
        sensor_data.MPLLog.logEvents(sensor_type, fdata, output->acc_accuracy_flag, timestamp);
        break;
    case MPLLogger::SENSOR_GYROSCOPE:
        fdata[0] = (float)output->gyr_cal_q16[0] * CONVERSION_Q16;
        fdata[1] = (float)output->gyr_cal_q16[1] * CONVERSION_Q16;
        fdata[2] = (float)output->gyr_cal_q16[2] * CONVERSION_Q16;
        sensor_data.MPLLog.logEvents(sensor_type, fdata, output->gyr_accuracy_flag, timestamp);
        break;
    case MPLLogger::SENSOR_MAGNETOMETER:
        fdata[0] = (float)output->mag_cal_q16[0] * CONVERSION_Q16;
        fdata[1] = (float)output->mag_cal_q16[1] * CONVERSION_Q16;
        fdata[2] = (float)output->mag_cal_q16[2] * CONVERSION_Q16;
        sensor_data.MPLLog.logEvents(sensor_type, fdata, output->mag_accuracy_flag, timestamp);
        break;
    case MPLLogger::SENSOR_UNCAL_GYROSCOPE:
        fdata[0] = (float)output->gyr_uncal_q16[0] * CONVERSION_Q16;
        fdata[1] = (float)output->gyr_uncal_q16[1] * CONVERSION_Q16;
        fdata[2] = (float)output->gyr_uncal_q16[2] * CONVERSION_Q16;
        fdata[3] = (float)output->gyr_bias_q16[0] * CONVERSION_Q16 * 256;
        fdata[4] = (float)output->gyr_bias_q16[1] * CONVERSION_Q16 * 256;
        fdata[5] = (float)output->gyr_bias_q16[2] * CONVERSION_Q16 * 256;
        sensor_data.MPLLog.logEvents(sensor_type, fdata, output->gyr_accuracy_flag, timestamp);
        break;
    case MPLLogger::SENSOR_UNCAL_MAGNETOMETER:
        fdata[0] = (float)output->mag_uncal_q16[0] * CONVERSION_Q16;
        fdata[1] = (float)output->mag_uncal_q16[1] * CONVERSION_Q16;
        fdata[2] = (float)output->mag_uncal_q16[2] * CONVERSION_Q16;
        fdata[3] = (float)output->mag_bias_q16[0] * CONVERSION_Q16;
        fdata[4] = (float)output->mag_bias_q16[1] * CONVERSION_Q16;
        fdata[5] = (float)output->mag_bias_q16[2] * CONVERSION_Q16;
        sensor_data.MPLLog.logEvents(sensor_type, fdata, output->mag_accuracy_flag, timestamp);
        break;
    case MPLLogger::SENSOR_GAME_ROTATION_VECTOR:
        fdata[0] = (float)output->grv_quat_q30[0] * INV_TWO_POWER_NEG_30;
        fdata[1] = (float)output->grv_quat_q30[1] * INV_TWO_POWER_NEG_30;
        fdata[2] = (float)output->grv_quat_q30[2] * INV_TWO_POWER_NEG_30;
        fdata[3] = (float)output->grv_quat_q30[3] * INV_TWO_POWER_NEG_30;
        sensor_data.MPLLog.logEvents(sensor_type, fdata, 0, timestamp);
        break;
    case MPLLogger::SENSOR_ROTATION_VECTOR:
        fdata[0] = (float)output->rv_quat_q30[0] * INV_TWO_POWER_NEG_30;
        fdata[1] = (float)output->rv_quat_q30[1] * INV_TWO_POWER_NEG_30;
        fdata[2] = (float)output->rv_quat_q30[2] * INV_TWO_POWER_NEG_30;
        fdata[3] = (float)output->rv_quat_q30[3] * INV_TWO_POWER_NEG_30;
        fdata[4] = (float)output->rv_accuracy_3sigma_q27 / (float)(1 << 27);
        sensor_data.MPLLog.logEvents(sensor_type, fdata, 0, timestamp);
        break;
    case MPLLogger::SENSOR_GRAVITY:
        fdata[0] = (float)output->gravity_q16[0] * CONVERSION_Q16;
        fdata[1] = (float)output->gravity_q16[1] * CONVERSION_Q16;
        fdata[2] = (float)output->gravity_q16[2] * CONVERSION_Q16;
        sensor_data.MPLLog.logEvents(sensor_type, fdata, 0, timestamp);
        break;
    case MPLLogger::SENSOR_LINEAR_ACCELERATION:
        fdata[0] = (float)output->linear_acc_q16[0] * CONVERSION_Q16;
        fdata[1] = (float)output->linear_acc_q16[1] * CONVERSION_Q16;
        fdata[2] = (float)output->linear_acc_q16[2] * CONVERSION_Q16;
        sensor_data.MPLLog.logEvents(sensor_type, fdata, 0, timestamp);
        break;
    case MPLLogger::SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
        fdata[0] = (float)output->gmrv_quat_q30[0] * INV_TWO_POWER_NEG_30;
        fdata[1] = (float)output->gmrv_quat_q30[1] * INV_TWO_POWER_NEG_30;
        fdata[2] = (float)output->gmrv_quat_q30[2] * INV_TWO_POWER_NEG_30;
        fdata[3] = (float)output->gmrv_quat_q30[3] * INV_TWO_POWER_NEG_30;
        fdata[4] = (float)output->gmrv_accuracy_3sigma_q27 / (float)(1 << 27);
        sensor_data.MPLLog.logEvents(sensor_type, fdata, 0, timestamp);
        break;
    default:
        break;
    }
}

static void mpl_build_quat(const InvnAlgoAGMOutput *output, int64_t timestamp)
{
    sensor_data.quat6_timestamp = timestamp;

    sensor_data.six_axis_data[0] = output->grv_quat_q30[0];
    sensor_data.six_axis_data[1] = output->grv_quat_q30[1];
    sensor_data.six_axis_data[2] = output->grv_quat_q30[2];
    sensor_data.six_axis_data[3] = output->grv_quat_q30[3];

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: grv out: (%d, %d, %d, %d)",
            sensor_data.six_axis_data[0], sensor_data.six_axis_data[1],
            sensor_data.six_axis_data[2], sensor_data.six_axis_data[3]);
}

static void mpl_build_quat9(const InvnAlgoAGMOutput *output, long long timestamp)
{
    sensor_data.quat9_timestamp = timestamp;
    sensor_data.or_timestamp = timestamp;

    sensor_data.nine_axis_data[0] = output->rv_quat_q30[0];
    sensor_data.nine_axis_data[1] = output->rv_quat_q30[1];
    sensor_data.nine_axis_data[2] = output->rv_quat_q30[2];
    sensor_data.nine_axis_data[3] = output->rv_quat_q30[3];
    sensor_data.agm_accuracy_rad = (float)output->rv_accuracy_3sigma_q27 / (float)(1 << 27);
    sensor_data.heading_accuracy_rad = (float)output->rv_accuracy_q27 / (float)(1 << 27);

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: rv out: (%d, %d, %d, %d) - %f",
            sensor_data.nine_axis_data[0], sensor_data.nine_axis_data[1],
            sensor_data.nine_axis_data[2], sensor_data.nine_axis_data[3],
            sensor_data.agm_accuracy_rad);
}

static void mpl_build_quat_compass(const InvnAlgoAGMOutput *output, long long timestamp)
{
    sensor_data.quat6_compass_timestamp = timestamp;

    sensor_data.six_axis_compass_data[0] = output->gmrv_quat_q30[0];
    sensor_data.six_axis_compass_data[1] = output->gmrv_quat_q30[1];
    sensor_data.six_axis_compass_data[2] = output->gmrv_quat_q30[2];
    sensor_data.six_axis_compass_data[3] = output->gmrv_quat_q30[3];
    sensor_data.am_accuracy_rad = (float)output->gmrv_accuracy_3sigma_q27 / (float)(1 << 27);

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: gmrv out: (%d, %d, %d, %d) - %f",
            sensor_data.six_axis_compass_data[0], sensor_data.six_axis_compass_data[1],
            sensor_data.six_axis_compass_data[2], sensor_data.six_axis_compass_data[3],
            sensor_data.am_accuracy_rad);
}

static void mpl_build_grav(const InvnAlgoAGMOutput *output, long long timestamp)
{
    sensor_data.gravity_timestamp = timestamp;

    sensor_data.gravity[0] = output->gravity_q16[0];
    sensor_data.gravity[1] = output->gravity_q16[1];
    sensor_data.gravity[2] = output->gravity_q16[2];

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: grav out: (%d, %d, %d)",
            sensor_data.gravity[0], sensor_data.gravity[1], sensor_data.gravity[2]);
}

static void mpl_build_la(const InvnAlgoAGMOutput *output, long long timestamp)
{
    sensor_data.la_timestamp = timestamp;

    sensor_data.linear_acceleration[0] = output->linear_acc_q16[0];
    sensor_data.linear_acceleration[1] = output->linear_acc_q16[1];
    sensor_data.linear_acceleration[2] = output->linear_acc_q16[2];

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: la out: (%d, %d, %d)",
            sensor_data.linear_acceleration[0], sensor_data.linear_acceleration[1], sensor_data.linear_acceleration[2]);
}

int mpl_build_gyro(int *gyro, int temperature, long long timestamp)
{
    InvnAlgoAGMInput input;
    InvnAlgoAGMOutput output;
    int gyro_raw_data[3];

    if (timestamp <= sensor_data.gyro_timestamp) {
        return 0;
    }

    memset(&input, 0, sizeof(input));
    inv_convert_to_body(sensor_data.gyro_matrix_scalar, gyro, gyro_raw_data);

    sensor_data.gyro_timestamp = timestamp;
    sensor_data.gyro_raw_data[0] = (short)(gyro_raw_data[0]);
    sensor_data.gyro_raw_data[1] = (short)(gyro_raw_data[1]);
    sensor_data.gyro_raw_data[2] = (short)(gyro_raw_data[2]);
    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: gyro in: (%d, %d, %d)",
            sensor_data.gyro_raw_data[0], sensor_data.gyro_raw_data[1], sensor_data.gyro_raw_data[2]);

    // fill gyro inputs and call algo process
    input.mask = INVN_ALGO_AGM_INPUT_MASK_GYR;
    input.sRimu_time_us = timestamp / 1000LL;
    input.sRgyr_data[0] = sensor_data.gyro_raw_data[0] << 4;
    input.sRgyr_data[1] = sensor_data.gyro_raw_data[1] << 4;
    input.sRgyr_data[2] = sensor_data.gyro_raw_data[2] << 4;
    input.sRtemp_data = temperature;
    invn_algo_agm_process(&sensor_data.algo, &input, &output);
    if (!(output.mask & INVN_ALGO_AGM_OUTPUT_MASK_GYRO_CAL)) {
        LOGE("MPL Builder: algo processing error");
        return -1;
    }

    // data scaled in dps on Q16
    sensor_data.gyro_data[0] = output.gyr_cal_q16[0];
    sensor_data.gyro_data[1] = output.gyr_cal_q16[1];
    sensor_data.gyro_data[2] = output.gyr_cal_q16[2];
    sensor_data.gyro_bias[0] = output.gyr_bias_q16[0];
    sensor_data.gyro_bias[1] = output.gyr_bias_q16[1];
    sensor_data.gyro_bias[2] = output.gyr_bias_q16[2];
    sensor_data.gyro_accuracy = output.gyr_accuracy_flag;
    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: cal_gyro out: (%d, %d, %d) - %d",
            sensor_data.gyro_data[0], sensor_data.gyro_data[1], sensor_data.gyro_data[2], sensor_data.gyro_accuracy);

    if (sensor_data.gyro_accuracy > 1)
        sensor_data.gyro_bias_updated = 1; // calibrated after setting bias

    mpl_log(MPLLogger::SENSOR_RAW_GYROSCOPE, &input, &output);
    mpl_log(MPLLogger::SENSOR_RAW_TEMPERATURE, &input, &output);
    mpl_log(MPLLogger::SENSOR_GYROSCOPE, &input, &output);
    mpl_log(MPLLogger::SENSOR_UNCAL_GYROSCOPE, &input, &output);

    if (output.mask & INVN_ALGO_AGM_OUTPUT_MASK_QUAT_AG) {
        mpl_build_quat(&output, timestamp);
        mpl_log(MPLLogger::SENSOR_GAME_ROTATION_VECTOR, &input, &output);
    }
    if (output.mask & INVN_ALGO_AGM_OUTPUT_MASK_QUAT_AGM) {
        mpl_build_quat9(&output, timestamp);
        mpl_log(MPLLogger::SENSOR_ROTATION_VECTOR, &input, &output);
    }
    if (output.mask & INVN_ALGO_AGM_OUTPUT_MASK_GRAVITY) {
        mpl_build_grav(&output, timestamp);
        mpl_log(MPLLogger::SENSOR_GRAVITY, &input, &output);
    }
    if (output.mask & INVN_ALGO_AGM_OUTPUT_MASK_LINEARACC) {
        mpl_build_la(&output, timestamp);
        mpl_log(MPLLogger::SENSOR_LINEAR_ACCELERATION, &input, &output);
    }

    return 0;
}

int mpl_build_eis_gyro(int *gyro, int temperature, long long timestamp)
{
    if (timestamp > sensor_data.gyro_timestamp) {
        mpl_build_gyro(gyro, temperature, timestamp);
    }

    sensor_data.eis_gyro_timestamp = sensor_data.gyro_timestamp;
    sensor_data.eis_calibrated[0] = sensor_data.gyro_data[0];
    sensor_data.eis_calibrated[1] = sensor_data.gyro_data[1];
    sensor_data.eis_calibrated[2] = sensor_data.gyro_data[2];
    sensor_data.eis_calibrated[3] = gyro[3];
    sensor_data.eis_gyro_bias[0] = sensor_data.gyro_bias[0];
    sensor_data.eis_gyro_bias[1] = sensor_data.gyro_bias[1];
    sensor_data.eis_gyro_bias[2] = sensor_data.gyro_bias[2];

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: eis_gyro out: (%d, %d, %d, %d)",
            sensor_data.eis_calibrated[0], sensor_data.eis_calibrated[1],
            sensor_data.eis_calibrated[2], sensor_data.eis_calibrated[2]);

    return 0;
}

int mpl_build_accel(int *accel, int temperature, long long timestamp)
{
    InvnAlgoAGMInput input;
    InvnAlgoAGMOutput output;
    int accel_raw_data[3];

    if (timestamp <= sensor_data.accel_timestamp) {
        return 0;
    }

    memset(&input, 0, sizeof(input));
    inv_convert_to_body(sensor_data.accel_matrix_scalar, accel, accel_raw_data);

    sensor_data.accel_timestamp = timestamp;
    sensor_data.accel_raw_data[0] = (short)(accel_raw_data[0]);
    sensor_data.accel_raw_data[1] = (short)(accel_raw_data[1]);
    sensor_data.accel_raw_data[2] = (short)(accel_raw_data[2]);
    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: accel in: (%d, %d, %d)",
            sensor_data.accel_raw_data[0], sensor_data.accel_raw_data[1], sensor_data.accel_raw_data[2]);

    // fill accel inputs and call algo process
    input.mask = INVN_ALGO_AGM_INPUT_MASK_ACC;
    input.sRimu_time_us = timestamp / 1000LL;
    input.sRacc_data[0] = sensor_data.accel_raw_data[0] << 4;
    input.sRacc_data[1] = sensor_data.accel_raw_data[1] << 4;
    input.sRacc_data[2] = sensor_data.accel_raw_data[2] << 4;
    input.sRtemp_data = temperature;
    invn_algo_agm_process(&sensor_data.algo, &input, &output);
    if (!(output.mask & INVN_ALGO_AGM_OUTPUT_MASK_ACCEL_CAL)) {
        LOGE("MPL Builder: algo processing error");
        return -1;
    }

    // data scaled in g on Q16
    sensor_data.accel_data[0] = output.acc_cal_q16[0];
    sensor_data.accel_data[1] = output.acc_cal_q16[1];
    sensor_data.accel_data[2] = output.acc_cal_q16[2];
    sensor_data.accel_bias[0] = output.acc_bias_q16[0];
    sensor_data.accel_bias[1] = output.acc_bias_q16[1];
    sensor_data.accel_bias[2] = output.acc_bias_q16[2];
    sensor_data.accel_accuracy = output.acc_accuracy_flag;
    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: cal_accel out: (%d, %d, %d) - %d",
            sensor_data.accel_data[0], sensor_data.accel_data[1], sensor_data.accel_data[2],
            sensor_data.accel_accuracy);

    if (sensor_data.accel_accuracy > 1)
        sensor_data.accel_bias_updated = 1; // calibrated after setting bias

    mpl_log(MPLLogger::SENSOR_RAW_ACCELEROMETER, &input, &output);
    mpl_log(MPLLogger::SENSOR_RAW_TEMPERATURE, &input, &output);
    mpl_log(MPLLogger::SENSOR_ACCELEROMETER, &input, &output);

    if (output.mask & INVN_ALGO_AGM_OUTPUT_MASK_QUAT_AG) {
        mpl_build_quat(&output, timestamp);
        mpl_log(MPLLogger::SENSOR_GAME_ROTATION_VECTOR, &input, &output);
    }
    if (output.mask & INVN_ALGO_AGM_OUTPUT_MASK_QUAT_AM) {
        mpl_build_quat_compass(&output, timestamp);
        mpl_log(MPLLogger::SENSOR_GEOMAGNETIC_ROTATION_VECTOR, &input, &output);
    }
    if (output.mask & INVN_ALGO_AGM_OUTPUT_MASK_QUAT_AGM) {
        mpl_build_quat9(&output, timestamp);
        mpl_log(MPLLogger::SENSOR_ROTATION_VECTOR, &input, &output);
    }
    if (output.mask & INVN_ALGO_AGM_OUTPUT_MASK_GRAVITY) {
        mpl_build_grav(&output, timestamp);
        mpl_log(MPLLogger::SENSOR_GRAVITY, &input, &output);
    }
    if (output.mask & INVN_ALGO_AGM_OUTPUT_MASK_LINEARACC) {
        mpl_build_la(&output, timestamp);
        mpl_log(MPLLogger::SENSOR_LINEAR_ACCELERATION, &input, &output);
    }

    return 0;
}

int mpl_build_compass(int *compass, long long timestamp)
{
    InvnAlgoAGMInput input;
    InvnAlgoAGMOutput output;
    int compass_raw_data[3];

    if (timestamp <= sensor_data.compass_timestamp) {
        return 0;
    }

    memset(&input, 0, sizeof(input));
    inv_convert_to_body(sensor_data.compass_matrix_scalar, compass, compass_raw_data);

    sensor_data.compass_timestamp = timestamp;
    sensor_data.compass_raw_data[0] = (short)(compass_raw_data[0]);
    sensor_data.compass_raw_data[1] = (short)(compass_raw_data[1]);
    sensor_data.compass_raw_data[2] = (short)(compass_raw_data[2]);
    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: compass in: (%d, %d, %d)",
            compass_raw_data[0], compass_raw_data[1], compass_raw_data[2]);

    // fill mag inputs and call algo process
    input.mask = INVN_ALGO_AGM_INPUT_MASK_MAG;
    input.sRmag_time_us = timestamp / 1000LL;
    input.sRmag_data[0] = compass_raw_data[0];
    input.sRmag_data[1] = compass_raw_data[1];
    input.sRmag_data[2] = compass_raw_data[2];
    invn_algo_agm_process(&sensor_data.algo, &input, &output);
    if (!(output.mask & INVN_ALGO_AGM_OUTPUT_MASK_MAG_CAL)) {
        LOGE("MPL Builder: algo processing error");
        return -1;
    }

    // data scaled in uT on Q16
    sensor_data.compass_data[0] = output.mag_cal_q16[0];
    sensor_data.compass_data[1] = output.mag_cal_q16[1];
    sensor_data.compass_data[2] = output.mag_cal_q16[2];
    sensor_data.compass_bias[0] = output.mag_bias_q16[0];
    sensor_data.compass_bias[1] = output.mag_bias_q16[1];
    sensor_data.compass_bias[2] = output.mag_bias_q16[2];
    sensor_data.compass_accuracy = output.mag_accuracy_flag;
    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: cal_compass out: (%d, %d, %d) - %d",
            sensor_data.compass_data[0], sensor_data.compass_data[1], sensor_data.compass_data[2],
            sensor_data.compass_accuracy);

    mpl_log(MPLLogger::SENSOR_RAW_MAGNETOMETER, &input, &output);
    mpl_log(MPLLogger::SENSOR_MAGNETOMETER, &input, &output);
    mpl_log(MPLLogger::SENSOR_UNCAL_MAGNETOMETER, &input, &output);

    if (output.mask & INVN_ALGO_AGM_OUTPUT_MASK_QUAT_AM) {
        mpl_build_quat_compass(&output, timestamp);
        mpl_log(MPLLogger::SENSOR_GEOMAGNETIC_ROTATION_VECTOR, &input, &output);
    }
    if (output.mask & INVN_ALGO_AGM_OUTPUT_MASK_QUAT_AGM) {
        mpl_build_quat9(&output, timestamp);
        mpl_log(MPLLogger::SENSOR_ROTATION_VECTOR, &input, &output);
    }

    return 0;
}

int mpl_get_sensor_type_magnetic_field(float *values, int8_t *accuracy, int64_t *timestamp)
{
    values[0] = (float)sensor_data.compass_data[0] * COMPASS_CONVERSION;
    values[1] = (float)sensor_data.compass_data[1] * COMPASS_CONVERSION;
    values[2] = (float)sensor_data.compass_data[2] * COMPASS_CONVERSION;
    *accuracy = sensor_data.compass_accuracy;
    *timestamp = sensor_data.compass_timestamp;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: cal_compass get: (%f, %f, %f) - %d - %lld",
            values[0], values[1], values[2], *accuracy, (long long)(*timestamp));

    return 1;
}

int mpl_get_sensor_type_magnetic_field_raw(float *values, int8_t *accuracy, int64_t *timestamp)
{
    values[3] = (float)sensor_data.compass_bias[0] * COMPASS_CONVERSION;
    values[4] = (float)sensor_data.compass_bias[1] * COMPASS_CONVERSION;
    values[5] = (float)sensor_data.compass_bias[2] * COMPASS_CONVERSION;
    values[0] = ((float)sensor_data.compass_data[0] * COMPASS_CONVERSION) + values[3];
    values[1] = ((float)sensor_data.compass_data[1] * COMPASS_CONVERSION) + values[4];
    values[2] = ((float)sensor_data.compass_data[2] * COMPASS_CONVERSION) + values[5];
    *accuracy = 0;
    *timestamp = sensor_data.compass_timestamp;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: uncal_compass get: (%f, %f, %f) - (%f, %f, %f) - %d - %lld",
            values[0], values[1], values[2], values[3], values[4], values[5], *accuracy, (long long)(*timestamp));

    return 1;
}

int mpl_get_sensor_type_accelerometer(float *values, int8_t *accuracy, int64_t *timestamp)
{
    values[0] = (float)sensor_data.accel_data[0] * ACCEL_CONVERSION;
    values[1] = (float)sensor_data.accel_data[1] * ACCEL_CONVERSION;
    values[2] = (float)sensor_data.accel_data[2] * ACCEL_CONVERSION;
    *accuracy = sensor_data.accel_accuracy;
    *timestamp = sensor_data.accel_timestamp;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: cal_accel get: (%f, %f, %f) - %d - %lld",
            values[0], values[1], values[2], *accuracy, (long long)(*timestamp));

    return 1;
}

int mpl_get_sensor_type_accelerometer_raw(float *values, int8_t *accuracy, int64_t *timestamp)
{
    values[3] = (float)sensor_data.accel_bias[0] * ACCEL_CONVERSION;
    values[4] = (float)sensor_data.accel_bias[1] * ACCEL_CONVERSION;
    values[5] = (float)sensor_data.accel_bias[2] * ACCEL_CONVERSION;
    values[0] = ((float)sensor_data.accel_data[0] * ACCEL_CONVERSION) + values[3];
    values[1] = ((float)sensor_data.accel_data[1] * ACCEL_CONVERSION) + values[4];
    values[2] = ((float)sensor_data.accel_data[2] * ACCEL_CONVERSION) + values[5];
    *accuracy = 0;
    *timestamp = sensor_data.accel_timestamp;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: uncal_accel get: (%f, %f, %f) - (%f, %f, %f) - %d - %lld",
            values[0], values[1], values[2], values[3], values[4], values[5], *accuracy, (long long)(*timestamp));

    return 1;
}

int mpl_get_sensor_type_gyroscope(float *values, int8_t *accuracy, int64_t *timestamp)
{
    values[0] = (float)sensor_data.gyro_data[0] * GYRO_CONVERSION;
    values[1] = (float)sensor_data.gyro_data[1] * GYRO_CONVERSION;
    values[2] = (float)sensor_data.gyro_data[2] * GYRO_CONVERSION;
    *accuracy = sensor_data.gyro_accuracy;
    *timestamp = sensor_data.gyro_timestamp;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: cal_gyro get: (%f, %f, %f) - %d - %lld",
            values[0], values[1], values[2], *accuracy, (long long)(*timestamp));

    return 1;
}

int mpl_get_sensor_type_eis_gyroscope(float *values, int8_t *accuracy, int64_t *timestamp)
{
    values[0] = (float)sensor_data.eis_calibrated[0] * GYRO_CONVERSION;
    values[1] = (float)sensor_data.eis_calibrated[1] * GYRO_CONVERSION;
    values[2] = (float)sensor_data.eis_calibrated[2] * GYRO_CONVERSION;
    values[3] = sensor_data.eis_calibrated[3];
    *accuracy = sensor_data.gyro_accuracy;
    *timestamp = sensor_data.eis_gyro_timestamp;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: cal_eis_gyro get: (%f, %f, %f) - %d - %lld",
            values[0], values[1], values[2], *accuracy, (long long)(*timestamp));

    return 1;
}

int mpl_get_sensor_type_gyroscope_raw(float *values, int8_t *accuracy, int64_t *timestamp)
{
    values[3] = (float)sensor_data.gyro_bias[0] * GYRO_CONVERSION;
    values[4] = (float)sensor_data.gyro_bias[1] * GYRO_CONVERSION;
    values[5] = (float)sensor_data.gyro_bias[2] * GYRO_CONVERSION;
    values[0] = ((float)sensor_data.gyro_data[0] * GYRO_CONVERSION) + values[3];
    values[1] = ((float)sensor_data.gyro_data[1] * GYRO_CONVERSION) + values[4];
    values[2] = ((float)sensor_data.gyro_data[2] * GYRO_CONVERSION) + values[5];
    *accuracy = 0;
    *timestamp = sensor_data.gyro_timestamp;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: uncal_gyro get: (%f, %f, %f) - (%f, %f, %f) - %d - %lld",
            values[0], values[1], values[2], values[3], values[4], values[5], *accuracy, (long long)(*timestamp));

    return 1;
}

int mpl_get_sensor_type_rotation_vector(float *values, int8_t *accuracy, int64_t *timestamp)
{
    if (sensor_data.nine_axis_data[0] >= 0){
        values[0] = (float)sensor_data.nine_axis_data[1] * INV_TWO_POWER_NEG_30;
        values[1] = (float)sensor_data.nine_axis_data[2] * INV_TWO_POWER_NEG_30;
        values[2] = (float)sensor_data.nine_axis_data[3] * INV_TWO_POWER_NEG_30;
        values[3] = (float)sensor_data.nine_axis_data[0] * INV_TWO_POWER_NEG_30;
    } else {
        values[0] = -(float)sensor_data.nine_axis_data[1] * INV_TWO_POWER_NEG_30;
        values[1] = -(float)sensor_data.nine_axis_data[2] * INV_TWO_POWER_NEG_30;
        values[2] = -(float)sensor_data.nine_axis_data[3] * INV_TWO_POWER_NEG_30;
        values[3] = -(float)sensor_data.nine_axis_data[0] * INV_TWO_POWER_NEG_30;
    }
    values[4] = sensor_data.agm_accuracy_rad;
    *accuracy = 0; // dummy
    *timestamp = sensor_data.quat9_timestamp;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: rv get: (%f, %f, %f, %f) - %f - %lld",
            values[0], values[1], values[2], values[3], values[4], (long long)(*timestamp));

    return 1;
}

int mpl_get_sensor_type_game_rotation_vector(float *values, int8_t *accuracy, int64_t *timestamp)
{
    values[0] = (float)sensor_data.six_axis_data[1] * INV_TWO_POWER_NEG_30;
    values[1] = (float)sensor_data.six_axis_data[2] * INV_TWO_POWER_NEG_30;
    values[2] = (float)sensor_data.six_axis_data[3] * INV_TWO_POWER_NEG_30;
    values[3] = (float)sensor_data.six_axis_data[0] * INV_TWO_POWER_NEG_30;
    *accuracy = 0;
    *timestamp = sensor_data.quat6_timestamp;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: grv get: (%f, %f, %f, %f) - %lld",
            values[0], values[1], values[2], values[3], (long long)(*timestamp));

    return 1;
}

int mpl_get_sensor_type_geomagnetic_rotation_vector(float *values, int8_t *accuracy, int64_t *timestamp)
{
    values[0] = (float)sensor_data.six_axis_compass_data[1] * INV_TWO_POWER_NEG_30;
    values[1] = (float)sensor_data.six_axis_compass_data[2] * INV_TWO_POWER_NEG_30;
    values[2] = (float)sensor_data.six_axis_compass_data[3] * INV_TWO_POWER_NEG_30;
    values[3] = (float)sensor_data.six_axis_compass_data[0] * INV_TWO_POWER_NEG_30;
    values[4] = sensor_data.am_accuracy_rad;
    *accuracy = 0; // dummy
    *timestamp = sensor_data.quat6_compass_timestamp;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: gmrv get: (%f, %f, %f, %f) - %f - %lld",
            values[0], values[1], values[2], values[3], values[4], (long long)(*timestamp));

    return 1;
}

int mpl_get_sensor_type_linear_acceleration(float *values, int8_t *accuracy, int64_t *timestamp)
{
    values[0] = sensor_data.linear_acceleration[0] * ACCEL_CONVERSION;
    values[1] = sensor_data.linear_acceleration[1] * ACCEL_CONVERSION;
    values[2] = sensor_data.linear_acceleration[2] * ACCEL_CONVERSION;
    *accuracy = sensor_data.accel_accuracy;
    *timestamp = sensor_data.la_timestamp;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: la get: (%f, %f, %f) - %lld",
            values[0], values[1], values[2], (long long)(*timestamp));

    return 1;
}

int mpl_get_sensor_type_gravity(float *values, int8_t *accuracy, int64_t *timestamp)
{
    values[0] = sensor_data.gravity[0] * ACCEL_CONVERSION;
    values[1] = sensor_data.gravity[1] * ACCEL_CONVERSION;
    values[2] = sensor_data.gravity[2] * ACCEL_CONVERSION;
    *accuracy = sensor_data.accel_accuracy;
    *timestamp = sensor_data.gravity_timestamp;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: grav get: (%f, %f, %f) - %lld",
            values[0], values[1], values[2], (long long)(*timestamp));

    return 1;
}

int mpl_get_sensor_type_orientation(float *values, int8_t *accuracy, int64_t *timestamp)
{
    google_orientation(values);
    *accuracy = sensor_data.compass_accuracy;
    *timestamp = sensor_data.or_timestamp;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: orient get: (%f, %f, %f) - %lld",
            values[0], values[1], values[2], (long long)(*timestamp));

    return 1;
}

int mpl_get_sensor_type_heading(float *values, int8_t *accuracy, int64_t *timestamp)
{
    const float rad2deg = (float)(180.0 / M_PI);
    float orient[3];

    google_orientation(orient);
    values[0] = orient[0];
    values[1] = sensor_data.heading_accuracy_rad * rad2deg;
    *accuracy = 0;
    *timestamp = sensor_data.or_timestamp;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: heading get: %f - %f - %lld",
            values[0], values[1], (long long)(*timestamp));

    return 1;
}

void mpl_set_sample_rate(int sample_rate_us, int id)
{
    switch (id) {
    case ID_GY:
        sensor_data.gyro_sample_rate = sample_rate_us;
        inv_set_sample_rate();
        break;
    case ID_A:
        sensor_data.accel_sample_rate = sample_rate_us;
        inv_set_sample_rate();
        break;
    case ID_M:
        sensor_data.compass_sample_rate = sample_rate_us;
        inv_set_sample_rate();
        break;
    default:
        break;
    }
}

void mpl_set_accel_orientation_and_scale(int orientation, int scale)
{
    sensor_data.accel_matrix_scalar = orientation;
    sensor_data.accel_scale = scale;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: set accel matrix: %d  scale: %d",
                    orientation, scale);
}

void mpl_set_gyro_orientation_and_scale(int orientation, int scale)
{
    sensor_data.gyro_matrix_scalar = orientation;
    sensor_data.gyro_scale = scale;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: set gyro matrix: %d  scale: %d",
                    orientation, scale);
}

void mpl_set_compass_orientation_and_scale(int orientation, int scale, int *sensitivity, int *softIron)
{
    sensor_data.compass_matrix_scalar = orientation;
    sensor_data.compass_scale = scale;

    for (int i=0; i<3; i++)
        sensor_data.compass_sens[i] = sensitivity[i];

    for (int i=0; i<9; i++)
        sensor_data.compass_soft_iron[i] = softIron[i];

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: set compass soft iron: %d %d %d", sensor_data.compass_soft_iron[0],
                    sensor_data.compass_soft_iron[4], sensor_data.compass_soft_iron[8]);
    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: set compass matrix: %d  scale: %d sens_x: %d sens_y: %d sens_z: %d ",
                    orientation, scale, sensitivity[0], sensitivity[1], sensitivity[2]);
}

int inv_get_cal_accel(int *data)
{
    int accel[3];

    if (data == NULL)
        return -1;

    // lsb in chip frame
    inv_convert_to_chip(sensor_data.accel_matrix_scalar, sensor_data.accel_data, accel);
    data[0] = accel[0] / (2 * sensor_data.accel_scale);
    data[1] = accel[1] / (2 * sensor_data.accel_scale);
    data[2] = accel[2] / (2 * sensor_data.accel_scale);

    return 0;
}

int mpl_get_mpl_gyro_bias(int *bias, int *accuracy, int *updated)
{
    int algo_bias[3];

    // convert to lsb Q16 in chip frame
    inv_convert_to_chip(sensor_data.gyro_matrix_scalar, sensor_data.gyro_bias, algo_bias);
    bias[0] = ((int64_t)algo_bias[0] << 15) / sensor_data.gyro_scale;
    bias[1] = ((int64_t)algo_bias[1] << 15) / sensor_data.gyro_scale;
    bias[2] = ((int64_t)algo_bias[2] << 15) / sensor_data.gyro_scale;
    *accuracy = sensor_data.gyro_accuracy;

    *updated = sensor_data.gyro_bias_updated;
    sensor_data.gyro_bias_updated = 0;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: get gyro bias %d %d %d accuracy=%d", bias[0], bias[1], bias[2], *accuracy);
    return 1;
}

int mpl_set_mpl_gyro_bias(int *bias, int accuracy)
{
    int algo_bias[3];
    int ret;

    // convert from lsb Q16 in chip frame
    inv_convert_to_body(sensor_data.gyro_matrix_scalar, bias, algo_bias);
    sensor_data.gyro_bias[0] = ((int64_t)algo_bias[0] * (int64_t)sensor_data.gyro_scale) >> 15;
    sensor_data.gyro_bias[1] = ((int64_t)algo_bias[1] * (int64_t)sensor_data.gyro_scale) >> 15;
    sensor_data.gyro_bias[2] = ((int64_t)algo_bias[2] * (int64_t)sensor_data.gyro_scale) >> 15;
    sensor_data.gyro_accuracy = accuracy;

    ret = inv_init_lib();
    LOGE_IF(ret != 0, "MPL Builder: algo init error %d", ret);

    sensor_data.gyro_bias_updated = 0; // will be 1 when calibration is done by algo
    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: set gyro bias %d %d %d accuracy=%d", bias[0], bias[1], bias[2], accuracy);
    return 1;
}

int mpl_get_mpl_accel_bias(int *bias, int *accuracy, int *updated)
{
     int algo_bias[3];

    // convert to lsb Q16 in chip frame
    inv_convert_to_chip(sensor_data.accel_matrix_scalar, sensor_data.accel_bias, algo_bias);
    bias[0] = ((int64_t)algo_bias[0] << 15) / sensor_data.accel_scale;
    bias[1] = ((int64_t)algo_bias[1] << 15) / sensor_data.accel_scale;
    bias[2] = ((int64_t)algo_bias[2] << 15) / sensor_data.accel_scale;
    *accuracy = sensor_data.accel_accuracy;

    *updated = sensor_data.accel_bias_updated;
    sensor_data.accel_bias_updated = 0;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: get accel bias %d %d %d accuracy=%d", bias[0], bias[1], bias[2], *accuracy);
    return 1;
}

int mpl_set_mpl_accel_bias(int *bias, int accuracy)
{
    int algo_bias[3];
    int ret;

    // convert from lsb Q16 in chip frame
    inv_convert_to_body(sensor_data.accel_matrix_scalar, bias, algo_bias);
    sensor_data.accel_bias[0] = ((int64_t)algo_bias[0] * (int64_t)sensor_data.accel_scale) >> 15;
    sensor_data.accel_bias[1] = ((int64_t)algo_bias[1] * (int64_t)sensor_data.accel_scale) >> 15;
    sensor_data.accel_bias[2] = ((int64_t)algo_bias[2] * (int64_t)sensor_data.accel_scale) >> 15;
    sensor_data.accel_accuracy = accuracy;

    ret = inv_init_lib();
    LOGE_IF(ret != 0, "MPL Builder: algo init error %d", ret);

    sensor_data.accel_bias_updated = 0; // will be 1 when calibration is done by algo
    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: set accel bias %d %d %d accuracy=%d", bias[0], bias[1], bias[2], accuracy);
    return 1;
}

int mpl_get_mpl_compass_bias(int *bias)
{
    int algo_bias[3];

    // convert to lsb Q16 in chip frame
    inv_convert_to_chip(sensor_data.compass_matrix_scalar, sensor_data.compass_bias, algo_bias);
    bias[0] = ((int64_t)algo_bias[0] << 16) / sensor_data.compass_scale;
    bias[1] = ((int64_t)algo_bias[1] << 16) / sensor_data.compass_scale;
    bias[2] = ((int64_t)algo_bias[2] << 16) / sensor_data.compass_scale;

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: get compass bias %d %d %d", bias[0], bias[1], bias[2]);
    return 1;
}

int mpl_set_mpl_compass_bias(int *bias, int accuracy)
{
    int algo_bias[3];
    int ret;

    // convert from lsb Q16 in chip frame
    inv_convert_to_body(sensor_data.compass_matrix_scalar, bias, algo_bias);
    sensor_data.compass_bias[0] = ((int64_t)algo_bias[0] * (int64_t)sensor_data.compass_scale) >> 16;
    sensor_data.compass_bias[1] = ((int64_t)algo_bias[1] * (int64_t)sensor_data.compass_scale) >> 16;
    sensor_data.compass_bias[2] = ((int64_t)algo_bias[2] * (int64_t)sensor_data.compass_scale) >> 16;
    sensor_data.compass_accuracy = accuracy;

    ret = inv_init_lib();
    LOGE_IF(ret != 0, "MPL Builder: algo init error %d", ret);

    LOGD_IF(BUILDER_VERBOSE, "MPL Builder: set compass bias %d %d %d", bias[0], bias[1], bias[2]);
    return 1;
}

void mpl_set_lib_version(int version_number)
{
    (void)version_number;
}

int mpl_get_lib_version()
{
    return 0;
}
