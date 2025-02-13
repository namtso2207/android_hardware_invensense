/*
 $License:
    Copyright (C) 2014-2018 InvenSense Corporation, All Rights Reserved.
 $
 */

/*******************************************************************************
 *
 * $Id: ml_stored_data.h 5873 2011-08-11 03:13:48Z mcaramello $
 *
 ******************************************************************************/

#ifndef INV_MPL_STORED_DATA_H
#define INV_MPL_STORED_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

/*
    Includes.
*/
#include "mltypes.h"

/*
    Defines
*/
#ifdef CAL_BIN_CUSTOM_FILE
#define MLCAL_FILE CAL_BIN_CUSTOM_FILE
#else
#  ifdef CAL_BIN_DATA_VENDOR_SENSOR
#  define MLCAL_FILE "/data/vendor/sensor/inv_cal_data.bin"
#  else
#  define MLCAL_FILE "/data/inv_cal_data.bin"
#  endif
#endif

/*
    APIs
*/
inv_error_t inv_load_calibration(void);
inv_error_t inv_store_calibration(void);

/*
    Internal APIs
*/
inv_error_t inv_read_cal(unsigned char **, size_t *);
inv_error_t inv_write_cal(unsigned char *cal, size_t len);
inv_error_t inv_load_cal_V0(unsigned char *calData, size_t len);
inv_error_t inv_load_cal_V1(unsigned char *calData, size_t len);

/*
    Other prototypes
*/
inv_error_t inv_load_cal(unsigned char *calData);
inv_error_t inv_store_cal(unsigned char *calData, size_t length);

#ifdef __cplusplus
}
#endif
#endif  /* INV_MPL_STORED_DATA_H */
