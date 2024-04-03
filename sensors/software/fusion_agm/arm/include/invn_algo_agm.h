/*
$License:
	Copyright (C) 2018 InvenSense Corporation, All Rights Reserved.
$
*/
 

#ifndef _INVN_ALGO_AGM_H_
#define _INVN_ALGO_AGM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(INVN_API)
#define INVN_API
#endif

#define INVN_ALGO_AGM_DATA_STRUCTURE_SIZE 2150
#define ERROR_INVN_ALGO_AGM_STRUCT_NULL_POINTER 127

/** \defgroup AGM AGM
 *  \brief Algorithm that provides device orientation. 
 *         Inputs are raw accelerometer, gyroscope and optional magnetometer data. 
 *         Outputs are calibrated sensor and/or 9-axis sensor fusion. 
 *  \warning supported sampling frequency [50 Hz-1000 Hz]
 *  \warning supported gyroscope FSR [250 dps, 500 dps, 1000 dps, 2000 dps, 4000 dps]
 *  \warning supported accelerometer FSR [1 g, 2 g, 4 g, 8 g, 16 g, 32 g]
 */

#define INVN_ALGO_AGM_INPUT_MASK_ACC        (1 << 0)  ///< Raw Accel update mask
#define INVN_ALGO_AGM_INPUT_MASK_GYR        (1 << 1)  ///< Raw Gyro update mask
#define INVN_ALGO_AGM_INPUT_MASK_MAG        (1 << 2)  ///< Raw Mag update mask

#define INVN_ALGO_AGM_OUTPUT_MASK_ACCEL_CAL (1 << 0)  ///< Accel cal output update mask
#define INVN_ALGO_AGM_OUTPUT_MASK_GYRO_CAL  (1 << 1)  ///< Gyro cal output update mask
#define INVN_ALGO_AGM_OUTPUT_MASK_MAG_CAL   (1 << 2)  ///< Mag cal output update mask
#define INVN_ALGO_AGM_OUTPUT_MASK_QUAT_AG   (1 << 3)  ///< Game Rotation Vector (Accel and Gyro Fusion) output update mask
#define INVN_ALGO_AGM_OUTPUT_MASK_QUAT_AM   (1 << 4)  ///< Geomagnetic Rotation Vector (Accel and Mag Fusion) output update mask
#define INVN_ALGO_AGM_OUTPUT_MASK_QUAT_AGM  (1 << 5)  ///< Rotation Vector (Accel, Gyro and Mag Fusion) output update mask
#define INVN_ALGO_AGM_OUTPUT_MASK_GRAVITY   (1 << 6)  ///< Gravity vector output update mask
#define INVN_ALGO_AGM_OUTPUT_MASK_LINEARACC (1 << 7)  ///< Linear acceleration vector output update mask

 /*! \enum InvnAlgoAGM_ModeId_t
  * AGM enum to select mode ID \ingroup AGM
  */
	typedef enum {
		INVN_ALGO_AGM_MOBILE= 1,  /*!< Optimize for Mobile device. */
		INVN_ALGO_AGM_AUTOMOTIVE = 2,  /*!< Optimize for Automotive device. */
	} InvnAlgoAGM_ModeId_t;


/* Forward declarations */
struct inv_icm426xx;

/*! \struct InvnAGMStruct
 * Data structure holding internal algorithm state. 
 * Ensure data buffer is aligned to 32 bits for 32-bits MCU.
 * Other methods can be used to align memory using malloc or attribute((aligned, 4))
 */
typedef union InvnAlgoAGMStruct {
	uint8_t data[INVN_ALGO_AGM_DATA_STRUCTURE_SIZE];
	uint32_t data32;
}InvnAlgoAGMStruct;

/*! \struct InvnAlgoAGMInput
 * AGM input structure (raw data) \ingroup AGM
 */
typedef struct 
{	
	int64_t sRimu_time_us;  /*!<  Optional timestamp \f$ [\mu s]\f$ of raw accel and gyro.
								  Can be assigned to zero if timestamp is not available. */
	int64_t sRmag_time_us;  /*!<  Optional timestamp of raw mag.
								  Can be assigned to zero if timestamp is not available. */
	int32_t sRacc_data[3];  /*!<  Raw accelerometer in high resolution mode.
	                              Expect Full Scale Value coded on 20 bit (i.e. +/- FSR g = 1<<19 LSB) */
	int32_t sRgyr_data[3];  /*!<  Raw gyroscope in high resolution mode. 
	                              Expect Full Scale Value coded on 20 bit (i.e. +/- FSR dps = 1<<19 LSB) */
	int32_t sRmag_data[3];  /*!<  Raw magnetometer */
	int32_t mask;           /*!<  Bit field mask to specify one or more updated inputs. */
	int16_t sRtemp_data;    /*!<  Raw temperature */
} InvnAlgoAGMInput; 

	
/*! \struct InvnAlgoAGMOutput
 * AGM output structure (calibrated sensors and fusion output)  \ingroup AGM
 */
typedef struct 
{
	int32_t grv_quat_q30[4];    /*!< 6-axis (accel and gyro fusion) quaternion - warning use quaternion convention WXYZ */
	int32_t gmrv_quat_q30[4];   /*!< 6-axis (accel and magnetometer fusion) quaternion - warning use quaternion convention WXYZ */
	int32_t rv_quat_q30[4];     /*!< 9-axis (accel, gyro and magnetometer fusion) quaternion - warning use quaternion convention WXYZ */
	int32_t gravity_q16[3];     /*!< Gravity estimation in sensor frame */
	int32_t linear_acc_q16[3];  /*!< Linear acceleration estimation in sensor frame */

	int32_t acc_uncal_q16[3];   /*!< Uncalibrated accelerometer (1 g = 1<<16) */
	int32_t acc_cal_q16[3];     /*!< Calibrated accelerometer (1 g = 1<<16) */
	int32_t acc_bias_q16[3];    /*!< Accelerometer bias (1 g = 1<<16)*/

	int32_t gyr_uncal_q16[3];   /*!< Uncalibrated gyroscope (1 dps = 1<<16) */
	int32_t gyr_cal_q16[3];     /*!< Calibrated gyroscope (1 dps = 1<<16) */
	int32_t gyr_bias_q16[3];    /*!< Gyro bias (1 dps = 1<<16)*/

	int32_t mag_uncal_q16[3];   /*!< Uncalibrated magnetometer (1uT = 1<<16) */
	int32_t mag_cal_q16[3];     /*!< Calibrated magnetometer (1uT = 1<<16) */
	int32_t mag_bias_q16[3];    /*!< Magnetometer bias (1uT = 1<<16) */
	
	int32_t temp_degC_q16;				/*!< Temperature (1 \f$ [^{\circ}C]\f$ = 1<<16)*/
	int32_t gmrv_accuracy_3sigma_q27;	/*!< 6-axis (accel and magnetometer fusion) 3\sigma accuracy in rad */
	int32_t gmrv_accuracy_q27;			/*!< 6-axis (accel and magnetometer fusion) 1 sigma accuracy in rad (68 percent of population is within the accuracy)  */
	int32_t rv_accuracy_3sigma_q27;		/*!< 9-axis (accel, gyro and magnetometer fusion) 3\sigma accuracy in rad */
	int32_t rv_accuracy_q27;			/*!< 9-axis (accel, gyro and magnetometer fusion) 1 sigma accuracy in rad (68 percent of population is within the accuracy)  */
	int32_t mask;						/*!< Bit field mask to specify updated outputs */

	int8_t  acc_accuracy_flag;  /*!< Accelerometer accuracy from 0(non calibrated) to 3(well calibrated) */
	int8_t  gyr_accuracy_flag;  /*!< Gyro accuracy, from 0(non calibrated) to 3(well calibrated) */
	int8_t  mag_accuracy_flag;  /*!< Magnetometer accuracy, from 0(non calibrated) to 3(well calibrated) */

	int8_t  stationary;          /*!< Stationary detection based on gyro data.
									  Possible values 0: Motion, 1: Stationary, -1: Unknown */

} InvnAlgoAGMOutput; 


/*! \struct InvnAlgoAGMConfig
 * AGM configuration structure (sensor related settings) \ingroup AGM
 */
typedef struct 
{
	int32_t *acc_bias_q16;     /*!<  Previously stored accel bias pointer. 
	                                 If pointer is NULL or 0, offset will be set to { 0, 0, 0} */
	int32_t *gyr_bias_q16;     /*!<  Previously stored gyro bias pointer. 
	                                 If pointer is NULL or 0, offset will be set to { 0, 0, 0} */
	int32_t *mag_bias_q16;     /*!<  Mag_bias_q16 Previously stored mag bias pointer. 
	                                 If pointer is NULL or 0, offset will be set to { 0, 0, 0} */

	int32_t acc_fsr;           /*!<  Accelerometer full scale range [g] */
	int32_t gyr_fsr;           /*!<  Gyroscope full scale range [dps] */

	uint32_t acc_odr_us;       /*!<  Accelerometer output data rate in \f$ [\mu s]\f$ */
	uint32_t gyr_odr_us;       /*!<  Gyroscope output data rate \f$ [\mu s]\f$ */

	int32_t  mag_sc_q16;       /*!<  Magnetometer sensitivity 
	                                 In uT/LSB, e.g. mag_uT = (mag_sc_q16 * raw_mag_LSB)/65536 */
	uint32_t mag_odr_us;       /*!<  Magnetometer output data rate \f$ [\mu s]\f$ */

	int32_t temp_sensitivity;  /*!<  Temperature sensitivity in q30
	                                 if temperature(\f$ ^{\circ}C \f$) = LSB * k + z, then temp_sensitivity = k */
	int32_t temp_offset;       /*!<  Temperature offset in q16
	                                 if temperature(\f$ ^{\circ}C \f$) = LSB * k + z, then temp_offset = z */

	int8_t  acc_accuracy;      /*!<  Previously stored accelerometer bias accuracy (0 to 3) */
	int8_t  gyr_accuracy;      /*!<  Previously stored gyroscope bias accuracy (0 to 3)  */
	int8_t  mag_accuracy;      /*!<  Previously stored magnetometer bias accuracy (0 to 3)  */
	
	int32_t gyr_cal_stationary_duration_us;  /*!<  Duration for no motion window used for gyro bias calibration, in us, range [0, 6000000] */
	int32_t gyr_cal_sample_number_log2;      /*!<  Number of gyro sample collected for calibration, in sample number obtained after log2(x), range [6, 8] */
	int32_t gyr_cal_threshold_metric1;       /*!<  Stationary detection threshold of 1st metric for gyro calibration, no unit, range [0, 1200] */
	int32_t gyr_cal_threshold_metric2;       /*!<  Stationary detection threshold of 2nd metric for gyro calibration, no unit, range [0, 80000] */


} InvnAlgoAGMConfig; 


/*!
 * \brief Return library version x.y.z-suffix as a char array
 * \retval library version a char array "x.y.z-suffix"
 * \ingroup AGM
 */
INVN_API const char * invn_algo_agm_version(void);


/*!
 * \brief Generate default configuration.
 *        Some parameters such as sensor ODR and FSR can be changed to match sensor settings.
 * \config[out] algo init parameters structure.
 * \return initialization success indicator.
 * \retval 0 Success
 * \retval 1 Fail
 * \ingroup AGM
 */
INVN_API uint8_t invn_algo_agm_generate_config(InvnAlgoAGMConfig *config, InvnAlgoAGM_ModeId_t mode_id);

/*!
 * \brief Initializes algorithm with given parameter structure and reset states.
 * \self[in] memory allocated for algorithm internal state.
 * \config[in] algo init parameters structure.
 * \return initialization success indicator.
 * \retval 0 Success
 * \retval 1 Fail
 * \ingroup AGM
 */
INVN_API uint8_t invn_algo_agm_init(InvnAlgoAGMStruct *self, const InvnAlgoAGMConfig * config);

/*!
 * \brief Sets algo config structure.
 * \self[in] memory allocated for algorithm internal state.
 * \config[in] config structure of the algo.
  * \ingroup AGM
 */
INVN_API void invn_algo_agm_set_config(InvnAlgoAGMStruct *self, const InvnAlgoAGMConfig * config);


/*!
 * \brief Online, algo enable/disable sensor calibration (gyro, accel, mag).
 * \self[in] memory allocated for algorithm internal state.
 * \int[in] enable or disable gyro (0 disable, 1 enable).
 * \int[in] enable or disable accel calibration (0 disable, 1 enable).
 * \int[in] enable or disable mag calibration (0 disable, 1 enable).
 * \ingroup AGM
 */
INVN_API void invn_algo_agm_enable_calibration(InvnAlgoAGMStruct *self, const int8_t enable_gyrocal , const int8_t enable_accelcal, const int8_t enable_magcal);


/*!
 * \brief Performs algorithm computation.
 * \self[in] memory allocated for algorithm internal state.
 * \inputs[in] Algorithm input. 
 *             Input mask (inputs->mask) should be set with respect to new sensor data in InvnAlgoAGMInput.
 * \outputs[out] Algorithm output. 
 *               Output mask (outputs->mask) reports updated outputs in InvnAlgoAGMOutput.
 * \ingroup AGM
 */
INVN_API void invn_algo_agm_process(InvnAlgoAGMStruct *self, const InvnAlgoAGMInput *inputs, InvnAlgoAGMOutput *outputs);

#ifdef __cplusplus
}
#endif


#endif
