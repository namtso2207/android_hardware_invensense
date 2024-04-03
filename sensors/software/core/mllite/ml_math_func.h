/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
    See included License.txt for License information.
 $
 */
#ifndef INVENSENSE_INV_MATH_FUNC_H__
#define INVENSENSE_INV_MATH_FUNC_H__

#include "mltypes.h"

#define GYRO_MAG_SQR_SHIFT 6
#define NUM_ROTATION_MATRIX_ELEMENTS (9)
#define ROT_MATRIX_SCALE_LONG  (1073741824L)
#define ROT_MATRIX_SCALE_FLOAT (1073741824.0f)
#define ROT_MATRIX_LONG_TO_FLOAT( intval ) \
    ((float) ((intval) / ROT_MATRIX_SCALE_FLOAT ))
#define SIGNM(k)((int)(k)&1?-1:1)
#define SIGNSET(x) ((x) ? -1 : +1)

#define INV_TWO_POWER_NEG_30 9.313225746154785e-010f

#ifdef __cplusplus
extern "C" {
#endif

     typedef struct {
        float state[4];
        float c[5];
        float input;
        float output;
    }   inv_biquad_filter_t;

    static inline float inv_q30_to_float(int q30)
    {
        return (float) q30 / ((float)(1L << 30));
    }

    static inline double inv_q30_to_double(int q30)
    {
        return (double) q30 / ((double)(1L << 30));
    }

    static inline float inv_q16_to_float(int q16)
    {
        return (float) q16 / (1L << 16);
    }

    static inline double inv_q16_to_double(int q16)
    {
        return (double) q16 / (1L << 16);
    }




    int inv_q29_mult(int a, int b);
    int inv_q30_mult(int a, int b);

    /* UMPL_ELIMINATE_64BIT Notes:
     * An alternate implementation using float instead of long long accudoublemulators
     * is provided for q29_mult and q30_mult.
     * When long long accumulators are used and an alternate implementation is not
     * available, we eliminate the entire function and header with a macro.
     */
#ifndef UMPL_ELIMINATE_64BIT
    int inv_q30_div(int a, int b);
    int inv_q_shift_mult(int a, int b, int shift);
#endif
	void inv_triad(int *accel_body, int *compass_body, int accel_fs, int *Qbi_fp);
	void inv_rotation_to_quaternion(float Rcb[9], int Qcb_fp[4]);
	void inv_convert_quaternion_to_body(int *mcb,
									const int *Qc, int *Qb);
	void inv_orientation_scalar_to_matrix(unsigned short orientation, int *output);
    void inv_q_mult(const int *q1, const int *q2, int *qProd);
    void inv_q_add(int *q1, int *q2, int *qSum);
    void inv_q_normalize(int *q);
    void inv_q_invert(const int *q, int *qInverted);
    void inv_q_multf(const float *q1, const float *q2, float *qProd);
    void inv_q_addf(const float *q1, const float *q2, float *qSum);
    void inv_q_normalizef(float *q);
    void inv_q_norm4(float *q);
    void inv_q_invertf(const float *q, float *qInverted);
    void inv_quaternion_to_rotation(const int *quat, int *rot);
    unsigned char *inv_int32_to_big8(int x, unsigned char *big8);
    int inv_big8_to_int32(const unsigned char *big8);
    short inv_big8_to_int16(const unsigned char *big8);
    short inv_little8_to_int16(const unsigned char *little8);
    unsigned char *inv_int16_to_big8(short x, unsigned char *big8);
    float inv_matrix_det(float *p, int *n);
    void inv_matrix_det_inc(float *a, float *b, int *n, int x, int y);
    double inv_matrix_detd(double *p, int *n);
    void inv_matrix_det_incd(double *a, double *b, int *n, int x, int y);
    float inv_wrap_angle(float ang);
    float inv_angle_diff(float ang1, float ang2);
    void inv_quaternion_to_rotation_vector(const int *quat, int *rot);
    unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
    void inv_convert_to_body(unsigned short orientation, const int *input, int *output);
    void inv_convert_to_chip(unsigned short orientation, const int *input, int *output);
    void inv_convert_to_body_with_scale(unsigned short orientation, int sensitivity, const int *input, int *output);
    void inv_q_rotate(const int *q, const int *in, int *out);
	void inv_vector_normalize(int *vec, int length);
    uint32_t inv_checksum(const unsigned char *str, int len);
    float inv_compass_angle(const int *compass, const int *grav,
                            const float *quat);
    unsigned int inv_get_gyro_sum_of_sqr(const int *gyro);

    static inline int inv_delta_time_ms(inv_time_t t1, inv_time_t t2)
    {
        return (int)((t1 - t2) / 1000000L);
    }

    double quaternion_to_rotation_angle(const int *quat);
    double inv_vector_norm(const float *x);

    void inv_init_biquad_filter(inv_biquad_filter_t *pFilter, float *pBiquadCoeff);
    float inv_biquad_filter_process(inv_biquad_filter_t *pFilter, float input);
    void inv_calc_state_to_match_output(inv_biquad_filter_t *pFilter, float input);
    void inv_get_cross_product_vec(float *cgcross, float compass[3], float grav[3]);

    void mlMatrixVectorMult(int matrix[9], const int vecIn[3], int *vecOut);

	int inv_inverse_sqrt(int x0, int*rempow);
	int inv_fast_sqrt(int x0);
	int inv_one_over_x(int x0, int*pow);
	int test_limits_and_scale(int *x0, int *pow);
	int get_highest_bit_position(unsigned int *value);
    int inv_compute_scalar_part(const int * inQuat, int* outQuat);
    int inv_fastsine29(int x);
    int inv_fastcosine29(int x);

	// Q15 Math functions
    int inverse_sqrt_q15(int x);
    int sqrt_fun_q15(int x);
    int reciprocal_fun_q15(int x);
    int atan2_q15(int Y, int X);
    int inv_fastsine_q15(int x);
    int inv_fastcosine_q15(int x);

	// Q30 Math functions
    int inverse_sqrt_q30(int x, int *pow2);
    int sqrt_fun_q30(int x);
    int reciprocal_fun_q30(int x, int *pow2);
    int num_over_den_q30(int x, int y, int *pow2);
#ifdef __cplusplus
}
#endif
#endif // INVENSENSE_INV_MATH_FUNC_H__
