/**
 *  Self Test application for Invensense's devices
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>

#include "storage_manager.h"
#include "ml_stored_data.h"
#include "data_builder.h"
#include "ml_sysfs_helper.h"

#ifndef ABS
#define ABS(x)(((x) >= 0) ? (x) : -(x))
#endif

#define DEBUG_PRINT     /* Uncomment to print Gyro & Accel read from Driver */

#define MAX_SYSFS_NAME_LEN     (100)
#define MAX_SYSFS_ATTRB        (sizeof(struct sysfs_attrbs) / sizeof(char*))
//#define IIO_SYSFS_PATH         "/sys/bus/iio/devices/iio:device0"

#define GYRO_PASS_STATUS_BIT    0x01
#define ACCEL_PASS_STATUS_BIT   0x02
#define COMPASS_PASS_STATUS_BIT 0x04

/* will be set true if the chip is in the list above */
static bool support_gyro_ois_bias = false;
static bool support_accel_ois_bias = false;

static char *sysfs_names_ptr;
static struct sysfs_attrbs {
    char *name;
    char *enable;
    int enable_value;
    char *power_state;
    int power_state_value;
    char *dmp_on;
    int dmp_on_value;
    char *self_test;
    char *temperature;
    char *gyro_enable;
    int gyro_enable_value;
    char *gyro_x_calibbias;
    char *gyro_y_calibbias;
    char *gyro_z_calibbias;
    char *gyro_x_ois_calibbias;
    char *gyro_y_ois_calibbias;
    char *gyro_z_ois_calibbias;
    char *gyro_scale;
    char *gyro_x_offset;
    char *gyro_y_offset;
    char *gyro_z_offset;
    char *accel_enable;
    int accel_enable_value;
    char *accel_x_calibbias;
    char *accel_y_calibbias;
    char *accel_z_calibbias;
    char *accel_x_ois_calibbias;
    char *accel_y_ois_calibbias;
    char *accel_z_ois_calibbias;
    char *accel_scale;
    char *accel_x_offset;
    char *accel_y_offset;
    char *accel_z_offset;
    char *compass_enable;
    int compass_enable_value;
} mpu;

static struct inv_db_save_t save_data;
static struct inv_db_save_t save_data_2;
static bool write_biases_immediately = false;

/** This function receives the data that was stored in non-volatile memory
    between power off */
static inv_error_t inv_db_load_func(const unsigned char *data)
{
    memcpy(&save_data, data, sizeof(save_data));
    return INV_SUCCESS;
}

/** This function returns the data to be stored in non-volatile memory between
    power off */
static inv_error_t inv_db_save_func(unsigned char *data)
{
    memcpy(data, &save_data, sizeof(save_data));
    return INV_SUCCESS;
}

/** This function receives the data that was stored in non-volatile memory
    between power off */
static inv_error_t inv_db_load_2_func(const unsigned char *data)
{
    memcpy(&save_data_2, data, sizeof(save_data_2));
    return INV_SUCCESS;
}

/** This function returns the data to be stored in non-volatile memory between
    power off */
static inv_error_t inv_db_save_2_func(unsigned char *data)
{
    memcpy(data, &save_data_2, sizeof(save_data_2));
    return INV_SUCCESS;
}

/** read a sysfs entry that represents an integer */
static int read_sysfs_int(char *filename, int *var)
{
    int res=0;
    FILE *fp;

    fp = fopen(filename, "r");
    if (fp != NULL) {
        fscanf(fp, "%d\n", var);
        fclose(fp);
    } else {
        printf("inv_self_test: ERR open file to read %s", filename);
        res= -1;
    }
    return res;
}

/** write a sysfs entry that represents an integer */
static int write_sysfs_int(char *filename, int data)
{
    int res = 0;
    FILE *fp;

    fp = fopen(filename, "w");
    if (fp != NULL) {
        fprintf(fp, "%d\n", data);
        fclose(fp);
    } else {
        printf("inv_self_test: ERR open file to write");
        res= -1;
    }
    return res;
}

static int get_chip_name(char *chip_name)
{
    FILE *fp;
    size_t i;

    fp = fopen(mpu.name, "r");
    if (fp != NULL) {
        fgets(chip_name, 10, fp);
        fclose(fp);
    } else {
        printf("inv_self_test: ERR open file to read");
        return -1;
    }

    for (i = 0; i < strlen(chip_name); i++) {
        chip_name[i] = tolower(chip_name[i]);
    }
    return 0;
}

static bool check_ois_support(void)
{
    bool res = true;
    FILE *fp;

    fp = fopen(mpu.gyro_x_ois_calibbias, "r");
    if (fp == NULL) {
        res = false;
    } else {
        fclose(fp);
    }
    return res;
}

static int inv_init_sysfs_attributes(void)
{
    unsigned char i = 0;
    char sysfs_path[MAX_SYSFS_NAME_LEN];
    char *sptr;
    char **dptr;

    sysfs_names_ptr =
            (char*)malloc(sizeof(char[MAX_SYSFS_ATTRB][MAX_SYSFS_NAME_LEN]));
    sptr = sysfs_names_ptr;
    if (sptr != NULL) {
        dptr = (char**)&mpu;
        do {
            *dptr++ = sptr;
            sptr += sizeof(char[MAX_SYSFS_NAME_LEN]);
        } while (++i < MAX_SYSFS_ATTRB);
    } else {
        printf("inv_self_test: couldn't alloc mem for sysfs paths");
        return -1;
    }

    // Setup IIO sysfs path & build MPU's sysfs paths
    //sprintf(sysfs_path, "%s", IIO_SYSFS_PATH);
	memset(sysfs_path, 0 , sizeof(sysfs_path));
	inv_get_sysfs_path(sysfs_path);
	printf("sysfs path=%s\n", sysfs_path);
    sprintf(mpu.name, "%s%s", sysfs_path, "/name");
    sprintf(mpu.self_test, "%s%s", sysfs_path, "/misc_self_test");
    sprintf(mpu.temperature, "%s%s", sysfs_path, "/out_temperature");

    sprintf(mpu.gyro_x_calibbias, "%s%s",
            sysfs_path, "/in_anglvel_x_st_calibbias");
    sprintf(mpu.gyro_y_calibbias, "%s%s",
            sysfs_path, "/in_anglvel_y_st_calibbias");
    sprintf(mpu.gyro_z_calibbias, "%s%s",
            sysfs_path, "/in_anglvel_z_st_calibbias");
    sprintf(mpu.gyro_x_ois_calibbias, "%s%s",
            sysfs_path, "/in_anglvel_x_ois_st_calibbias");
    sprintf(mpu.gyro_y_ois_calibbias, "%s%s",
            sysfs_path, "/in_anglvel_y_ois_st_calibbias");
    sprintf(mpu.gyro_z_ois_calibbias, "%s%s",
            sysfs_path, "/in_anglvel_z_ois_st_calibbias");

    sprintf(mpu.gyro_x_offset, "%s%s", sysfs_path, "/in_anglvel_x_offset");
    sprintf(mpu.gyro_y_offset, "%s%s", sysfs_path, "/in_anglvel_y_offset");
    sprintf(mpu.gyro_z_offset, "%s%s", sysfs_path, "/in_anglvel_z_offset");

    sprintf(mpu.accel_x_calibbias, "%s%s", sysfs_path, "/in_accel_x_st_calibbias");
    sprintf(mpu.accel_y_calibbias, "%s%s", sysfs_path, "/in_accel_y_st_calibbias");
    sprintf(mpu.accel_z_calibbias, "%s%s", sysfs_path, "/in_accel_z_st_calibbias");
    sprintf(mpu.accel_x_ois_calibbias, "%s%s", sysfs_path, "/in_accel_x_ois_st_calibbias");
    sprintf(mpu.accel_y_ois_calibbias, "%s%s", sysfs_path, "/in_accel_y_ois_st_calibbias");
    sprintf(mpu.accel_z_ois_calibbias, "%s%s", sysfs_path, "/in_accel_z_ois_st_calibbias");
    sprintf(mpu.accel_x_offset, "%s%s", sysfs_path, "/in_accel_x_offset");
    sprintf(mpu.accel_y_offset, "%s%s", sysfs_path, "/in_accel_y_offset");
    sprintf(mpu.accel_z_offset, "%s%s", sysfs_path, "/in_accel_z_offset");


#if 0
    // test print sysfs paths
    dptr = (char**)&mpu;
    for (i = 0; i < MAX_SYSFS_ATTRB; i++) {
        printf("inv_self_test: sysfs path: %s\n", *dptr++);
    }
#endif
    return 0;
}

static void print_cal_file_content(struct inv_db_save_t *data)
{
    printf("------------------------------\n");
    printf("-- Calibration file content --\n");
    printf("   factory_gyro_bias[3]  = %+d %+d %+d\n",
           data->factory_gyro_bias[0], data->factory_gyro_bias[1],
           data->factory_gyro_bias[2]);
    printf("   factory_accel_bias[3] = %+d %+d %+d\n",
           data->factory_accel_bias[0], data->factory_accel_bias[1],
           data->factory_accel_bias[2]);
    printf("   compass_bias[3]      = %+d %+d %+d\n",
           data->compass_bias[0], data->compass_bias[1], data->compass_bias[2]);
    printf("   gyro_temp            = %+d\n", data->gyro_temp);
    printf("   gyro_bias_tc_set     = %+d\n", data->gyro_bias_tc_set);
    printf("   accel_temp           = %+d\n", data->accel_temp);
    printf("   gyro_accuracy        = %d\n", data->gyro_accuracy);
    printf("   accel_accuracy       = %d\n", data->accel_accuracy);
    printf("   compass_accuracy     = %d\n", data->compass_accuracy);
    printf("------------------------------\n");
}

static void print_usage(char *argv0)
{
    printf("%s\n", argv0);
    printf("   -l : show the contents of %s (only after running self test)\n", MLCAL_FILE);
    printf("   -w : write bias to driver after running self test\n");
    printf("   -h : show help\n");
}

/*******************************************************************************
 *                       M a i n
 ******************************************************************************/
int main(int argc, char **argv)
{
    FILE *fptr;
    int self_test_status = 0;
    inv_error_t result;
    int gyro_bias[3];
    int gyro_bias_lp[3];
    int gyro_scale;
    int accel_bias[3];
    int accel_bias_lp[3];
    int accel_scale;
    int scale_ratio;
    int axis_sign = 1;
    int timestamp;
    long long temperature = 0;
    bool compass_present = true;
    bool gyro_present = true;
    int c;
    char chip_name[16];

    result= inv_init_sysfs_attributes();
    if (result)
        return -1;

    inv_init_storage_manager();

    // Register packet to be saved.
    result = inv_register_load_store(
                inv_db_load_func, inv_db_save_func,
                sizeof(save_data), INV_DB_SAVE_KEY);
    result = inv_register_load_store(
                inv_db_load_2_func, inv_db_save_2_func,
                sizeof(save_data), INV_DB_SAVE_2_KEY);

    // get chip name
    if (get_chip_name(chip_name) == 0) {
        printf("Self-Test: chip name = %s\n", chip_name);
    } else {
        printf("Self-Test:ERR-Couldn't read chip name\n");
        result = -1;
        goto free_sysfs_storage;
    }

    // check ois support
    if (check_ois_support()) {
        printf("Self-Test:Ois supported\n");
        support_gyro_ois_bias = true;
        support_accel_ois_bias = true;
    } else {
        printf("Self-Test:Ois not supported\n");
        support_gyro_ois_bias = false;
        support_accel_ois_bias = false;
    }

    /* testing hook: if the command-line parameter is '-l' as in 'load',
       the system will read out the MLCAL_FILE */
    while ((c = getopt(argc, argv, "lwh")) != -1) {
        switch (c) {
        case 'l':
            result = inv_load_calibration();
            if (result) {
                printf("Self-Test: cannot load calibration file - error %d\n", result);
                goto free_sysfs_storage;
            }
            printf("== Data set 1 ==\n");
            print_cal_file_content(&save_data);
            printf("== Data set 2 ==\n");
            print_cal_file_content(&save_data_2);
            goto free_sysfs_storage;
            break;

        case 'w':
            write_biases_immediately = true;
            break;

        case 'h':
            print_usage(argv[0]);
            result = 0;
            goto free_sysfs_storage;
        }
    }

    // Check if offset registers are already set
    if (read_sysfs_int(mpu.gyro_x_offset, &gyro_bias[0]) < 0 ||
        read_sysfs_int(mpu.gyro_y_offset, &gyro_bias[1]) < 0 ||
        read_sysfs_int(mpu.gyro_z_offset, &gyro_bias[2]) < 0 ||
        read_sysfs_int(mpu.accel_x_offset, &accel_bias[0]) < 0 ||
        read_sysfs_int(mpu.accel_y_offset, &accel_bias[1]) < 0 ||
        read_sysfs_int(mpu.accel_z_offset, &accel_bias[2]) < 0) {

        printf("Self-Test:Failed to read Gyro / Accel initial  bias\n");
        result = -1;
        goto free_sysfs_storage;
    }
    if (gyro_bias[0]  || gyro_bias[1]  || gyro_bias[2] ||
        accel_bias[0] || accel_bias[1] || accel_bias[2]) {
        //
        // factory bias already set.
        // Need to remove cal file, reboot and run self test again.
        //
        printf("Self-Test:Offset registers have been already set\n");
        result = -1;
        goto free_sysfs_storage;
    }

    // Clear out data.
    memset(&save_data, 0, sizeof(save_data));
    memset(&save_data_2, 0, sizeof(save_data_2));
    memset(gyro_bias,0, sizeof(gyro_bias));
    memset(gyro_bias_lp,0, sizeof(gyro_bias_lp));
    memset(accel_bias,0, sizeof(accel_bias));
    memset(accel_bias_lp,0, sizeof(accel_bias_lp));

    fptr = fopen(mpu.self_test, "r");
    if (!fptr) {
        printf("Self-Test:ERR-Couldn't invoke self-test\n");
        result = -1;
        goto free_sysfs_storage;
    }

    // Invoke self-test
    fscanf(fptr, "%d", &self_test_status);
    fclose(fptr);
    printf("Self-Test:Self test result ");
    printf("Accel passed = %s", (self_test_status & ACCEL_PASS_STATUS_BIT) ? "PASS" : "FAIL");
    if (gyro_present == true) {
        printf(", Gyro passed = %s", (self_test_status & GYRO_PASS_STATUS_BIT) ? "PASS" : "FAIL");
    }
    if (compass_present == true) {
        printf(", AUX Compass passed = %s", (self_test_status & COMPASS_PASS_STATUS_BIT) ? "PASS" : "FAIL");
    }
    printf("\n");

    gyro_scale = 0;
    if (gyro_present == true) {
        if (self_test_status & GYRO_PASS_STATUS_BIT) {
            // Read Gyro Bias
            if (support_gyro_ois_bias) {
                if (read_sysfs_int(mpu.gyro_x_calibbias, &gyro_bias_lp[0]) < 0 ||
                        read_sysfs_int(mpu.gyro_y_calibbias, &gyro_bias_lp[1]) < 0 ||
                        read_sysfs_int(mpu.gyro_z_calibbias, &gyro_bias_lp[2]) < 0 ||
                        read_sysfs_int(mpu.gyro_x_ois_calibbias, &gyro_bias[0]) < 0 ||
                        read_sysfs_int(mpu.gyro_y_ois_calibbias, &gyro_bias[1]) < 0 ||
                        read_sysfs_int(mpu.gyro_z_ois_calibbias, &gyro_bias[2]) < 0 ) {
                    printf("Self-Test:Failed to read Gyro bias\n");
                    result = -1;
                    goto free_sysfs_storage;
                }
            } else {
                if (read_sysfs_int(mpu.gyro_x_calibbias, &gyro_bias[0]) < 0 ||
                        read_sysfs_int(mpu.gyro_y_calibbias, &gyro_bias[1]) < 0 ||
                        read_sysfs_int(mpu.gyro_z_calibbias, &gyro_bias[2]) < 0 ) {
                    printf("Self-Test:Failed to read Gyro bias\n");
                    result = -1;
                    goto free_sysfs_storage;
                }
            }
            gyro_scale = 250;
            save_data.gyro_accuracy = 3;
            if (support_gyro_ois_bias) {
                save_data_2.gyro_accuracy = 3;
            }
#ifdef DEBUG_PRINT
            if (support_gyro_ois_bias)
                printf("Self-Test:Gyro bias lp[0..2] = [%d %d %d]\n",
                        gyro_bias_lp[0], gyro_bias_lp[1], gyro_bias_lp[2]);
            printf("Self-Test:Gyro bias[0..2] = [%d %d %d]\n",
                    gyro_bias[0], gyro_bias[1], gyro_bias[2]);
#endif
        } else {
            printf("Self-Test:Failed Gyro self-test\n");
        }
    }

    accel_scale = 0;
    if (self_test_status & ACCEL_PASS_STATUS_BIT) {
        // Read Accel Bias
        if (support_accel_ois_bias) {
            if (read_sysfs_int(mpu.accel_x_calibbias, &accel_bias_lp[0]) < 0 ||
                read_sysfs_int(mpu.accel_y_calibbias, &accel_bias_lp[1]) < 0 ||
                read_sysfs_int(mpu.accel_z_calibbias, &accel_bias_lp[2]) < 0 ||
                read_sysfs_int(mpu.accel_x_ois_calibbias, &accel_bias[0]) < 0 ||
                read_sysfs_int(mpu.accel_y_ois_calibbias, &accel_bias[1]) < 0 ||
                read_sysfs_int(mpu.accel_z_ois_calibbias, &accel_bias[2]) < 0 ) {
                printf("Self-Test:Failed to read Accel bias\n");
                result = -1;
                goto free_sysfs_storage;
            }
        } else {
            if (read_sysfs_int(mpu.accel_x_calibbias, &accel_bias[0]) < 0 ||
                read_sysfs_int(mpu.accel_y_calibbias, &accel_bias[1]) < 0 ||
                read_sysfs_int(mpu.accel_z_calibbias, &accel_bias[2]) < 0 ) {
                printf("Self-Test:Failed to read Accel bias\n");
                result = -1;
                goto free_sysfs_storage;
            }
        }
        accel_scale = 2;
        save_data.accel_accuracy = 3;
        if (support_accel_ois_bias) {
            save_data_2.accel_accuracy = 3;
        }
#ifdef DEBUG_PRINT
        if (support_accel_ois_bias)
            printf("Self-Test:Accel bias lp[0..2] = [%d %d %d]\n",
                    accel_bias_lp[0], accel_bias_lp[1], accel_bias_lp[2]);
        printf("Self-Test:Accel bias[0..2] = [%d %d %d]\n",
                accel_bias[0], accel_bias[1], accel_bias[2]);
#endif
    } else {
        printf("Self-Test:Failed Accel self-test\n");
    }

    if (!(self_test_status & (GYRO_PASS_STATUS_BIT | ACCEL_PASS_STATUS_BIT))) {
        printf("Self-Test:Failed Gyro and Accel self-test, "
               "nothing left to do\n");
        result = -1;
        goto free_sysfs_storage;
    }

    // Read temperature
    fptr = fopen(mpu.temperature, "r");
    if (fptr != NULL) {
        fscanf(fptr,"%lld %d", &temperature, &timestamp);
        fclose(fptr);
    } else {
        printf("Self-Test:ERR-Couldn't read temperature\n");
    }

    /*
        When we read gyro bias from sysfs, the bias is in the raw units
        at the full scale used during self-test
        We store the biases in raw units (+/- 2000 dps full scale assumed)
        scaled by 2^16 therefore the gyro_bias[] have to be divided by the
        ratio of 2000 / gyro_scale to comply.
    */
    // TODO read this from the regular full scale in sysfs
    if (gyro_scale) {
        scale_ratio = 2000 / gyro_scale;
        save_data.factory_gyro_bias[0] =
            (int)(gyro_bias[0] * 65536.f / scale_ratio);
        save_data.factory_gyro_bias[1] =
            (int)(gyro_bias[1] * 65536.f / scale_ratio);
        save_data.factory_gyro_bias[2] =
            (int)(gyro_bias[2] * 65536.f / scale_ratio);
        if (support_gyro_ois_bias) {
            save_data_2.factory_gyro_bias[0] =
                (int)(gyro_bias_lp[0] * 65536.f / scale_ratio);
            save_data_2.factory_gyro_bias[1] =
                (int)(gyro_bias_lp[1] * 65536.f / scale_ratio);
            save_data_2.factory_gyro_bias[2] =
                (int)(gyro_bias_lp[2] * 65536.f / scale_ratio);
        }
    } else {
        save_data.factory_gyro_bias[0] = 0;
        save_data.factory_gyro_bias[1] = 0;
        save_data.factory_gyro_bias[2] = 0;
        if (support_gyro_ois_bias) {
            save_data_2.factory_gyro_bias[0] = 0;
            save_data_2.factory_gyro_bias[1] = 0;
            save_data_2.factory_gyro_bias[2] = 0;
        }
    }

    // Save temperature @ time stored.
    //  Temperature is in degrees Celsius scaled by 2^16
    save_data.gyro_temp = temperature * (1L << 16);
    save_data.gyro_bias_tc_set = true;
    save_data.accel_temp = save_data.gyro_temp;
    if (support_gyro_ois_bias) {
        save_data_2.gyro_temp = temperature * (1L << 16);
        save_data_2.gyro_bias_tc_set = true;
        save_data_2.accel_temp = save_data.gyro_temp;
    }

    // When we read accel bias, the bias is in raw units
    // and it contains the gravity vector.
    // Find the orientation of the device, by looking for gravity
    int axis = 0;
    if (ABS(accel_bias[1]) > ABS(accel_bias[0])) {
        axis = 1;
    }
    if (ABS(accel_bias[2]) > ABS(accel_bias[axis])) {
        axis = 2;
    }
    if (accel_bias[axis] < 0) {
        axis_sign = -1;
    }

    // Convert scaling from raw units to raw scaled by 2^16
    /*
        When we read accel bias from sysfs, the bias is in the raw units
        at the full scale used during self-test
        We store the biases in raw units (+/- 2 gee full scale assumed)
        scaled by 2^16 therefore the accel_bias[] have to be multipled by the
        ratio of accel_scale / 2 to comply.
    */
    // TODO read this from the regular full scale in sysfs
    if (accel_scale) {
        scale_ratio = accel_scale / 2;
        save_data.factory_accel_bias[0] =
            (int)(accel_bias[0] * 65536.f * scale_ratio);
        save_data.factory_accel_bias[1] =
            (int)(accel_bias[1] * 65536.f * scale_ratio);
        save_data.factory_accel_bias[2] =
            (int)(accel_bias[2] * 65536.f * scale_ratio);
        if (support_accel_ois_bias) {
            save_data_2.factory_accel_bias[0] =
                (int)(accel_bias_lp[0] * 65536.f * scale_ratio);
            save_data_2.factory_accel_bias[1] =
                (int)(accel_bias_lp[1] * 65536.f * scale_ratio);
            save_data_2.factory_accel_bias[2] =
                (int)(accel_bias_lp[2] * 65536.f * scale_ratio);
        }
    } else {
        save_data.factory_accel_bias[0] = 0;
        save_data.factory_accel_bias[1] = 0;
        save_data.factory_accel_bias[2] = 0;
        if (support_accel_ois_bias) {
            save_data_2.factory_accel_bias[0] = 0;
            save_data_2.factory_accel_bias[1] = 0;
            save_data_2.factory_accel_bias[2] = 0;
        }
    }

#ifdef DEBUG_PRINT
    printf("Self-Test:Saved Accel bias[0..2] = [%d %d %d]\n",
            save_data.factory_accel_bias[0], save_data.factory_accel_bias[1],
            save_data.factory_accel_bias[2]);
    if (support_accel_ois_bias)
        printf("Self-Test:Saved Accel bias lp[0..2] = [%d %d %d]\n",
                save_data_2.factory_accel_bias[0], save_data_2.factory_accel_bias[1],
                save_data_2.factory_accel_bias[2]);
#endif

    /*
        Remove gravity, gravity in raw units should be 16384 = 2^14 for a +/-
        2 gee setting. The data has been saved in Hw units scaled to 2^16,
        so gravity needs to scale up accordingly.
    */
    if (accel_scale) {
        /* gravity correction for accel_bias format */
        int gravity = axis_sign * (32768 / accel_scale);
        accel_bias[axis] -= gravity;
        if (support_accel_ois_bias)
            accel_bias_lp[axis] -= gravity;
        /* gravity correction for saved_data.accel_bias format */
        gravity = axis_sign * (32768 / accel_scale) * 65536;
#ifdef DEBUG_PRINT
        printf("Self-Test:Gravity = %d\n", gravity);
#endif
        save_data.factory_accel_bias[axis] -= gravity;
        if (support_accel_ois_bias)
            save_data_2.factory_accel_bias[axis] -= gravity;
    }

    printf("Self-Test:Saved Accel bias (gravity removed) [0..2] = "
           "[%d %d %d]\n",
           save_data.factory_accel_bias[0], save_data.factory_accel_bias[1],
           save_data.factory_accel_bias[2]);
    if (support_accel_ois_bias)
        printf("Self-Test:Saved Accel bias lp (gravity removed) [0..2] = "
                "[%d %d %d]\n",
                save_data_2.factory_accel_bias[0], save_data_2.factory_accel_bias[1],
                save_data_2.factory_accel_bias[2]);

    if (accel_scale) {
        /* write the accel biases down to the hardware now */
        if (write_biases_immediately) {
            int offsets[3] = {0};
            /* driver expects raw << 16 @ 2g */
            scale_ratio = accel_scale / 2;
            offsets[0] = accel_bias[0] * 65536.f * scale_ratio;
            if (write_sysfs_int(mpu.accel_x_offset, offsets[0]) < 0) {
                printf("Self-Test:ERR-Failed to write %s\n", mpu.accel_x_offset);
            }
            offsets[1] = accel_bias[1] * 65536.f * scale_ratio;
            if (write_sysfs_int(mpu.accel_y_offset, offsets[1]) < 0) {
                printf("Self-Test:ERR-Failed to write %s\n", mpu.accel_y_offset);
            }
            offsets[2] = accel_bias[2] * 65536.f * scale_ratio;
            if (write_sysfs_int(mpu.accel_z_offset, offsets[2]) < 0) {
                printf("Self-Test:ERR-Failed to write %s\n", mpu.accel_z_offset);
            }
            printf("Self-Test:Written Accel offsets[0..2] = [%d %d %d]\n",
                   offsets[0], offsets[1], offsets[2]);
        }
    }

    printf("Self-Test:Saved Gyro bias[0..2] = [%d %d %d]\n",
           save_data.factory_gyro_bias[0], save_data.factory_gyro_bias[1],
           save_data.factory_gyro_bias[2]);
    if (support_gyro_ois_bias)
        printf("Self-Test:Saved Gyro bias lp[0..2] = [%d %d %d]\n",
                save_data_2.factory_gyro_bias[0], save_data_2.factory_gyro_bias[1],
                save_data_2.factory_gyro_bias[2]);

    if (gyro_scale) {
        /* write the gyro biases down to the hardware now */
        if (write_biases_immediately) {
            if (gyro_present == true) {
                int offsets[3] = {0};
                if (gyro_scale) {
                    /* driver expects raw << 16 @ 2000dps */
                    scale_ratio = 2000 / gyro_scale;
                    offsets[0] = gyro_bias[0] * 65536.f / scale_ratio;
                    if (write_sysfs_int(mpu.gyro_x_offset, offsets[0]) < 0) {
                        printf("Self-Test:ERR-Failed to write %s\n", mpu.gyro_x_offset);
                    }
                    offsets[1] = gyro_bias[1] * 65536.f / scale_ratio;
                    if (write_sysfs_int(mpu.gyro_y_offset, offsets[1]) < 0) {
                        printf("Self-Test:ERR-Failed to write %s\n", mpu.gyro_y_offset);
                    }
                    offsets[2] = gyro_bias[2] * 65536.f / scale_ratio;
                    if (write_sysfs_int(mpu.gyro_z_offset, offsets[2]) < 0) {
                        printf("Self-Test:ERR-Failed to write %s\n", mpu.gyro_z_offset);
                    }
                    printf("Self-Test:Written Gyro offsets[0..2] = [%d %d %d]\n",
                            offsets[0], offsets[1], offsets[2]);
                }
            }
        }
    }

    printf("Self-Test:Gyro temperature @ time stored %d\n",
           save_data.gyro_temp);
    printf("Self-Test:Accel temperature @ time stored %d\n",
           save_data.accel_temp);

    result = inv_store_calibration();
    if (result) {
        printf("Self-Test:ERR- Can't save bias into calibration file\n");
    }

free_sysfs_storage:
    free(sysfs_names_ptr);
    return result;
}