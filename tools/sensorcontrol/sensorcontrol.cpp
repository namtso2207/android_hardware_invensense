/*
 * Copyright (C) 2010 The Android Open Source Project
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

#include <stdint.h>
#include <inttypes.h>
#include <android/sensor.h>
#include <sensor/Sensor.h>
#include <sensor/SensorManager.h>
#include <sensor/SensorEventQueue.h>
#include <utils/Looper.h>

#include <utils/Vector.h>
#include <utils/KeyedVector.h>
#include <utils/SystemClock.h>

//#define DEF_RESTRICT_VALID_RATES

using namespace android;

/*
    Prototypes
*/
nsecs_t now_ns(void);

/*
    Typedefs
*/
typedef unsigned long msecs_t;

// TODO clean up formatting
class SensorStats {
    private:
        Sensor const* sensorHandle;
        bool enabled;

    public:
        int sampleCount;
        nsecs_t timestamp;
        int rateHz;

    public:
        SensorStats();
        SensorStats(Sensor const* sensor);
        ~SensorStats() {};

        Sensor const* getSensor() const { return sensorHandle; }

        void setRate(int rate) { rateHz = rate; }
        const int& getRate() const { return rateHz; }

        void setEnable(bool en) { enabled = en; }
        const bool& getEnable() const { return enabled; }

        // shorthand for sensor->getName().string()
        const char* getName() const;
        // short hand for sensor->getType()
        int getType() const;

        void clearStats() { sampleCount = 0; timestamp = 0UL; }
        int getSampleCount() const { return sampleCount; }
        //SensorStats& operator++(int count)
        //                          { sampleCount++; return *this; }
        //SensorStats& operator+=(int count)
        //                          { sampleCount += count; return *this; }
        //SensorStats& operator=(int count)
        //                          { sampleCount = count; return *this; }
};

SensorStats::SensorStats() : sensorHandle(NULL)
{
    // info
    rateHz = 0;
    // stats
    sampleCount = 0;
    timestamp = now_ns();
}

SensorStats::SensorStats(Sensor const* sensor) : sensorHandle(sensor)
{
    // info
    rateHz = 0;
    // stats
    sampleCount = 0;
    timestamp = now_ns();
}

const char* SensorStats::getName() const
{
    if (sensorHandle)
        return (sensorHandle->getName().string());
    else
        return ("<not set>");
}

int SensorStats::getType() const
{
    if (sensorHandle)
        return int(sensorHandle->getType());
    else
        return (-1);
}

/*
    Global variables
*/
enum batchModes {
    BATCH_LEAVE_UNCHANGED = 0,
    BATCH_FIFOFULL,
    BATCH_TIMEOUT,
};

static const bool mplOnly = false;
static bool printOnScreen = false;
static bool printMetadata = false;
static enum batchModes batchMode = BATCH_LEAVE_UNCHANGED;
static bool batchModeDryrun = false;
static bool verbose = false;
static bool debug = false;

static int numSensors = 0;
static const int nReqInputPars = 1;
#ifdef DEF_RESTRICT_VALID_RATES
static const int validRates[] = {5, 14, 50, 100, 200};
#else
static const int validRateRange[] = {1, 1000};
#endif
KeyedVector<int32_t, int8_t> handle2index;

// time tracking
static msecs_t beforePollMs = 0UL;
static msecs_t flushTimeMs = 0UL, flushLastTimeMs = 0UL;
static char flushMode = ' ';

// list of sensors and personalized statistics for each
static Vector<SensorStats> sensorStatsList;
static int firstSensorIndex = -1;

/*
    Functions
*/

/**
 *  Get current timestamp in nanoseconds
 */
nsecs_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_BOOTTIME, &ts);
    //return (nsecs_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
    return elapsedRealtimeNano();
}

msecs_t now_ms(void)
{
    return (msecs_t)(now_ns() / 1000000LL);
}

/**
 *  Lookup array index based on the info in the ASensorEvent
 */
int32_t build_sensor_id(int32_t sensor, int32_t type)
{
    if (sensor > 100)
        return (type + 20);
    else
        return sensor;
}

/**
 *  Callback to process any incoming sensor data
 */
static int Total_time = 0;
static int Print_batch_duration=0;
static int Total_sensors=0;
int Reset_flag=0;
int receiver(int fd, int events, void* data)
{
    sp<SensorEventQueue> q((SensorEventQueue*)data);
    ssize_t n;
    // TODO changed buffer and read size from 8 bytes to 64, in the hope it
    //      helps mitigating the data loss observed in batch mode. Revise.
    const size_t bufferSize = 64;
    ASensorEvent buffer[bufferSize];
    static msecs_t lastAfterPollMs = 0UL;
    (void)fd; (void)events;

    // grab a timestamp here and compare to the timestamp before poll -
    //  in batch mode the time between these should match the expected batch
    //  time
    msecs_t afterPollMs = now_ms();
    if (batchMode) {
        // trying to detect situation where a longer than normal time
        //  was spent hanging on the pollOnce call. This may indicate a batch
        //  was received instead of continuous data. This method is totally
        //  empirical and will certainly not work as the batching time
        //  approaches 4 x (1 / 5) = 800 ms, that is twice the longest
        //  interarrival period
        msecs_t lastMs = lastAfterPollMs;
        Total_sensors=0;
        // if (lastMs == 0UL)
        //     lastMs = beforePollMs;
        if ((afterPollMs - lastMs) > 1 * 200UL) {
            // sample count statistic from the previous batch
            for (int ii = 0; ii < (int)sensorStatsList.size(); ii++) {
                if (!sensorStatsList[ii].getEnable())
                    continue;
                Total_sensors+=1;
                if ( sensorStatsList[ii].getSampleCount() ) {
                    printf("INFO Previous batch count for sensor '%s' is %d\n",
                       sensorStatsList[ii].getName(),
                       sensorStatsList[ii].getSampleCount());
                    fflush(stdout);
                    Print_batch_duration=1;
                }
                // Ugh, operator[] returns a read-only object - cannot be used here
                sensorStatsList.editItemAt(ii).sampleCount = 0;
            }
            Total_time = Total_time + afterPollMs - lastMs;
            // time spent batching the new batch
            if ( Print_batch_duration == 1 ) {
                if ( Reset_flag == 0 ) {
                    Total_time = Total_time / 1000;
                    Reset_flag=1;
                }
                printf("INFO New batch duration %d ms\n", Total_time);
                fflush(stdout);
                Total_time=0;
                Print_batch_duration = 0;
            }
//            printf("INFO New batch duration %lu ms\n", afterPollMs - lastMs);
            lastAfterPollMs = afterPollMs;
        }
    }

    while ((n = q->read(buffer, bufferSize)) > 0) {
        for (int ii = 0; ii < n; ii++) {
            /*
                ASensorEvent class definition can be found in
                   'frameworks/native/include/android/sensor.h',
                whose definition should match the one for:
                   'hardware/libhardware/include/hardware/sensors.h'.
                Sensor class definition can be found in
                    'frameworks/native/include/gui/Sensor.h'.
            */
            if (debug) {
                printf("DATALOG: version:%d handle:%d type:%d reserved0:%d "
                       "timestamp=%" PRId64 "\n",
                       buffer[ii].version, buffer[ii].sensor, buffer[ii].type,
                       buffer[ii].reserved0, buffer[ii].timestamp);
                fflush(stdout);
            }

            /* bail out when a meta event is received.
               For now only FLUSH_COMPLETE meta event are supported */
            if (buffer[ii].type == SENSOR_TYPE_META_DATA) {
                printf("METADATA Flush complete, sensor:%d, what:%d\n",
                       buffer[ii].meta_data.sensor, buffer[ii].meta_data.what);
                fflush(stdout);
                continue;
            }

            const int32_t sensorIndex =
                                    handle2index.valueFor(buffer[ii].sensor);
            if (sensorIndex == -1) {
                printf("Error: unrecognized sensor event with "
                       "version:%d sensor:%d type:%d reserved0:%d\n",
                       buffer[ii].version, buffer[ii].sensor, buffer[ii].type,
                       buffer[ii].reserved0);
                fflush(stdout);
                return -1;
            }

            float deltaT = 0.0f;
            float deltaMono = 0.0f;
            if (buffer[ii].type != SENSOR_TYPE_ADDITIONAL_INFO) {
                deltaT = float(buffer[ii].timestamp - sensorStatsList[sensorIndex].timestamp)
                        / 1000000.f;
                sensorStatsList.editItemAt(sensorIndex).timestamp = buffer[ii].timestamp;
                deltaMono = (now_ns() - buffer[ii].timestamp) / 1000000.f;
            }

            if (printOnScreen) {
                // print tag
                switch (buffer[ii].type) {
                case SENSOR_TYPE_ADDITIONAL_INFO:
                    printf("ADDINFO");
                    break;
                default:
                    printf("DATALOG");
                    break;
                }
                // sensor name
                printf(" %-30s, ", sensorStatsList[sensorIndex].getName());
                // data values
                if (buffer[ii].type == SENSOR_TYPE_ADDITIONAL_INFO) {
                    const AAdditionalInfoEvent *addinfo = &buffer[ii].additional_info;
                    switch (addinfo->type) {
                    case AINFO_BEGIN:
                        printf("begin");
                        break;
                    case AINFO_END:
                        printf("end");
                        break;
                    case AINFO_INTERNAL_TEMPERATURE:
                        printf("internal_temperature: %+10f", addinfo->data_float[0]);
                        break;
                    case AINFO_SENSOR_PLACEMENT:
                        printf("sensor_placement: %+f, %+f, %+f, %+f, %+f, %+f, %+f, %+f, %+f ; %+f, %+f, %+f",
                                addinfo->data_float[0], addinfo->data_float[1], addinfo->data_float[2],
                                addinfo->data_float[4], addinfo->data_float[5], addinfo->data_float[6],
                                addinfo->data_float[8], addinfo->data_float[9], addinfo->data_float[10],
                                addinfo->data_float[3], addinfo->data_float[7], addinfo->data_float[11]);
                        break;
                    default:
                        printf("unsupported: %+f, %+f, %+f",
                                addinfo->data_float[0], addinfo->data_float[1], addinfo->data_float[2]);
                        break;
                    }
                } else if (buffer[ii].type == SENSOR_TYPE_STEP_COUNTER) {
                    printf(
                        "%13" PRIu64 ", %13d, %13d, ",
                        buffer[ii].u64.step_counter, 0, 0);
                } else {
                    printf(
                        "%+13f, %+13f, %+13f",
                        buffer[ii].vector.v[0], buffer[ii].vector.v[1],
                        buffer[ii].vector.v[2]
                    );
                }
                if (buffer[ii].type >= SENSOR_TYPE_DEVICE_PRIVATE_BASE) {
                    /* Vendor/Customer defined, only dumps the remaining of .data (float[16]) */
                    printf(
                        ", %+13f, %+13f, %+13f, %+13f,\n%-38s, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f, %+13f,\n%-38s, %+13f, %+13f, ",
                        buffer[ii].data[3], buffer[ii].data[4], buffer[ii].data[5], buffer[ii].data[6], " ", buffer[ii].data[7],
                        buffer[ii].data[8], buffer[ii].data[9], buffer[ii].data[10], buffer[ii].data[11], buffer[ii].data[12],
                        buffer[ii].data[13], " ", buffer[ii].data[14], buffer[ii].data[15]
                    );
                }
                printf(
                    "%15" PRId64 ", ",  // timestamp
                    buffer[ii].timestamp
                );
                printf(
                    "%8.3f, ", // delta timestamp
                    deltaT
                );
                //if (!batchMode)
                printf(
                    "%8.3fms, ", // delta mono
                    deltaMono
                );
                // if any of these sensors, use estimated_accuracy, a floating
                //  point number instead of the usual integer accuracy [0,3].
                if (buffer[ii].type == SENSOR_TYPE_ROTATION_VECTOR ||
                    buffer[ii].type == SENSOR_TYPE_GAME_ROTATION_VECTOR ||
                    buffer[ii].type ==
                        SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR) {
                    printf("%+13f, ", buffer[ii].data[4]);
                } else {
                    printf("%13d, ", buffer[ii].vector.status);
                }
                // these sensors have 3 more data fields storing the bias
                //  correction in the appropriate units
                if (buffer[ii].type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED ||
                    buffer[ii].type ==
                        SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED ||
                    buffer[ii].type ==
                        SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED) {
                    printf("%+13f, %+13f, %+13f, ",      // biases, 3 values
                           buffer[ii].data[3],
                           buffer[ii].data[4],
                           buffer[ii].data[5]
                    );
                }
                /*if (buffer[ii].type == SENSOR_TYPE_EIS_GYROSCOPE) {
                        printf(
                            "%+13f",
                            buffer[ii].vector.v[3]
                        );
                }*/
                printf("\n");
                fflush(stdout);
            }

            // stats
            if (buffer[ii].type != SENSOR_TYPE_ADDITIONAL_INFO) {
                // Ugh, operator[] returns a read-only object - cannot be used here
                sensorStatsList.editItemAt(sensorIndex).sampleCount++;
            }
        }
    }
    if (n < 0 && n != -EAGAIN) {
        printf("Error reading events (%s)\n", strerror(-n));
        fflush(stdout);
    }
    return 1;
}

/**
 *  help
 */
void print_help(char *argv0)
{
    printf(
        "\n"
        "Usage : %s [OPTIONs] <sensor1,rate1> [<sensor2,rate2> ...]\n"
        "\n"
        "        where \n"
        "        <sensorN,rateN>\n"
        "           is the comma separated sensor index or name and required\n"
        "           rate of the sensor in Hz.\n"
        "           Supported rates are > 5 Hz.\n"
        "           Run with -h/--help for the reference list of sensor\n"
        "           available and their associated index.\n"
        "\n"
        "Options\n"
        "        -b / --batching MODE\n"
        "           enabled batching in mode MODE.\n"
        "           MODE can be 'timeout=MSEC' for timeout mode. In this\n"
        "           mode the data will be delivered to the clients every\n"
        "           MSEC milli seconds; in this mode the data accumulated in\n"
        "           the device fifo is allowed to overflow when the timeout\n"
        "           value exceed the storing capability of the fifo and some\n"
        "           data may get dropped.\n"
        "           The 'fifofull' MODE is to have the data delivered to\n"
        "           the clients everytime the fifo fills up and avoid\n"
        "           dropping data. This mode is not currently supported due\n"
        "           limited space in the hardware FIFO, therefore invoking\n"
        "           the batch API with this flag should return -EINVAL (-22)\n"
        "           error code.\n"
        "        -d / --debug\n"
        "           debugging output.\n"
        "        -f / --flush MODE,TIME\n"
        "           flush every TIME milliseconds either one time (MODE=o)\n"
        "           or periodically (MODE=p)\n"
        "        -h / --help\n"
        "           show this help screen and exit.\n"
        "        -l / --list\n"
        "           show the sensor list and exit.\n"
        "        -m / --metadata\n"
        "           print each sensor's metadata once.\n"
        "           This is independent from -p/--print option.\n"
        "        -p / --print\n"
        "           print sensor data on screen.\n"
        "        -r / --dryrun\n"
        "           enable batch mode dry run to test the batch API but\n"
        "           leaving the HAL state unchanged.\n"
        "        -t / --time TIME\n"
        "           run for the provided amount of milliseconds.\n"
        "        -v / --verbose\n"
        "           more verbose output (excluding sensor that is enabled\n"
        "           separately with option -p/--print).\n"
        "\n",
        argv0);
    fflush(stdout);
}

/**
 *
 */
void dumpMetadata(Sensor const* sensor)
{
    printf("Sensor '%s' metadata:\n",   sensor->getName().string());
    printf("    vendor       : %s\n",   sensor->getVendor().string());
    printf("    handle       : %d\n",   sensor->getHandle());
    printf("    type         : %d\n",   sensor->getType());
    printf("    min value    : %f\n",   sensor->getMinValue());
    printf("    max value    : %f\n",   sensor->getMaxValue());
    printf("    resolution   : %f\n",   sensor->getResolution());
    printf("    power        : %f\n",   sensor->getPowerUsage());
    printf("    min delay    : %d\n",   sensor->getMinDelay());
    printf("    min delay ns : %" PRId64 "\n", sensor->getMinDelayNs());
    printf("    max delay    : %d\n",   sensor->getMaxDelay());
    printf("    version      : %d\n",   sensor->getVersion());
    printf("    fifo reserved: %d\n",   sensor->getFifoReservedEventCount());
    printf("    fifo max     : %d\n",   sensor->getFifoMaxEventCount());
    printf("    string type  : %s\n",   (const char*)sensor->getStringType());
    printf("    flags        : %d\n",   sensor->getFlags());
    printf("    wake up      : %d\n",   sensor->isWakeUpSensor() ? 1 : 0);
    printf("    report mode  : %d\n",   sensor->getReportingMode());

    printf("\n");
    fflush(stdout);
}


/**
 *  Main loop
 */
int main(int argc, char** argv)
{
    SensorManager& mgr = SensorManager::getInstanceForPackage(String16("Sensor control Test"));

    msecs_t duration = 0L;
    msecs_t batchTimeout = 0L;

    // parse command-line arguments
    if (argc < (nReqInputPars + 1)) {
        printf(
            "\n"
            "Error: invalid number of command-line arguments.\n"
            "       You must specify at at least %d parameters.\n",
            nReqInputPars);
        fflush(stdout);
        print_help(argv[0]);
        return(10);
    }

    // load the list of available sensors in the manager
    //  used to validate the sensor's name received as input
    Sensor const* const* list;
    ssize_t count = mgr.getSensorList(&list);
    printf("\n");
    printf("Sensors' list:\n");
    fflush(stdout);
    for (int ii = 0; ii < int(count); ii++) {
        const Sensor *sensor = list[ii];
        if (mplOnly) {
            if (strncmp(sensor->getName().string(), "MPL", 3)) {
                continue;
            }
        }

        if (handle2index.indexOfKey(sensor->getHandle()) >= 0) {
            int oldIndex = handle2index.valueFor(sensor->getHandle());
            printf("Error: duplicated sensor handle '%d' "
                   "(name '%s', type '%d') - "
                   "already assigned as index '%d' "
                   "(name '%s', type '%d').\n",
                   sensor->getHandle(), sensor->getName().string(),
                   sensor->getType(), oldIndex,
                   sensorStatsList[oldIndex].getName(),
                   sensorStatsList[oldIndex].getType()
                   );
            fflush(stdout);
            return -1;
        }
        handle2index.add(sensor->getHandle(),
                         sensorStatsList.add(SensorStats(sensor)));
        printf("\t%-2d -> type %2d, name '%s'\n",
               ii + 1, sensorStatsList[ii].getType(),
               sensorStatsList[ii].getName());
        fflush(stdout);
    }
    printf("\n");
    fflush(stdout);
    numSensors = sensorStatsList.size();

    //
    // validate the command-line arguments: OPTIONS
    //
    int lastDashOption = 1;
    for (int ii = 1; ii < argc; ii++) {
        if(strcmp(argv[ii], "-h") == 0 ||
           strcmp(argv[ii], "--help") == 0) {
            print_help(argv[0]);
            return 0;

        } else if(strcmp(argv[ii], "-b") == 0 ||
                  strcmp(argv[ii], "--batching") == 0) {
            ii++;
            if (strncmp(argv[ii], "fifofull", 8) == 0) {
                batchMode = BATCH_FIFOFULL;
                batchTimeout = 1000L;
                printf("-- batching enabled in fifo full mode\n");
                fflush(stdout);
            } else if (strncmp(argv[ii], "timeout=", 8) == 0) {
                batchMode = BATCH_TIMEOUT;
                batchTimeout = strtoul(&argv[ii][8], NULL, 10);
                if (batchTimeout == 0) {
                    printf("-- batching disabled, %lu ms\n",
                           batchTimeout);
                    fflush(stdout);
                } else{
                    printf("-- batching enabled in timeout mode, %lu ms\n",
                           batchTimeout);
                    fflush(stdout);
                }
            } else {
                printf(
                    "\n"
                    "Error: invalid batch mode specification '%s'.\n"
                    //"       Please specify either 'fifofull' or \n"
                    //"       'timeout=MSEC'. See the help for details.\n"
                    "       Please specify 'timeout=MSEC'.\n"
                    "       See the help for details.\n"
                    "\n",
                    argv[ii]);
                fflush(stdout);
                    return (-1);
            }
            lastDashOption = ii;

        } else if(strcmp(argv[ii], "-d") == 0 ||
           strcmp(argv[ii], "--debug") == 0) {
            printf("-- enable debug output\n");
            fflush(stdout);
            debug = true;

        } else if(strcmp(argv[ii], "-f") == 0 ||
           strcmp(argv[ii], "--flush") == 0) {
            ii++;
            char *clFlushMode = strtok(argv[ii], ",");
            char *clFlushTime = strtok(NULL, " ");
            //printf("%s", clFlushMode);
            //printf("\n%s", clFlushTime);
            // parse flushMode
            if (strlen(clFlushMode) > 2) {
                printf("Error: flush mode format is a single character.\n"
                       "       Use 'o' for 'one time' and 'p' for 'periodic'.\n"
                       "       Provided '%s'\n", clFlushMode);
                fflush(stdout);
                return(30);
            }
            if(clFlushMode[0] != 'o' && clFlushMode[0] != 'p') {
                printf("Error: flush mode can only be one of\n"
                       "       'o' for 'one time' and 'p' for 'periodic'.\n"
                       "       Provided '%c'\n", clFlushMode[0]);
                fflush(stdout);
                return(31);
            }
            flushMode = clFlushMode[0];
            // parse flushTimeMs
            flushTimeMs = strtoul(clFlushTime, NULL, 10);
            if(flushTimeMs == 0UL) {
                printf("Error: flush time should be an integer > 0\n"
                       "       Provided '%s'\n", clFlushTime);
                fflush(stdout);
                return(32);
            }
            flushLastTimeMs = 0UL;
            printf("-- flush request, MODE=%s, TIME=%s\n",
                   clFlushMode, clFlushTime);
            fflush(stdout);

        } else if(strcmp(argv[ii], "-l") == 0 ||
           strcmp(argv[ii], "--list") == 0) {
            // the list was already printed, we bail out immediately
            return 0;

        } else if(strcmp(argv[ii], "-m") == 0 ||
                  strcmp(argv[ii], "--metadata") == 0) {
            printf("-- printing sensors' metadata on screen\n");
            fflush(stdout);
            printMetadata = true;

        } else if(strcmp(argv[ii], "-p") == 0 ||
                  strcmp(argv[ii], "--print") == 0) {
            printf("-- enabled printing on screen\n");
            fflush(stdout);
            printOnScreen = true;
            lastDashOption = ii;

        } else if(strcmp(argv[ii], "-r") == 0 ||
           strcmp(argv[ii], "--dryrun") == 0) {
            printf("-- batching, DRYRUN enabled\n");
            fflush(stdout);
            batchModeDryrun = true;

        } else if(strcmp(argv[ii], "-t") == 0 ||
                  strcmp(argv[ii], "--time") == 0) {
            ii++;
            duration = strtoul(argv[ii], NULL, 10);
            if (duration == 0) {
                printf(
                    "\n"
                    "Error: invalid time specification '%s'.\n"
                    "       Please specify an integer number > 0 for the\n"
                    "       number of milliseconds.\n"
                    "\n",
                    argv[ii]);
                fflush(stdout);
                return(33);
            }
            printf("-- polling sensors for %lu seconds\n",
                   duration / 1000L);
            fflush(stdout);
            lastDashOption = ii;

        } else if(strcmp(argv[ii], "-v") == 0 ||
                  strcmp(argv[ii], "--verbose") == 0) {
            printf("-- enabled verbose output\n");
            fflush(stdout);
            verbose = true;
            lastDashOption = ii;

        } else {
            /* break at the first not 'dash' command-line parameter found:
               the rest will be expected to be strings of sensor, rate
               selections */
            lastDashOption = ii - 1;
            break;
        }
    }

    // acquire a smart-pointer reference to the event queue: as long
    //  as this object is not released/destroyed the sensor enabled will keep
    //  staying enabled.
    sp<SensorEventQueue> q(mgr.createEventQueue());
    if (verbose) {
        printf("queue = %p\n", q.get());
        fflush(stdout);
    }

    if (lastDashOption == argc - 1) {
        int optionVisited = 0;
        if (printMetadata) {
            for (int ii = 0; ii < numSensors; ii++)
                dumpMetadata(sensorStatsList[ii].getSensor());
            optionVisited = 1;
        }
        if (optionVisited)
            return (0);

        // --- Error --- user did not provide a sensor selection, bail out
        printf(
            "\n"
            "Error: you must specify at least one sensor/rate selection\n"
            "\n");
        fflush(stdout);
        print_help(argv[0]);
        return (40);
    }

    for (int ii = lastDashOption + 1; ii < argc; ii++) {

        char *clSensorSel = strtok(argv[ii], ",");
        char *clRateSel = strtok(NULL, " ");
        if (verbose) {
            printf("sensor = '%s', rate = %s\n", clSensorSel, clRateSel);
            fflush(stdout);
        }

        //
        // validate the command-line arguments: sensor_name
        //
        String8 sensorSel(clSensorSel);
        int sensorIndex = -1;
        // -> check if it is a sensor index (Warning: system dependent)
        int tmpIndex = atoi(sensorSel.string());
        if (tmpIndex > 0) {
            if (tmpIndex <= int(sensorStatsList.size())) {
                sensorIndex = tmpIndex;
            } else {
                printf("\n");
                printf("Error: cannot find requested sensor index '%s'\n",
                       sensorSel.string());
                printf("\n");
                fflush(stdout);
                return(20);
            }
        } else {
            // -> check if it is the actual sensor name
            for (int ii = 0; ii < int(sensorStatsList.size()); ii++) {
                if (sensorSel == sensorStatsList[ii].getSensor()->getName()) {
                    sensorIndex = ii + 1;
                }
            }
            if (sensorIndex == -1) {
                printf("\n");
                printf("Error: cannot find requested sensor name '%s'\n",
                       sensorSel.string());
                printf("\n");
                fflush(stdout);
                return(21);
            }
        }
        sensorIndex--;
        Sensor const* sensor = sensorStatsList[sensorIndex].getSensor();
        if (verbose) {
            printf("sensor name/index = %s / %d\n",
                   sensor->getName().string(), sensorIndex + 1);
            fflush(stdout);
        }

        //
        // validate the command-line arguments: rate
        //
        String8 rateLiteral(clRateSel);
        int rateHz = atoi(rateLiteral.string());
        if (rateHz == 0) {
            printf("\n");
            printf("Error: invalid rate specification '%s'\n",
                   rateLiteral.string());
            printf("\n");
            fflush(stdout);
            return(30);
        }
#ifdef DEF_RESTRICT_VALID_RATES
        bool valid = false;
        for (int ii = 0;
             ii < int(sizeof(validRates) / sizeof(validRates[0])); ii++) {
            if (rateHz == validRates[ii]) {
                valid = true;
                break;
            }
        }
        if (!valid) {
            printf("\n");
            printf("Error: invalid rate specification '%d'\n", rateHz);
            printf("       Only %d, %d, %d, and %d Hz supported\n",
                   validRates[0], validRates[1], validRates[2], validRates[3]);
            printf("\n");
            fflush(stdout);
            return(31);
        }
#else
        if (rateHz < validRateRange[0] || rateHz > validRateRange[1]) {
            printf("\n");
            printf("Error: invalid rate specification '%d'\n", rateHz);
            printf("       Only rate in the [%d, %d] Hz range are allowed\n",
                   validRateRange[0], validRateRange[1]);
            printf("\n");
            fflush(stdout);
            return(31);
        }
#endif
        if (verbose) {
            printf("rate selection = %d Hz\n", rateHz);
            fflush(stdout);
        }

        //
        // print the metadata if requested
        //
        if (printMetadata)
            dumpMetadata(sensor);

        //
        // enable the sensor
        //
        nsecs_t delayNs = nsecs_t(1000000000LL / rateHz);
        printf("## enabling '%s' @ %d Hz (%lld ms)\n",
               sensor->getName().string(), rateHz, delayNs / 1000000LL);
        fflush(stdout);
        //q->enableSensorBatch(sensor->getHandle(), delayNs / 1000, 0, false);

        //
        // enable batching
        //
        if (batchMode) {
            int batchFlags = 0;
            int result = 0;
            // batch API 'flags' parameter (2nd parameter) is a bitmask defined
            // as follows:
            //      SENSORS_BATCH_DRY_RUN bit             = 0x01
            //      SENSORS_BATCH_WAKE_UPON_FIFO_FULL bit = 0x02

            // dry-run mode
            if (batchModeDryrun)
                batchFlags = SENSORS_BATCH_DRY_RUN;

            switch (batchMode) {
            case BATCH_FIFOFULL:
                batchFlags |= SENSORS_BATCH_WAKE_UPON_FIFO_FULL;
                batchTimeout = 100000UL;    // 100 s
                /*result = q->enableBatch(sensor, batchFlags, delayNs,
                               (unsigned long long)batchTimeout * 1000000ULL);*/
                result = q->enableSensor(sensor->getHandle(), delayNs/1000,
                               (unsigned long long)batchTimeout * 1000ULL, batchFlags);
                if (verbose || result) {
                    printf("enableBatch(sensor#=%d, flags=%02X, "
                           "period=%" PRIu64 " ms, timeout=%lu ms) = %d\n",
                           sensorIndex, batchFlags, delayNs, batchTimeout,
                           result);
                    fflush(stdout);
                }
                if (result < 0) {
                    //batchMode = BATCH_LEAVE_UNCHANGED;
                    if (verbose) {
                        printf("batch mode is turned off\n");
                        fflush(stdout);
                    }
                }
                break;

            case BATCH_TIMEOUT:
                /*result = q->enableBatch(sensor, batchFlags, delayNs,
                               (unsigned long long)batchTimeout * 1000000ULL);*/
                result = q->enableSensor(sensor->getHandle(), delayNs/1000,
                               (unsigned long long)batchTimeout * 1000ULL, batchFlags);
                if (verbose || result) {
                    printf("enableBatch(sensor#=%d, flags=%02X, "
                           "period=%" PRIu64 " ms, timeout=%lu ms) = %d\n",
                           sensorIndex, batchFlags, delayNs, batchTimeout,
                           result);
                    fflush(stdout);
                }
                if (result < 0) {
                    //batchMode = BATCH_LEAVE_UNCHANGED;
                    if (verbose) {
                        printf("batch mode is turned off\n");
                        fflush(stdout);
                    }
                }
                break;

            default:
                // nothing to do, possibly batch request malformed?
                break;
            }
        } else {
            q->enableSensor(sensor->getHandle(), delayNs / 1000, 0, false);
        }

        // record info regarding the sensor
        // Ugh, operator[] returns a read-only object - cannot be used here
        sensorStatsList.editItemAt(sensorIndex).setRate(rateHz);
        SensorStats& sensorStats = sensorStatsList.editItemAt(sensorIndex);
        sensorStats.setRate(rateHz);
        sensorStats.setEnable(true);

        if (firstSensorIndex == -1)
            firstSensorIndex = sensorIndex;
    }

    //
    // process the sensor data
    //

    // initialize the timestamp
    nsecs_t nowNs = now_ns();
    for (int ii = 0; ii < int(sensorStatsList.size()); ii++) {
        sensorStatsList.editItemAt(ii).timestamp = nowNs;
    }
    msecs_t nowMs = (long)(nowNs / 1000000LL);
    msecs_t durationStart = nowMs;
    msecs_t durationKa = nowMs;
    flushLastTimeMs = nowMs;
    beforePollMs = nowMs;

    sp<Looper> loop = new Looper(false);
    loop->addFd(q->getFd(), 0, ALOOPER_EVENT_INPUT, receiver, q.get());

    /* go until interrupted (SIGKILL/CTRL+c) or time has expired */
    while ((nowMs = now_ms()), duration == 0 ||
           (nowMs - durationStart) < duration) {

        // service the flush request between batches - it's not going to
        //  observe precise timing
        // TODO make flush be precisely at the time the user requested it and
        //      not just between groups of samples
        if (flushMode != ' '){
            if((nowMs - flushLastTimeMs) >= flushTimeMs) {
                printf("Flushing sensor '%s' at %lu\n",
                       sensorStatsList[firstSensorIndex].getName(), nowMs - durationStart);
                fflush(stdout);
                //q->enableFlush(sensorStatsList[firstSensorIndex].getSensor());
                q->flush();
                flushLastTimeMs = nowMs;
                // flush one time only and disable it
                if (flushMode == 'o')
                    flushMode = ' ';
            }
        }

        int32_t ret = loop->pollOnce(200);
        switch (ret) {
        case ALOOPER_POLL_WAKE:
            break;
        case ALOOPER_POLL_CALLBACK:
            // this is for the next poll -
            //  we acquire the timestamp after exiting the callback
            beforePollMs = now_ms();
            break;
        case ALOOPER_POLL_TIMEOUT:
            break;
        case ALOOPER_POLL_ERROR:
            break;
        default:
            printf("ugh? poll returned %d\n", ret);
            fflush(stdout);
            break;
        }
        if (duration > 0 && !printOnScreen) {
            if ((nowMs - durationKa) > 1000UL) { // every 1 second
                printf("time elapsed: %lu ms\n", beforePollMs - durationStart);
                fflush(stdout);
                durationKa = beforePollMs;
            }
        }
    }

    return 0;
}
