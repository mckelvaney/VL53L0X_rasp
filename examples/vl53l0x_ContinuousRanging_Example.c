#define VL53L0X_DBG	0
#include <stdlib.h>
#include <stdio.h>
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"


#define VERSION_REQUIRED_MAJOR 1
#define VERSION_REQUIRED_MINOR 0
#define VERSION_REQUIRED_BUILD 2


void print_pal_error(VL53L0X_Error st){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(st, buf);
    printf("API st: %i : %s\n", st, buf);
}


void print_range_status(VL53L0X_RangingMeasurementData_t* measured) {
    char buf[VL53L0X_MAX_STRING_LENGTH];
    uint8_t RangeStatus;

    /*
     * New Range st: data is valid when measured->RangeStatus = 0
     */

    RangeStatus = measured->RangeStatus;

    VL53L0X_GetRangeStatusString(RangeStatus, buf);
    printf("Range st: %i : %s\n", RangeStatus, buf);
}


VL53L0X_Error WaitMeasurementDataReady(VL53L0X_Dev_t* Dev) {
    VL53L0X_Error st    = VL53L0X_ERROR_NONE;
    uint8_t NewDatReady = 0;
    uint32_t loop_nr    = 0;

    // Wait until it finished, use timeout to avoid deadlock
    while (++loop_nr <= VL53L0X_DEFAULT_MAX_LOOP) {
        st = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
        if (NewDatReady == 0x01 || st != VL53L0X_ERROR_NONE) {
            break;
        }
        VL53L0X_PollingDelay(Dev);
    }
    if (loop_nr > VL53L0X_DEFAULT_MAX_LOOP)
        st = VL53L0X_ERROR_TIME_OUT;
    return st;
}


VL53L0X_Error WaitStopCompleted(VL53L0X_Dev_t* Dev) {
    VL53L0X_Error st       = VL53L0X_ERROR_TIME_OUT;
    uint32_t StopCompleted = 0;
    uint32_t loop_nr       = 0;

    // Wait until it finished, use timeout to avoid deadlock
    while (++loop_nr <= VL53L0X_DEFAULT_MAX_LOOP) {
        st = VL53L0X_GetStopCompletedStatus(Dev, &StopCompleted);
        if (StopCompleted == 0x00 || st != VL53L0X_ERROR_NONE) {
            st = VL53L0X_ERROR_NONE;
            break;
        }
        VL53L0X_PollingDelay(Dev);
    }
    if (loop_nr > VL53L0X_DEFAULT_MAX_LOOP)
        st = VL53L0X_ERROR_TIME_OUT;
    return st;
}


VL53L0X_Error rangingTest(VL53L0X_Dev_t *dev)
{
    VL53L0X_RangingMeasurementData_t   measured[1];
    VL53L0X_Error st = VL53L0X_ERROR_NONE;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    // Device Initialization
    if (st == VL53L0X_ERROR_NONE) {
        // StaticInit will set interrupt by default
        st = VL53L0X_StaticInit(dev);
        #if VL53L0X_DBG
        printf ("Call of VL53L0X_StaticInit\n");
        print_pal_error(st);
        #endif
    }
    if (st == VL53L0X_ERROR_NONE) {
        st = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);
	#if VL53L0X_DBG
        printf ("Call of VL53L0X_PerformRefCalibration\n");
        print_pal_error(st);
        #endif
    }

    if (st == VL53L0X_ERROR_NONE) {
        st = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);
        #if VL53L0X_DBG
        printf ("Call of VL53L0X_PerformRefSpadManagement\n");
        print_pal_error(st);
        #endif
    }

    if (st == VL53L0X_ERROR_NONE) {
        // Setup in continuous ranging mode
        st = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
        #if VL53L0X_DBG
        printf ("Call of VL53L0X_SetDeviceMode\n");
        print_pal_error(st);
        #endif
    }

    if (st == VL53L0X_ERROR_NONE) {
        st = VL53L0X_StartMeasurement(dev);
        #if VL53L0X_DBG
        printf ("Call of VL53L0X_StartMeasurement\n");
        print_pal_error(st);
        #endif
    }

    if (st == VL53L0X_ERROR_NONE) {
        #define NR_MEASURE 5000
        uint16_t* pResults  = (uint16_t*)malloc(sizeof(uint16_t) * NR_MEASURE);
        uint32_t i;

        for(i = 0; i < NR_MEASURE; i++) {
            st = WaitMeasurementDataReady(dev);
            if (st != VL53L0X_ERROR_NONE) {
                break;
            }

            st = VL53L0X_GetRangingMeasurementData(dev, measured);

            *(pResults + i) = measured->RangeMilliMeter;
            printf("In loop i %d: %d\n", i, measured->RangeMilliMeter);

            // Clear the interrupt
            VL53L0X_ClearInterruptMask(dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
            // VL53L0X_PollingDelay(dev);
        }

        if (st == VL53L0X_ERROR_NONE) {
            for (i = 0; i < NR_MEASURE; i++) {
                printf("i %d: %d\n", i, *(pResults + i));
            }
        }
        free(pResults);
    }

    if (st == VL53L0X_ERROR_NONE) {
        #if VL53L0X_DBG
        printf ("Call of VL53L0X_StopMeasurement\n");
        #endif
        st = VL53L0X_StopMeasurement(dev);
    }

    if (st == VL53L0X_ERROR_NONE) {
        #if VL53L0X_DBG
        printf ("Wait Stop to be competed\n");
        #endif
        st = WaitStopCompleted(dev);
    }

    if (st == VL53L0X_ERROR_NONE)
        st = VL53L0X_ClearInterruptMask(dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);

    return st;
}

int main(int argc, char **argv)
{
    VL53L0X_Error        st = VL53L0X_ERROR_NONE;
    VL53L0X_Dev_t        dev[1];
    VL53L0X_Version_t    ver[1];
    VL53L0X_DeviceInfo_t dev_info;

    printf ("VL53L0X PAL Continuous Ranging example\n\n");

    // Initialize Comms
    dev->I2cDevAddr = 0x29;

    // choose between i2c-0 and i2c-1; On the raspberry pi zero, i2c-1 are pins 2 and 3
    dev->fd = VL53L0X_i2c_init("/dev/i2c-1", dev->I2cDevAddr);
    if (dev->fd < 0) {
        st = VL53L0X_ERROR_CONTROL_INTERFACE;
        printf ("Failed to init interface\n");
        return 1;
    }

    /*
     *  Get the version of the VL53L0X API running in the firmware
     */
    if (st == VL53L0X_ERROR_NONE) {
        int st2 = VL53L0X_GetVersion(ver);
        if (st2 != 0)
            st = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    /*
     *  Verify the version of the VL53L0X API running in the firmware
     */
    if (st == VL53L0X_ERROR_NONE) {
        if (ver->major != VERSION_REQUIRED_MAJOR ||
            ver->minor != VERSION_REQUIRED_MINOR ||
            ver->build != VERSION_REQUIRED_MINOR )
        {
            printf("VL53L0X API Version Error: Your firmware has %d.%d.%d (revision %d).",
                ver->major, ver->minor, ver->build, ver->revision);
            printf("This example requires %d.%d.%d.\n",
                VERSION_REQUIRED_MAJOR, VERSION_REQUIRED_MINOR, VERSION_REQUIRED_BUILD);
        }
    }

    // End of implementation specific
    if (st == VL53L0X_ERROR_NONE) {
        // Data initialization
        st = VL53L0X_DataInit(dev);
        #if VL53L0X_DBG
        printf ("Call of VL53L0X_DataInit\n");
        print_pal_error(st);
        #endif
    }

    if (st == VL53L0X_ERROR_NONE) {
        st = VL53L0X_GetDeviceInfo(dev, &dev_info);
    }
    if (st == VL53L0X_ERROR_NONE) {
        printf("VL53L0X_GetDeviceInfo:\n");
        printf("Device Name : %s\n",          dev_info.Name);
        printf("Device Type : %s\n",          dev_info.Type);
        printf("Device ID : %s\n",            dev_info.ProductId);
        printf("ProductRevisionMajor : %d\n", dev_info.ProductRevisionMajor);
        printf("ProductRevisionMinor : %d\n", dev_info.ProductRevisionMinor);

        if (dev_info.ProductRevisionMinor != 1 && dev_info.ProductRevisionMinor != 1) {
            printf("Error expected cut 1.1 but found cut %d.%d\n",
                    dev_info.ProductRevisionMajor, dev_info.ProductRevisionMinor);
            st = VL53L0X_ERROR_NOT_SUPPORTED;
        }
    }

    if (st == VL53L0X_ERROR_NONE) {
        st = rangingTest(dev);
    }

    // Implementation specific

    /*
     *  Disconnect comms - part of VL53L0X_platform.c
     */

    #if VL53L0X_DBG
    printf ("Close Comms\n");
    #endif
    VL53L0X_i2c_close();

    print_pal_error(st);

    return (0);
}
