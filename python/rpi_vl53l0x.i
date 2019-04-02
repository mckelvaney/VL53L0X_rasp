// SWIG interface file to define rpi_vl53l0x library python wrapper.
// Author: Peter Yang (turmary@126.com)

// Define module name rpi_vl53l0x.  This will actually be imported under
// the name _rpi_vl53l0x following the SWIG & Python conventions.
%module rpi_vl53l0x

// Include standard SWIG types & array support for support of uint32_t
// parameters and arrays.
%include "stdint.i"
%include "carrays.i"

%array_class(uint8_t, uint8_t);
%array_class(uint32_t, uint32_t);

// Declare functions which will be exported as anything in the vl53l0x_xxx headers.
%{
#include "VL53L0X_1.0.2/Api/core/inc/vl53l0x_def.h"
#include "VL53L0X_1.0.2/Api/core/inc/vl53l0x_api.h"
#include "platform/inc/vl53l0x_platform.h"

%}

// Process vl53l0x_api.h header and export all included functions.
%include "VL53L0X_1.0.2/Api/core/inc/vl53l0x_api.h"
%include "VL53L0X_1.0.2/Api/core/inc/vl53l0x_def.h"
%include "platform/inc/vl53l0x_platform.h"
