#ifndef __arduino_h__
#define __arduino_h__

#include <math.h>
#include "Print.h"
#include "Stream.h"

#include "SerialClass.h" // Arduino style Serial class

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

#define PI M_PI
#define RAD_TO_DEG (180.0 / M_PI)

#define delay(ms) HAL_Delay(ms)
#define delayMicroseconds(us) HAL_Delay(us*1000)
#define millis() (HAL_GetTick() / 1)
#define micros() HAL_GetTick()*1000

#endif
