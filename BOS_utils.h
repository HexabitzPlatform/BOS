#ifndef BOS_UTILS_H
#define BOS_UTILS_H

#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>

#define HIGH 										0x1
#define LOW  										0x0

#define PI 											3.1415926535897932384626433832795
#define HALF_PI 								1.5707963267948966192313216916398
#define TWO_PI 									6.283185307179586476925286766559

#define DEG_TO_RAD 							0.017453292519943295769236907684886
#define RAD_TO_DEG 							57.295779513082320876798154814105

#define EULER 									2.718281828459045235360287471352

#define celsiusToFahrenheit(c)	(((c * 9.0) / 5) + 32)
#define fahrenheitToCelsius(f)	(((f - 32) * 5.0) / 9.0)

#define min(x,y) 								((x)>(y) ? (y) : (x))
#define max(x,y) 								((x)>(y) ? (x) : (y))

#define absolute(x,y)						((x) >= 0 ? (x) : -(x))

#define constrain(a,low,high)		(((a) > (high)) ? (high) : (((a) < (low)) ? (low) : (a)))

#define radians(deg) 						((deg)*DEG_TO_RAD)
#define degrees(rad) 						((rad)*RAD_TO_DEG)

#define lowByte(w) 							((uint8_t)((w) & 0xff))
#define highByte(w) 						((uint8_t)((w) >> 8))

#define concatBytes(low,high)		(low | (((uint16_t)high) << 8))

#define bitRead(value,bit) 							(((value) >> (bit)) & 0x01)
#define bitSet(value,bit) 							((value) |= (1UL << (bit)))
#define bitClear(value,bit) 						(value) &= ~(1UL << (bit)))
#define bitWrite(value,bit,bitvalue) 		(bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define square(x) 											((x)*(x))

#define iseven(x) 											((x) & 0x01)
#define isodd(x)												(!iseven(x))

/* Double Negation Normalize Boolean values i {0,1} */
#define bit_at(a,i)											(!!((a)[(unsigned)(i) >> 3] & (1 << ((unsigned)(i) & 0x07))))
	
#define array_size(a)										(sizeof(a)/sizeof(a))

#define isblank(c) 											((c == ' ') || (c == '\t') || (c == '\v'))
#define isbacksplash(c) 								(c == '/')
#define ishex(c)           							(isxdigit(c))



#endif
