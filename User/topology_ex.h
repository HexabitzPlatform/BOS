/*
    BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
    All rights reserved

    File Name     : topology_ex.h
    Description   : Array topology definition.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __topology_ex_H
#define __topology_ex_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
	 

#define _N	4					// Number of array modules

// Array modules
#define _mod1	1<<3
#define _mod2	2<<3
#define _mod3	3<<3
#define _mod4	4<<3


// Topology
static uint16_t array[_N][7] = {
{ _H01R0, 0, 0, 0, _mod2|P3, _mod3|P4, 0},		// Module 1
{ _H01R0, 0, _mod3|P5, _mod1|P4, 0, 0, 0},		// Module 2
{ _H01R0, _mod4|P4, 0, 0, _mod1|P5, _mod2|P2, 0},		// Module 3
{ _H01R0, 0, 0, 0, _mod3|P1, 0, 0}		// Module 4
};

// Configurations for duplex serial ports
#if ( _module == 1 )
	#define	H01R0	1
	#define	_P1pol_normal	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_normal	1	
	#define	_P5pol_normal	1
	#define	_P6pol_normal	1
#endif
#if ( _module == 2 )
	#define	H01R0	1
	#define	_P1pol_normal	1
	#define	_P2pol_normal	1
	#define	_P3pol_reversed	1
	#define	_P4pol_normal	1	
	#define	_P5pol_normal	1
	#define	_P6pol_normal	1
#endif
#if ( _module == 3 )
	#define	H01R0	1
	#define	_P1pol_normal	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_reversed	1	
	#define	_P5pol_reversed	1
	#define	_P6pol_normal	1
#endif
#if ( _module == 4 )
	#define	H01R0	1
	#define	_P1pol_normal	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_reversed	1	
	#define	_P5pol_normal	1
	#define	_P6pol_normal	1
#endif


#ifdef __cplusplus
}
#endif
#endif /*__ topology_ex_H */


/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
