/*
 BitzOS (BOS) V0.3.2 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : FreeRtosConfig.h
 Description   : Header file provides configuration for FreeRtos.

 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

/* Section where include file can be added */

/* Ensure stdint is only used by the compiler, and not the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
#include <stdint.h>
extern uint32_t SystemCoreClock;
#endif

#define configUSE_PREEMPTION                     1
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configCPU_CLOCK_HZ                       ( SystemCoreClock )
#define configTICK_RATE_HZ                       ((TickType_t)1000)
#define configMAX_PRIORITIES                     ( 7 )
//#define configMINIMAL_STACK_SIZE                 ((uint16_t)160)
//#define configTOTAL_HEAP_SIZE                    ((size_t)19000)
#ifdef STM32G0B1xx
#define configMINIMAL_STACK_SIZE                 ((uint16_t)2500)
#else
#define configMINIMAL_STACK_SIZE                 ((uint16_t)150)
#endif
#ifdef STM32G0B1xx
#define configTOTAL_HEAP_SIZE                    ((size_t)90000)
#else
#define configTOTAL_HEAP_SIZE                    ((size_t)16200)
#endif
#define configMAX_TASK_NAME_LEN                  ( 13 )
#define configUSE_TRACE_FACILITY                 1
#define configUSE_16_BIT_TICKS                   0
#define configUSE_MUTEXES                        1
#define configQUEUE_REGISTRY_SIZE                8
#define configUSE_RECURSIVE_MUTEXES              1
#define configUSE_COUNTING_SEMAPHORES            1

/*BOS Tasks stack size: */
#ifdef STM32G0B1xx
#define CLI_TASK_STACK_SIZE						((uint16_t)500*2)
#define BACKEND_TASK_STACK_SIZE					((uint16_t)500*2)
#define USER_TASK_STACK_SIZE					((uint16_t)500*2)
#define DEFAULT_TASK_STACK_SIZE					((uint16_t)500*2)
#define PORT_TASK_STACK_SIZE					((uint16_t)500*2)
#else
#define CLI_TASK_STACK_SIZE						((uint16_t)140*2)
#define BACKEND_TASK_STACK_SIZE					((uint16_t)120*2)
#define USER_TASK_STACK_SIZE					((uint16_t)120*2)
#define DEFAULT_TASK_STACK_SIZE					((uint16_t)120*2)
#define PORT_TASK_STACK_SIZE					((uint16_t)120*2)
#endif

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                    0
#define configMAX_CO_ROUTINE_PRIORITIES          ( 2 )

/* Set the following definitions to 1 to include the API function, or zero
 to exclude the API function. */
#define INCLUDE_vTaskPrioritySet            1
#define INCLUDE_uxTaskPriorityGet           1
#define INCLUDE_vTaskDelete                 1
#define INCLUDE_vTaskCleanUpResources       0
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_vTaskDelayUntil             0
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_xTaskGetSchedulerState      1
#define INCLUDE_xTimerPendFunctionCall      1

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
 /* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
 #define configPRIO_BITS         __NVIC_PRIO_BITS
#else
#define configPRIO_BITS         2
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
 function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   3

/* The highest interrupt priority that can be used by any interrupt service
 routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
 INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
 PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 1

/* Interrupt priorities used by the kernel port layer itself.  These are generic
 to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
 See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* Dimensions a buffer that can be used by the FreeRTOS+CLI command
 interpreter.  Set this value to 1 to save RAM if FreeRTOS+CLI does not supply
 the output buffer.  See the FreeRTOS+CLI documentation for more information:
 http://www.FreeRTOS.org/FreeRTOS-Plus/FreeRTOS_Plus_CLI/ */
#define configCOMMAND_INT_MAX_OUTPUT_SIZE			612

/* Hook function related definitions. */
#define configUSE_MALLOC_FAILED_HOOK	1
#define configCHECK_FOR_STACK_OVERFLOW	2
//#define configCHECK_FOR_STACK_OVERFLOW	0

/* Normal assert() semantics without relying on the provision of an assert.h
 header file. */
//#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
 standard names. */
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler

/* IMPORTANT: This define MUST be commented when used with STM32Cube firmware, 
 to prevent overwriting SysTick_Handler defined within STM32Cube HAL */
/* #define xPortSysTickHandler SysTick_Handler */

/* Section where parameter definitions can be added (for instance, to override default ones in FreeRTOS.h) */

/* Run time stats gathering definitions. */
void vMainConfigureTimerForRunTimeStats(void);
uint32_t ulMainGetRunTimeCounterValue(void);
#define configGENERATE_RUN_TIME_STATS	1
#define configUSE_STATS_FORMATTING_FUNCTIONS		 1 
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() vMainConfigureTimerForRunTimeStats()
#define portGET_RUN_TIME_COUNTER_VALUE() ulMainGetRunTimeCounterValue()

/* No MPU in Cortex-M0 */
//#define portUSING_MPU_WRAPPERS 0
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* Software Timers */
#define configUSE_TIMERS 	1
#define	configTIMER_TASK_PRIORITY		3			// (osPriorityNormal-osPriorityIdle)	
#define configTIMER_QUEUE_LENGTH		4
#define configTIMER_TASK_STACK_DEPTH		configMINIMAL_STACK_SIZE	

/* USe uxTaskGetStackHighWaterMark */
#define INCLUDE_uxTaskGetStackHighWaterMark 	1

/* Place holder for calls to ioctl that don't use the value parameter. */
#define cmdPARAMTER_NOT_USED		( ( void * ) 0 )

/* Block times of 50 and 500milliseconds, specified in ticks. */
#define cmd50ms						( 50UL / portTICK_RATE_MS ) 
#define cmd500ms					( 500UL / portTICK_RATE_MS ) 

#endif /* FREERTOS_CONFIG_H */

