/*
    BitzOS (BOS) V0.1.4 - Copyright (C) 2019 Hexabitz
    All rights reserved
		
    File Name     : BOS_MsgCodes.h
    Description   : Header file for for BOS communication message codes. 
*/
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BOS_MSGCODES_H
#define BOS_MSGCODES_H



/* -----------------------------------------------------------------------
	|												BOS Message Codes	 														 	|
   ----------------------------------------------------------------------- 
*/
#define	CODE_UNKNOWN_MESSAGE							0
#define	CODE_PING													1
#define	CODE_PING_RESPONSE								2
#define	CODE_IND_ON												3
#define	CODE_IND_OFF											4
#define	CODE_IND_TOGGLE										5

#define	CODE_HI														10
#define	CODE_HI_RESPONSE									11
#define	CODE_EXPLORE_ADJ									12
#define	CODE_EXPLORE_ADJ_RESPONSE					13
#define	CODE_PORT_DIRECTION								14
#define	CODE_BAUDRATE											15
#define	CODE_MODULE_ID										16
#define	CODE_TOPOLOGY											17
#define	CODE_BROADCAST_PLAN								18
#define	CODE_READ_PORT_DIR								19
#define	CODE_READ_PORT_DIR_RESPONSE				20
#define	CODE_EXP_EEPROM	 									21
#define	CODE_DEF_ARRAY	 									22
#define	CODE_CLI_COMMAND 									23
#define	CODE_CLI_RESPONSE  								24
#define	CODE_UPDATE  											25
#define	CODE_UPDATE_VIA_PORT  						26
#define	CODE_DMA_CHANNEL  								27
#define	CODE_DMA_SCAST_STREAM  						28

#define	CODE_READ_REMOTE  								30
#define	CODE_READ_REMOTE_RESPONSE  				31
#define	CODE_WRITE_REMOTE  								32
#define	CODE_WRITE_REMOTE_RESPONSE  			33
#define	CODE_WRITE_REMOTE_FORCE						34

#define	CODE_PORT_FORWARD     						35


/* -----------------------------------------------------------------------
	|											Module Message Codes	 														|
   ----------------------------------------------------------------------- 
*/

// H01R0x
#define	CODE_H01R0_ON												100
#define	CODE_H01R0_OFF											101
#define	CODE_H01R0_TOGGLE										102
#define	CODE_H01R0_COLOR										103
#define	CODE_H01R0_PULSE										104
#define	CODE_H01R0_SWEEP										105
#define	CODE_H01R0_DIM											106

// H07R3x
#define CODE_H07R3_PLAY_SINE								350
#define CODE_H07R3_PLAY_WAVE								351
#define CODE_H07R3_PLAY_Tone								352

// H08R6x
#define CODE_H08R6_GET_INFO                 400
#define CODE_H08R6_SAMPLE                   401
#define CODE_H08R6_STREAM_PORT              402
#define CODE_H08R6_STREAM_MEM               403
#define CODE_H08R6_RESULT_MEASUREMENT       404
#define CODE_H08R6_STOP_RANGING             405
#define CODE_H08R6_SET_UNIT                 406
#define CODE_H08R6_GET_UNIT                 407
#define CODE_H08R6_RESPOND_GET_UNIT         408
#define CODE_H08R6_MAX_RANGE                409
#define CODE_H08R6_MIN_RANGE                410
#define CODE_H08R6_TIMEOUT                	411


// H0BR4x
#define CODE_H0BR4_GET_GYRO                 550
#define CODE_H0BR4_GET_ACC                  551
#define CODE_H0BR4_GET_MAG		              552
#define CODE_H0BR4_GET_TEMP		              553
#define CODE_H0BR4_RESULT_GYRO              554
#define CODE_H0BR4_RESULT_ACC               555
#define CODE_H0BR4_RESULT_MAG		 	          556
#define CODE_H0BR4_RESULT_TEMP		          557
#define CODE_H0BR4_STREAM_GYRO							558
#define CODE_H0BR4_STREAM_ACC								559
#define CODE_H0BR4_STREAM_MAG								560
#define CODE_H0BR4_STREAM_TEMP							561
#define CODE_H0BR4_STREAM_STOP							562

// H0FR6x
#define	CODE_H0FR6_ON												750
#define	CODE_H0FR6_OFF											751
#define	CODE_H0FR6_TOGGLE										752
#define	CODE_H0FR6_PWM											753

// H1BR6x




#endif /* BOS_MSGCODES_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
