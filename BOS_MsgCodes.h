/*
    BitzOS (BOS) V0.2.1 - Copyright (C) 2020-2021 Hexabitz
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
#define	CODE_UNKNOWN_MESSAGE					0
#define	CODE_PING								      1
#define	CODE_PING_RESPONSE						2
#define	CODE_IND_ON							    	3
#define	CODE_IND_OFF							    4
#define	CODE_IND_TOGGLE						   	5

#define	CODE_HI							        		10
#define	CODE_HI_RESPONSE						    11
#define	CODE_EXPLORE_ADJ					    	12
#define	CODE_EXPLORE_ADJ_RESPONSE				13
#define	CODE_PORT_DIRECTION						  14
#define	CODE_BAUDRATE						      	15
#define	CODE_MODULE_ID							    16
#define	CODE_TOPOLOGY						      	17
#define	CODE_BROADCAST_PLAN						  18
#define	CODE_READ_PORT_DIR					  	19
#define	CODE_READ_PORT_DIR_RESPONSE			20
#define	CODE_EXP_EEPROM	 						    21
#define	CODE_DEF_ARRAY	 					    	22
#define	CODE_CLI_COMMAND 						    23
#define	CODE_CLI_RESPONSE  						  24
#define	CODE_UPDATE  					      		25
#define	CODE_UPDATE_VIA_PORT  					26
#define	CODE_DMA_CHANNEL  					  	27
#define	CODE_DMA_SCAST_STREAM  					28

#define	CODE_READ_REMOTE  						  30
#define	CODE_READ_REMOTE_RESPONSE  			31
#define	CODE_WRITE_REMOTE  						  32
#define	CODE_WRITE_REMOTE_RESPONSE  		33
#define	CODE_WRITE_REMOTE_FORCE					34

#define	CODE_PORT_FORWARD     					35


/* -----------------------------------------------------------------------
	|											User Message Codes (70-99)	 											|
   ----------------------------------------------------------------------- 
*/


/* -----------------------------------------------------------------------
	|											Module Message Codes	 														|
   ----------------------------------------------------------------------- 
*/
// Reserve 50 messages for each PN based on its decimal value

// H01R0x
#define	CODE_H01R0_ON						  100
#define	CODE_H01R0_OFF						101
#define	CODE_H01R0_TOGGLE					102
#define	CODE_H01R0_COLOR					103
#define	CODE_H01R0_PULSE					104
#define	CODE_H01R0_SWEEP					105
#define	CODE_H01R0_DIM						106

// H07R3x
#define CODE_H07R3_PLAY_SINE								350
#define CODE_H07R3_PLAY_WAVE								351
#define CODE_H07R3_PLAY_TUNE								352
#define CODE_H07R3_SCAN_WAVE_RESPONSE				353

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
#define CODE_H0BR4_GET_GYRO           550
#define CODE_H0BR4_GET_ACC            551
#define CODE_H0BR4_GET_MAG		        552
#define CODE_H0BR4_GET_TEMP		        553
#define CODE_H0BR4_RESULT_GYRO        554
#define CODE_H0BR4_RESULT_ACC         555
#define CODE_H0BR4_RESULT_MAG		 	    556
#define CODE_H0BR4_RESULT_TEMP		    557
#define CODE_H0BR4_STREAM_GYRO				558
#define CODE_H0BR4_STREAM_ACC				  559
#define CODE_H0BR4_STREAM_MAG			  	560
#define CODE_H0BR4_STREAM_TEMP				561
#define CODE_H0BR4_STREAM_STOP				562

// H0FR6x
#define	CODE_H0FR6_ON						  750
#define	CODE_H0FR6_OFF						751
#define	CODE_H0FR6_TOGGLE					752
#define	CODE_H0FR6_PWM						753

// H1BR6x
#define CODE_H1BR6_READ_WAVE            		1350
#define CODE_H1BR6_SCAN_WAVE								1351

// H23R0x
#define CODE_H23Rx_GET_INFO                 1700
#define CODE_H23Rx_DOWNLOAD_SCRIPT_OTA      1701
#define CODE_H23Rx_DOWNLOAD_SCRIPT_UART     1702
#define CODE_H23Rx_RUN_AUTORUN_SCRIPT       1703
#define CODE_H23Rx_VSP_COMMAND_MODE         1704
#define CODE_H23Rx_VSP_BRIDGE_MODE          1705
#define CODE_H23Rx_SPP_MODE                 1706
#define CODE_H23Rx_LED_STATUS_ON            1707
#define CODE_H23Rx_LED_STATUS_OFF           1708
#define CODE_H23Rx_BTC_DEL_ALL_DATA_SEG     1709
#define CODE_H23Rx_EVBTC_SPPCONN            1710
#define CODE_H23Rx_EVBTC_SPPDISCON          1711
#define CODE_H23Rx_EVBTC_PAIR_REQUEST       1712
#define CODE_H23Rx_EVBTC_PIN_REQUEST        1713
#define CODE_H23Rx_EVBTC_PAIR_RESULT        1714
#define CODE_H23Rx_EVBTC_AUTHREQ            1715
#define CODE_H23Rx_EVBTC_PASSKEY            1716
#define CODE_H23Rx_SHOW_DEBUG_INFO          1717
#define CODE_H23Rx_SCAN_INQUIRE             1718
#define CODE_H23Rx_SCAN_RESPOND             1719
#define CODE_H23Rx_SCAN_RESPOND_ERR         1720
#define CODE_H23Rx_CONNECT_INQUIRE          1721
#define CODE_H23Rx_CONNECT_RESPOND          1722
#define CODE_H23Rx_FINISHED_SCAN            1723
#define CODE_H23Rx_UNKNOWN_CMD              1799

// H26R0x
#define CODE_H26R0_SET_RATE                 1900
#define CODE_H26R0_STREAM_PORT_GRAM         1901
#define CODE_H26R0_STREAM_PORT_KGRAM        1902
#define CODE_H26R0_STREAM_PORT_OUNCE        1903
#define CODE_H26R0_STREAM_PORT_POUND        1904
#define CODE_H26R0_STOP                     1905
#define CODE_H26R0_SAMPLE_GRAM              1906
#define CODE_H26R0_SAMPLE_KGRAM             1907
#define CODE_H26R0_SAMPLE_OUNCE             1908
#define CODE_H26R0_SAMPLE_POUND             1909
#define CODE_H26R0_ZEROCAL                	1910
#define CODE_H26R0_STREAM_RAW               1911
#define CODE_H26R0_SAMPLE_RAW               1912
#define CODE_H26R0_STREAM_FORMAT            1913


#endif /* BOS_MSGCODES_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
