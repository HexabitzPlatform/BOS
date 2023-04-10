/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
 All rights reserved

 File Name     : BOS_MsgCodes.h
 Description   : Header file for for BOS communication message codes.

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BOS_MSGCODES_H
#define BOS_MSGCODES_H

/* -----------------------------------------------------------------------
 |												BOS Message Codes	      |
   -----------------------------------------------------------------------
 */
#define	CODE_UNKNOWN_MESSAGE				  0
#define	CODE_PING							  1
#define	CODE_PING_RESPONSE					  2
#define	CODE_IND_ON							  3
#define	CODE_IND_OFF					      4
#define	CODE_IND_TOGGLE						  5

#define	CODE_HI						    	  10
#define	CODE_HI_RESPONSE					  11
#define	CODE_EXPLORE_ADJ				      12
#define	CODE_EXPLORE_ADJ_RESPONSE			  13
#define	CODE_PORT_DIRECTION		    		  14
#define	CODE_BAUDRATE						  15
#define	CODE_MODULE_ID						  16
#define	CODE_TOPOLOGY						  17
#define	CODE_BROADCAST_PLAN					  18
#define	CODE_READ_PORT_DIR					  19
#define	CODE_READ_PORT_DIR_RESPONSE	    	  20
#define	CODE_EXP_EEPROM	 					  21
#define	CODE_DEF_ARRAY	 					  22
#define	CODE_CLI_COMMAND 					  23
#define	CODE_CLI_RESPONSE  					  24
#define	CODE_UPDATE  					      25
#define	CODE_UPDATE_VIA_PORT  				  26
#define	CODE_DMA_CHANNEL  					  27
#define	CODE_DMA_SCAST_STREAM  				  28

#define	CODE_READ_REMOTE  					  30
#define	CODE_READ_REMOTE_RESPONSE  	   		  31
#define	CODE_WRITE_REMOTE  					  32
#define	CODE_WRITE_REMOTE_RESPONSE            33
#define	CODE_WRITE_REMOTE_FORCE				  34
#define	CODE_PORT_FORWARD     				  35
#define	CODE_READ_REMOTE_ModBus_RESPONSE  	  36
#define CODE_READ_ADC_VALUE					  40
#define CODE_READ_TEMPERATURE				  41
#define CODE_READ_VREF						  42
#define CODE_READ_ADC_PERCENTAGE			  43

#define	MSG_Acknowledgment_Accepted 	  	  44
#define	MSG_rejected 						  45


/*
   -----------------------------------------------------------------------
  |					User Message Codes (70-99)  						  |
   -----------------------------------------------------------------------
*/

/*
   -----------------------------------------------------------------------
  |				Module Message Codes	                                  |
   -----------------------------------------------------------------------
*/

// Reserve 50 messages for each PN based on its decimal value
// H01R0x
#define	CODE_H01R0_ON						  100
#define	CODE_H01R0_OFF						  101
#define	CODE_H01R0_TOGGLE				  	  102
#define	CODE_H01R0_COLOR					  103
#define	CODE_H01R0_PULSE					  104
#define	CODE_H01R0_SWEEP					  105
#define	CODE_H01R0_DIM						  106

// H07R3x
#define CODE_H07R3_PLAY_SINE				  350
#define CODE_H07R3_PLAY_WAVE	   			  351
#define CODE_H07R3_PLAY_TUNE				  352
#define CODE_H07R3_SCAN_WAVE_RESPONSE		  353

// H08R6x
#define CODE_H08R6_GET_INFO                   400
#define CODE_H08R6_SAMPLE_PORT                401
#define CODE_H08R6_STREAM_PORT                402
#define CODE_H08R6_STREAM_MEM                 403
#define CODE_H08R6_RESULT_MEASUREMENT         404
#define CODE_H08R6_STOP_RANGING               405
#define CODE_H08R6_SET_UNIT                   406
#define CODE_H08R6_GET_UNIT                   407
#define CODE_H08R6_RESPOND_GET_UNIT           408
#define CODE_H08R6_MAX_RANGE                  409
#define CODE_H08R6_MIN_RANGE                  410
#define CODE_H08R6_TIMEOUT                	  411


// H09R0
#define CODE_H09R0_STREAM_PORT_C              450
#define CODE_H09R0_STREAM_PORT_F              451
#define CODE_H09R0_STREAM_PORT_K              452
#define CODE_H09R0_SAMPLE_PORT_C              453
#define CODE_H09R0_SAMPLE_PORT_F              454
#define CODE_H09R0_SAMPLE_PORT_K              455
#define CODE_H09R0_STOP                       456

//H09R9
#define CODE_H09R9_SAMPLE_TEMP                475
#define CODE_H09R9_STREAM_TEMP                476
#define CODE_H09R9_STREAM_STOP                477

// H0AR9
#define CODE_H0AR9_SAMPLE_COLOR               500
#define CODE_H0AR9_SAMPLE_DISTANCE            501
#define CODE_H0AR9_SAMPLE_TEMP                502
#define CODE_H0AR9_SAMPLE_HUMIDITY            503
#define CODE_H0AR9_SAMPLE_PIR                 504
#define CODE_H0AR9_STREAM_COLOR               505
#define CODE_H0AR9_STREAM_DISTANCE            506
#define CODE_H0AR9_STREAM_TEMP                507
#define CODE_H0AR9_STREAM_HUMIDITY            508
#define CODE_H0AR9_STREAM_PIR                 509
#define CODE_H0AR9_STREAM_STOP                510

// H0BR4x
#define CODE_H0BR4_SAMPLE_PORT_GYRO           550
#define CODE_H0BR4_SAMPLE_PORT_ACC            551
#define CODE_H0BR4_SAMPLE_PORT_MAG		      552
#define CODE_H0BR4_SAMPLE_PORT_TEMP		      553
#define CODE_H0BR4_RESULT_GYRO                554
#define CODE_H0BR4_RESULT_ACC                 555
#define CODE_H0BR4_RESULT_MAG		 	      556
#define CODE_H0BR4_RESULT_TEMP		          557
#define CODE_H0BR4_STREAM_PORT_GYRO			  558
#define CODE_H0BR4_STREAM_PORT_ACC			  559
#define CODE_H0BR4_STREAM_PORT_MAG			  560
#define CODE_H0BR4_STREAM_PORT_TEMP			  561
#define CODE_H0BR4_STREAM_STOP				  562

// H0FR1x
#define	CODE_H0FR1_ON						  750
#define	CODE_H0FR1_OFF						  751
#define	CODE_H0FR1_TOGGLE					  752

// H0FR6x
#define	CODE_H0FR6_ON						  760
#define	CODE_H0FR6_OFF						  761
#define	CODE_H0FR6_TOGGLE					  762
#define	CODE_H0FR6_PWM						  763

// H0FR7x
#define	CODE_H0FR7_ON						  770
#define	CODE_H0FR7_OFF						  771
#define	CODE_H0FR7_TOGGLE					  772
#define	CODE_H0FR7_PWM						  773
#define	CODE_H0FR7_SAMPLE_PORT				  774
#define	CODE_H0FR7_STREAM_PORT				  775
#define	CODE_H0FR7_STREAM_BUFFER			  776
#define	CODE_H0FR7_STOP_MEASUREMENT			  777

//H18R1
#define CODE_H18R1_Turn_ON					  1200
#define CODE_H18R1_Turn_OFF					  1201
#define CODE_H18R1_Turn_PWM					  1202

//H1AR0
#define CODE_H1AR0_Transmit_Data			  1300

// H1BR6x
#define CODE_H1BR6_READ_WAVE            	  1350
#define CODE_H1BR6_SCAN_WAVE			   	  1351

// H10R4x
#define CODE_H10R4_STOP						  1360
#define CODE_H10R4_STREAM_PORT				  1361
#define CODE_H10R4_STREAM_CLI				  1362
#define CODE_H10R4_STREAM_RAW				  1363
#define CODE_H10R4_STREAM_VARIANT			  1364
#define CODE_H10R4_STREAM_BUFFER			  1365
#define CODE_H10R4_STREAM_TYPE				  1366

// H23R0x and H23R3x
#define CODE_H23Rx_SCAN_INQUIRE               1700
#define CODE_H23Rx_CONNECT_INQUIRE            1701
#define CODE_H23Rx_DISCONNECT_INQUIRE         1702
#define CODE_H23Rx_CLEAR_USER_BUFFER		  1703
#define CODE_H23Rx_SEND_DATA		 		  1704
#define CODE_H23Rx_SET_NAME					  1705
#define CODE_H23Rx_SET_DISCOVERABLE			  1706
#define CODE_H23Rx_STREAM_TO_PORT			  1707

// H26R0x
#define CODE_H26R0_SET_RATE                   1900
#define CODE_H26R0_STREAM_PORT_GRAM           1901
#define CODE_H26R0_STREAM_PORT_KGRAM          1902
#define CODE_H26R0_STREAM_PORT_OUNCE          1903
#define CODE_H26R0_STREAM_PORT_POUND          1904
#define CODE_H26R0_STOP                       1905
#define CODE_H26R0_SAMPLE_PORT_GRAM           1906
#define CODE_H26R0_SAMPLE_PORT_KGRAM          1907
#define CODE_H26R0_SAMPLE_PORT_OUNCE          1908
#define CODE_H26R0_SAMPLE_PORT_POUND          1909
#define CODE_H26R0_ZEROCAL                	  1910
#define CODE_H26R0_STREAM_RAW                 1911
#define CODE_H26R0_SAMPLE_RAW                 1912
#define CODE_H26R0_STREAM_FORMAT              1913

// H15R0x
#define CODE_H15R0_AnalogPercentage           1950
#define CODE_H15R0_AnalogOutValue             1951
//H2AR3
#define CODE_H2AR3_SAMPLE_V                   2100
#define CODE_H2AR3_SAMPLE_A				      2101
#define CODE_H2AR3_STREAM_PORT_V		      2102
#define CODE_H2AR3_STREAM_PORT_A		      2103
#define CODE_H2AR3_STOP                       2104
// H1DR1x
#define CODE_H1DR1_MODE                       2900
#define CODE_H1DR1_READ                       2901
#define CODE_H1DR1_WRITE                      2902
#define CODE_H1DR1_MULTIWRITE                 2903
#define CODE_H1DR1_STIMEOUT                   2904

//H12R0x
#define CODE_H12R0_STREAM_PORT                2905
#define CODE_H12R0_STOP          		      2906
#define CODE_H12R0_SAMPLE          		      2907

//H3BR6
#define CODE_H3BR6_SevenDisplayNumber          2950
#define CODE_H3BR6_SevenDisplayNumberF         2951
#define CODE_H3BR6_SevenDisplayQuantities      2952
#define CODE_H3BR6_SevenDisplayLetter          2953
#define CODE_H3BR6_SevenDisplaySentence        2954
#define CODE_H3BR6_SevenDisplayMovingSentence  2955
#define CODE_H3BR6_SevenDisplayOff             2956
#define CODE_H3BR6_SetIndicator                2957
#define CODE_H3BR6_ClearIndicator              2958

// H1FR5x


#endif /* BOS_MSGCODES_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
