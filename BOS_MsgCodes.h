/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : BOS_MsgCodes.h
 Description   : Header file for for BOS communication message codes.

 */

/* Define to prevent recursive inclusion************************************/
#ifndef BOS_MSGCODES_H
#define BOS_MSGCODES_H

/***************************************************************************/
/* BOS Message Codes *******************************************************/
/***************************************************************************/
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
//#define	CODE_WRITE_REMOTE_FORCE				  34
#define	CODE_PORT_FORWARD     				  35
#define	CODE_READ_REMOTE_ModBus_RESPONSE  	  36
#define CODE_READ_ADC_VALUE					  40
#define CODE_READ_TEMPERATURE				  41
#define CODE_READ_VREF						  42
#define CODE_READ_ADC_PERCENTAGE			  43

#define CODE_READ_RESPONSE			          46

#define ENABLE_STOP_MODE_UARTX                47
#define ENABLE_STANDBY_MODE_WAKE_UP_PINX      48

#define CODE_RAW_DATA                         49

/***************************************************************************/
/* User Message Codes (70 -  99) *******************************************/
/***************************************************************************/



/***************************************************************************/
/* Module Message Codes ****************************************************/
/***************************************************************************/
/* Reserve 50 messages for each PN based on its decimal value **************/

// H01R0x
#define	CODE_H01R0_ON						  100
#define	CODE_H01R0_OFF						  101
#define	CODE_H01R0_TOGGLE				  	  102
#define	CODE_H01R0_COLOR					  103
#define	CODE_H01R0_PULSE					  104
#define	CODE_H01R0_SWEEP					  105
#define	CODE_H01R0_DIM						  106

// H05R0x
#define CODE_H05R0_CELLVOLTAGE				  250
#define CODE_H05R0_CELLCURRENT			      251
#define CODE_H05R0_CELLPOWER				  252
#define CODE_H05R0_CELLTEMPERATURE			  253
#define CODE_H05R0_CELLCAPACITY				  254
#define CODE_H05R0_SOC    		        	  255
#define CODE_H05R0_CELLAGE					  258
#define CODE_H05R0_CELLCYCLES    			  259


// H07R3x
#define CODE_H07R3_PLAY_SINE				  350
#define CODE_H07R3_PLAY_WAVE	   			  351
#define CODE_H07R3_PLAY_TUNE				  352
#define CODE_H07R3_SCAN_WAVE_RESPONSE		  353

// H07R8x
#define CODE_H07R8_CODEC_STREAM_START         375
#define CODE_H07R8_CODEC_STREAM_STOP          376
#define CODE_H07R8_CODEC_DAC_GAIN		      377
#define CODE_H07R8_CODEC_AUDIO_LEVEL_CTRL     378
#define CODE_H07R8_CODEC_AUDIO_MUTE           379
#define CODE_H07R8_CODEC_AUDIO_UNMUTE         380
#define CODE_H07R8_CODEC_EN_SHUTDOWN	      381
#define CODE_H07R8_CODEC_DIS_SHUTDOWN	      382
#define CODE_H07R8_AMP_GAIN					  383
#define CODE_H07R8_AMP_MUTE                   384
#define CODE_H07R8_AMP_UNMUTE                 385
#define CODE_H07R8_AMP_EN_SHUTDOWN            386
#define CODE_H07R8_AMP_DIS_SHUTDOWN           387

// H08R7x
#define CODE_H08R7_SAMPLE_PORT                400



// P08R7
#define CODE_P08R7_GET_INFO                   420
#define CODE_P08R7_SAMPLE_PORT                421
#define CODE_P08R7_STREAM_PORT                422

// H08R6x


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

/* H0AR9 */
#define CODE_H0AR9_SAMPLE_COLOR               500
#define CODE_H0AR9_SAMPLE_DISTANCE            501
#define CODE_H0AR9_SAMPLE_TEMP                502
#define CODE_H0AR9_SAMPLE_HUMIDITY            503
#define CODE_H0AR9_SAMPLE_PIR                 504

/* H0BR4x */
#define CODE_H0BR4_SAMPLE_GYRO                550
#define CODE_H0BR4_SAMPLE_ACC                 551
#define CODE_H0BR4_SAMPLE_MAG		          552
#define CODE_H0BR4_SAMPLE_TEMP		          553

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
#define	CODE_H0FR7_GET_CURRENT			      774

// H14RAx
#define	CODE_H14RA_ON						  1000
#define	CODE_H14RA_OFF                        1001
#define	CODE_H14RA_SPEED                      1002
#define	CODE_H14RA_PWM                        1003

// H16R6x
#define CODE_H16R6_SET_COLOR                  1100
#define CODE_H16R6_SET_ALL_COLOR              1101
#define CODE_H16R6_SET_RGB                    1102
#define CODE_H16R6_SET_ALL_RGB                1103
#define CODE_H16R6_SET_LED_OFF                1104
#define CODE_H16R6_SET_ALL_LED_OFF            1105
#define CODE_H16R6_SET_LED_ON                 1106
#define CODE_H16R6_SET_ALL_LED_ON             1107
#define CODE_H16R6_SCROLL_MODE                1108
#define CODE_H16R6_FLASH_MODE                 1109
#define CODE_H16R6_COLOR_PICKER_MODE          1110
#define CODE_H16R6_SET_COLOR_SOME_LED         1111
#define CODE_H16R6_MOTION_MODE                1112
#define CODE_H16R6_CROSS_FADE_MODE            1113
#define CODE_H16R6_CROSS_FADE_MODE_LED_RGB    1114
#define CODE_H16R6_CROSS_FADE_MODE_ALL_LED_RGB 1115
//#define CODE_H16R6_SPRINKLEMODE             1116

// H17R1x
#define CODE_H17R1_STEPPER_IC_INIT            1150
#define CODE_H17R1_STEPPER_MOVE               1151
#define CODE_H17R1_STEPPER_RUN                1152
#define CODE_H17R1_STEPPER_STOP               1153

//H18R1
#define CODE_H18R1_TURN_ON                    1200
#define CODE_H18R1_TURN_OFF                   1201
#define CODE_H18R1_TURN_PWM                   1202

//H1AR0
#define CODE_H1AR0_Transmit_Data              1300

// H1BR6x
#define CODE_H1BR6_READ_WAVE                  1350
#define CODE_H1BR6_SCAN_WAVE                  1351

// H10R4x
#define CODE_H10R4_STOP                       1360
#define CODE_H10R4_STREAM_PORT                1361
#define CODE_H10R4_STREAM_CLI                 1362
#define CODE_H10R4_STREAM_RAW                 1363
#define CODE_H10R4_STREAM_VARIANT             1364
#define CODE_H10R4_STREAM_BUFFER              1365
#define CODE_H10R4_STREAM_TYPE                1366

//H1DR5x
#define CODE_H1DR5_ETHERNET_SEND_DATA         1450
//#define CODE_H1DR5_Ethernet_Receive_Data    1451
#define CODE_H1DR5_SET_LOCAL_IP               1452
#define CODE_H1DR5_SET_REMOTE_IP              1453
#define CODE_H1DR5_SET_SUBNET_MASK            1454
#define CODE_H1DR5_SET_LOCAL_PORT             1455
#define CODE_H1DR5_SET_REMOTE_PORT            1456
#define CODE_H1DR5_SET_REMOTE_IP_REMOTE_MAC   1457
#define CODE_H1DR5_DEFAULT_VALUES             1458
#define CODE_H1DR5_RECEIVE_DEFAULT_VALUE      1459

// H1FR5x
#define CODE_H1FR5_GET_POSITION               1550
#define CODE_H1FR5_GET_UTC                    1551
#define CODE_H1FR5_GET_SPEED                  1552
#define CODE_H1FR5_GET_HIEGHT                 1553

// H21R2x
#define CODE_H21R2_ESP_RESET                  1650
#define CODE_H21R2_ESP_BOOT                   1651
#define CODE_H21R2_ESP_SERVER                 1652
#define CODE_H21R2_ESP_CLIENT                 1653
#define CODE_H21R2_ESP_ACCESS_POINT           1654
#define CODE_H21R2_ESP_STATION                1655
#define CODE_H21R2_ESP_READ_FROM_SERVER       1656
#define CODE_H21R2_ESP_WRITE_TO_SERVER        1657
#define CODE_H21R2_ESP_READ_FROM_CLIENT       1658
#define CODE_H21R2_ESP_WRITE_TO_CLIENT        1659

// H23R0x and H23R3x
#define CODE_H23Rx_SCAN_INQUIRE              1700
#define CODE_H23Rx_CONNECT_INQUIRE            1701
#define CODE_H23Rx_DISCONNECT_INQUIRE         1702
#define CODE_H23Rx_CLEAR_USER_BUFFER          1703
#define CODE_H23Rx_SEND_DATA                  1704
#define CODE_H23Rx_SET_NAME                   1705
#define CODE_H23Rx_SET_DISCOVERABLE           1706
#define CODE_H23Rx_STREAM_TO_PORT             1707

// H26R0x
#define CODE_H26R0_SET_RATE                   1900
#define CODE_H26R0_STREAM_PORT_GRAM           1901
#define CODE_H26R0_STREAM_PORT_KGRAM          1902
#define CODE_H26R0_STREAM_PORT_OUNCE          1903
#define CODE_H26R0_STREAM_PORT_POUND          1904
#define CODE_H26R0_STOP                       1905
#define CODE_H26R0_STREAM_PORT                    6
#define CODE_H26R0_SAMPLE_PORT_GRAM         1906
#define CODE_H26R0_SAMPLE_PORT_KGRAM         1907
#define CODE_H26R0_SAMPLE_PORT_OUNCE          1908
#define CODE_H26R0_SAMPLE_PORT_POUND         1909
#define CODE_H26R0_ZEROCAL                    1910
#define CODE_H26R0_STREAM_RAW                 1911
#define CODE_H26R0_SAMPLE_RAW                 1912
#define CODE_H26R0_STREAM_FORMAT              1913

// H15R0x
#define CODE_H15R0_AnalogPercentage           1950
#define CODE_H15R0_AnalogOutValue             1951

//H2AR3
#define CODE_H2AR3_SAMPLE_VOLT                2100
#define CODE_H2AR3_SAMPLE_CURR                2101

// H2BR0x and H2BR1x
#define CODE_H2BR0_ECG_SAMPLE                 2150
#define CODE_H2BR0_EOG_SAMPLE                 2151
#define CODE_H2BR0_EEG_SAMPLE                 2152
#define CODE_H2BR0_EMG_SAMPLE                 2153
#define CODE_H2BR0_EMG_SET_THRESHOLD          2154
#define CODE_H2BR0_EMG_CHECK_PULSE            2155
#define CODE_H2BR0_ECG_HEART_RATE             2156
#define CODE_H2BR0_EOG_CHECK_EYE_BLINK        2157
#define CODE_H2BR0_LEADS_STATUS               2158

// H2BR1
#define CODE_H2BR1_HR_SAMPLE                  2175
#define CODE_H2BR1_SPO2_SAMPLE                2176

// H1DR1x
#define CODE_H1DR1_MODE                   2900
#define CODE_H1DR1_READ                   2901
#define CODE_H1DR1_WRITE                   2902
#define CODE_H1DR1_MULTIWRITE                   2903
#define CODE_H1DR1_STIMEOUT                 2904

//H12R0x
#define CODE_H12R0_STREAM_PORT               2905
#define CODE_H12R0_STOP                     2906
#define CODE_H12R0_SAMPLE                   2907

//H3BR6 and H3BR7
#define CODE_H3BRX_SEVEN_DISPLAY_NUMBER          2950
#define CODE_H3BRX_SEVEN_DISPLAY_NUMBER_F        2951
#define CODE_H3BRX_SEVEN_DISPLAY_QUANTITIES      2952
#define CODE_H3BRX_SEVEN_DISPLAY_LETTER          2953
#define CODE_H3BRX_SEVEN_DISPLAY_SENTENCE        2954
#define CODE_H3BRX_SEVEN_DISPLAY_MOVING_SENTENCE 2955
#define CODE_H3BRX_SEVEN_DISPLAY_OFF             2956
#define CODE_H3BRX_SET_INDICATOR                 2957
#define CODE_H3BRX_CLEAR_INDICATOR               2958

// H3BR2x
#define CODE_H3BR2_SEVEN_DISPLAY_NUMBER          2975
#define CODE_H3BR2_SEVEN_DISPLAY_NUMBER_HEXA     2976
#define CODE_H3BR2_SEVEN_DISPLAY_ONE_DIGIT       2977
#define CODE_H3BR2_SEVEN_DISPLAY_ONE_DIGIT_HEXA  2978
#define CODE_H3BR2_SEVEN_DISPLAY_OFF             2979
#define CODE_H3BR2_SEVEN_DISPLAY_NUMBER_F        2980

#endif /* BOS_MSGCODES_H */

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
