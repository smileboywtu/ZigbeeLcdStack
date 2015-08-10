/**************************************************************************************************
  Filename:       DemoApp.h
 
  Description:    Header file for the Sensor Demo application. This application
                  has two device types; Collectors and Sensors. 

                  A collector device functions as Routers and may be configured
                  in gateway mode to accept binding requests from Sensors. The
                  gateway node also forward received sensor reports to its
                  serial port. 

                  The sensors will periodically report sensor data to the 
                  gateway in the system. 


  Copyright 2009 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/


#ifndef SIMPLE_APP_H
#define SIMPLE_APP_H

/******************************************************************************
 * CONSTANTS
 */

#define MY_PROFILE_ID                     0x0F20
#define MY_ENDPOINT_ID                    0x02

// Define devices
#define DEV_ID_SENSOR                     1
#define DEV_ID_COLLECTOR                  2

#define DEVICE_VERSION_SENSOR             1
#define DEVICE_VERSION_COLLECTOR          1

// Define the Command ID's used in this application
#define SENSOR_REPORT_CMD_ID              2
#define DUMMY_REPORT_CMD_ID               3

// Sensor report data format
#define SENSOR_TEMP_OFFSET                0
#define SENSOR_VOLTAGE_OFFSET             1
#define SENSOR_PARENT_OFFSET              2
#define SENSOR_REPORT_LENGTH              4

#define RX_BUF_LEN                        128

/******************************************************************************
 * PUBLIC FUNCTIONS
 */

void initUart(halUARTCBack_t pf);
void uartRxCB( uint8 port, uint8 event );

#endif // SIMPLE_APP_H




