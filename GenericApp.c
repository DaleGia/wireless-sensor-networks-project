/******************************************************************************
  Filename:       GenericApp.c
  Revised:        $Date: 2014-09-07 13:36:30 -0700 (Sun, 07 Sep 2014) $
  Revision:       $Revision: 40046 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2014 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
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
******************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful - it is
  intended to be a simple example of an application's structure.

  This application periodically sends a "Hello World" message to
  another "Generic" application (see 'txMsgDelay'). The application
  will also receive "Hello World" packets.

  This application doesn't have a profile, so it handles everything
  directly - by itself.

  Key control:
    SW1:  changes the delay between TX packets
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "GenericApp.h"
#include "DebugTrace.h"

#if !defined( WIN32 ) || defined( ZBIT )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
#include "hal_adc.h"

/* RTOS */
#if defined( IAR_ARMCM3_LM )
#include "RTOS_App.h"
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */
typedef struct
{
  uint8 command;
  uint8 value;
  uint8 device_type;
} command_value;
/*********************************************************************
 * GLOBAL VARIABLES
 */
// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
{
  GENERICAPP_CLUSTERID,
  SERIAL_CLUSTERID_1,
  SERIAL_CLUSTERID_2,
};

const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
{
  GENERICAPP_ENDPOINT,              //  int Endpoint;
  GENERICAPP_PROFID,                //  uint16 AppProfId[2];
  GENERICAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  GENERICAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  GENERICAPP_FLAGS,                 //  int   AppFlags:4;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList,  //  byte *pAppInClusterList;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t GenericApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte GenericApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // GenericApp_Init() is called.

devStates_t GenericApp_NwkState;
byte GenericApp_TransID;  // This is the unique message ID (counter)
afAddrType_t serial_tx_addr;
// Number of recieved messages
static uint16 rxMsgCount;

static afAddrType_t serial_tx_addr;
static uint8 serial_tx_buf[SERIAL_TX_MAX+1];
static uint8 serial_tx_len;




/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void GenericApp_HandleKeys( byte shift, byte keys );
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
static void sendSensorResponse( void );
static void sendSensorRequest(void);

static void serialCallBack(uint8 port, uint8 event);
void serialInit(uint8 task_id);
static void sendDiscoveryResponse(void);
static void sendDiscoveryRequest(void);


#if defined( IAR_ARMCM3_LM )
static void GenericApp_ProcessRtosMessage( void );
#endif

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void serialInit(uint8 task_id)
{
  halUARTCfg_t uartConfig;
  
  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = SERIAL_BAUD;
  uartConfig.flowControl          = TRUE;
  uartConfig.flowControlThreshold = SERIAL_THRESH; // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = SERIAL_RX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = SERIAL_TX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = SERIAL_IDLE;   // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = serialCallBack;
  HalUARTOpen (SERIAL_PORT, &uartConfig);
}

/*********************************************************************
 * @fn      GenericApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void GenericApp_Init( uint8 task_id )
{
  GenericApp_TaskID = task_id;
  GenericApp_NwkState = DEV_INIT;
  GenericApp_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  //serial_tx_addr.addrMode = (afAddrMode_t)AddrNotPresent;
  //serial_tx_addr.endPoint = 0;
  serial_tx_addr.addrMode = (afAddrMode_t)AddrBroadcast;
  serial_tx_addr.endPoint = GENERICAPP_ENDPOINT;
  serial_tx_addr.addr.shortAddr = 0xFFFF;

  // Fill out the endpoint description.
  GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_epDesc.task_id = &GenericApp_TaskID;
  GenericApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
  GenericApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &GenericApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( GenericApp_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "GenericApp", HAL_LCD_LINE_1 );
#endif

  ZDO_RegisterForZDOMsg( GenericApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( GenericApp_TaskID, Match_Desc_rsp );
  
  serialInit(GenericApp_TaskID);


#if defined( IAR_ARMCM3_LM )
  // Register this task with RTOS task initiator
  RTOS_RegisterApp( task_id, GENERICAPP_RTOS_MSG_EVT );
#endif
}

/*********************************************************************
 * @fn      GenericApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 GenericApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
        {
          GenericApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;
        }
        case KEY_CHANGE:
        {
          GenericApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;
        }
        case AF_DATA_CONFIRM_CMD:
        {
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;

          sentEP = afDataConfirm->endpoint;
          (void)sentEP;  // This info not used now
          sentTransID = afDataConfirm->transID;
          (void)sentTransID;  // This info not used now

          sentStatus = afDataConfirm->hdr.status;
          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
          }
          break;
        }
        case AF_INCOMING_MSG_CMD:
        {
          GenericApp_MessageMSGCB( MSGpkt );
          break;
        }
        case ZDO_STATE_CHANGE:
        {
          GenericApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (GenericApp_NwkState == DEV_ZB_COORD) ||
               (GenericApp_NwkState == DEV_ROUTER) ||
               (GenericApp_NwkState == DEV_END_DEVICE) )
          {
          }
          break;
        }
        default:
        {
          break;
        }
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      GenericApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined( BLINK_LEDS )
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            serial_tx_addr.addrMode = (afAddrMode_t)Addr16Bit;
            serial_tx_addr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            serial_tx_addr.endPoint = pRsp->epList[0];
            

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

/*********************************************************************
 * @fn      GenericApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void GenericApp_HandleKeys( uint8 shift, uint8 keys )
{
  zAddrType_t dstAddr;
  if(keys & HAL_KEY_UP)
  {     
    uint8 yo[] = {"yomamarama"};
    if (HalUARTWrite(SERIAL_PORT, yo, sizeof(yo)+1))
    {
          HalLedSet ( HAL_LED_4, HAL_LED_MODE_BLINK );  // Blink an LED
    }
  }
  
  if ( keys & HAL_KEY_RIGHT)
  {

    // Initiate an End Device Bind Request for the mandatory endpoint
    dstAddr.addrMode = Addr16Bit;
    dstAddr.addr.shortAddr = 0x0000; // Coordinator
    ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                          GenericApp_epDesc.endPoint,
                          GENERICAPP_PROFID,
                          GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                          GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                          FALSE );
  }

  if ( keys & HAL_KEY_DOWN)
  {
    if(GenericApp_NwkState == DEV_ZB_COORD)
    {
      sendSensorRequest();
    }
  }

  if ( keys & HAL_KEY_LEFT)
  {
    HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
    // Initiate a Match Description Request (Service Discovery)
    dstAddr.addrMode = AddrBroadcast;
    dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
    ZDP_MatchDescReq( &dstAddr, NWK_BROADCAST_SHORTADDR,
                      GENERICAPP_PROFID,
                      GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                      GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                      FALSE );
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
    case GENERICAPP_CLUSTERID:
    {
      command_value* data = (command_value*)pkt->cmd.Data;
      switch(data->command)
      {
        case ADC_RESPONSE:
        {
          rxMsgCount += 1;  // Count this message
          //HalLedSet ( HAL_LED_4, HAL_LED_MODE_BLINK );  // Blink an LED
          //HalUARTWrite( SERIAL_PORT, (uint8*)pkt->cmd.Data, (pkt->cmd.DataLength-1));
          uint8 data_snd[5];
          data_snd[0] = pkt->cmd.Data[0];
          data_snd[1] = pkt->cmd.Data[1];
          data_snd[2] = pkt->cmd.Data[2];
          data_snd[3] = pkt->srcAddr.addr.shortAddr & 0xFF00;
          data_snd[4] = pkt->srcAddr.addr.shortAddr & 0x00FF;
          if(HalUARTWrite( SERIAL_PORT,data_snd, sizeof(data_snd)+1))
          {   
            HalLedSet ( HAL_LED_3, HAL_LED_MODE_BLINK );  // Blink an LED
          }
        //  HalLcdWriteStringValue("Value:", data->value, 16, HAL_LCD_LINE_1 );
        //  HalLcdWriteStringValue( "Rcvd:", rxMsgCount, 10, HAL_LCD_LINE_2 );
          break;
        }
        case ADC_REQUEST:
        {
          HalLedSet ( HAL_LED_3, HAL_LED_MODE_BLINK );  // Blink an LED
          sendSensorResponse();
          break;
        }
        case DISCOVERY_RESPONSE:
        {
          rxMsgCount += 1;  // Count this message
          uint8 data_snd[5];
          data_snd[0] = pkt->cmd.Data[0];
          data_snd[1] = pkt->cmd.Data[1];
          data_snd[2] = pkt->cmd.Data[2];
          data_snd[3] = pkt->srcAddr.addr.shortAddr & 0xFF00;
          data_snd[4] = pkt->srcAddr.addr.shortAddr & 0x00FF;
          if(HalUARTWrite( SERIAL_PORT, data_snd, sizeof(data_snd)+1))
          {   
            HalLedSet ( HAL_LED_3, HAL_LED_MODE_BLINK );  // Blink an LED
          }
        //  HalLcdWriteStringValue("Value:", data->value, 16, HAL_LCD_LINE_1 );
        //  HalLcdWriteStringValue( "Rcvd:", rxMsgCount, 10, HAL_LCD_LINE_2 );
          break;
        }
        case DISCOVERY_REQUEST:
        {
           HalLedSet ( HAL_LED_3, HAL_LED_MODE_BLINK );  // Blink an LED
          sendDiscoveryResponse();
        }
        break;
      }
      break;
    }
  }
}


/*********************************************************************
 * @fn      SerialApp_CallBack
 *
 * @brief   Send data OTA.
 *
 * @param   port - UART port.
 * @param   event - the UART port event flag.
 *
 * @return  none
 */
static void serialCallBack(uint8 port, uint8 event)
{
  (void)port;

  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)) &&
#if SERIAL_LOOPBACK
      (serial_tx_len < SERIAL_TX_MAX))
#else
      !serial_tx_len)
#endif
  {  
    if (HalUARTRead(SERIAL_PORT, serial_tx_buf, SERIAL_TX_MAX))
    {
      command_value* data = (command_value*)serial_tx_buf;
      HalLcdWriteStringValue("V:",data->command,16, HAL_LCD_LINE_3);
      if(data->command == ADC_REQUEST)
      {
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_BLINK );
        sendSensorRequest();
      }
      else if(data->command == DISCOVERY_REQUEST)
      {
        sendDiscoveryRequest();
      }
    }
  }
}

/*********************************************************************
*********************************************************************/
static void sendSensorRequest(void)
{
  command_value data;
  data.command = ADC_REQUEST;
  data.value = 0;
  data.device_type = GenericApp_NwkState;

  if( AF_DataRequest(&serial_tx_addr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       sizeof(data)+1,
                       (uint8*)&data,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
     HalLedSet ( HAL_LED_2, HAL_LED_MODE_BLINK );

  }
  else
  {
    HalLcdWriteString("ADC RQST FAIL", HAL_LCD_LINE_3);
  }
}

static void sendDiscoveryResponse(void)
{
    command_value data;
    data.command = DISCOVERY_RESPONSE;
    data.value = 0;
    data.device_type = GenericApp_NwkState;

    if( AF_DataRequest(&serial_tx_addr, &GenericApp_epDesc,
                         GENERICAPP_CLUSTERID,
                         sizeof(data)+1,
                         (uint8*)&data,
                         &GenericApp_TransID,
                         AF_DISCV_ROUTE, 
                         AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
      // Successfully requested to be sent.
    }
    else
    {
      // Error occurred in request to send.
    }
}

static void sendDiscoveryRequest(void)
{
  command_value data;
  data.command = DISCOVERY_REQUEST;
  data.value = 0;
  data.device_type = GenericApp_NwkState;

  if( AF_DataRequest(&serial_tx_addr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       sizeof(data)+1,
                       (uint8*)&data,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
     HalLedSet ( HAL_LED_2, HAL_LED_MODE_BLINK );

  }
  else
  {
    HalLcdWriteString("DISC RQST FAIL", HAL_LCD_LINE_3);
  }
}

static void sendSensorResponse( void )
{
  if(GenericApp_NwkState == DEV_END_DEVICE)
  {
    command_value data;
    data.command = ADC_RESPONSE;
    data.value = HalAdcRead(HAL_ADC_CHANNEL_7, 8);
    data.device_type = GenericApp_NwkState;
    // Address to Coordinator
   // afAddrType_t dstAddr;
   // dstAddr.addrMode = Addr16Bit;
    //dstAddr.addr.shortAddr = 0x0000; // Coordinator

    if( AF_DataRequest(&serial_tx_addr, &GenericApp_epDesc,
                         GENERICAPP_CLUSTERID,
                         sizeof(data)+1,
                         (uint8*)&data,
                         &GenericApp_TransID,
                         AF_DISCV_ROUTE, 
                         AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
      // Successfully requested to be sent.
    }
    else
    {
      // Error occurred in request to send.
    }
  }
}

