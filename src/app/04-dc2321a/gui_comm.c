/**
Copyright (c) 2016, Dust Networks.  All rights reserved.

GUI Comm Code Module

This code handles the GUI communication through the Eterna UART.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app_task_cfg.h"
#include "loc_task.h"
#include "dn_system.h"
#include "dn_uart.h"

#include "dc2321a.h"
#include "devices.h"
#include "cog_display.h"
#include "coulomb_counter.h"
#include "network.h"
#include "gui_comm.h"

//=========================== defines =========================================
#define MAX_UART_PACKET_SIZE   (128u)
#define MAX_UART_TRX_CHNL_SIZE (sizeof(dn_chan_msg_hdr_t) + MAX_UART_PACKET_SIZE)
#define GUI_COMM_DELAY     30

typedef struct
{
   // uartTxTask
   OS_STK          uartTxTaskStack[TASK_APP_UART_TX_STK_SIZE];
   volatile INT8U  uartTxBuffer[MAX_UART_PACKET_SIZE];
   volatile INT16U uartTxLen;
   INT16U          uartTxIdx;
   INT16U          uartTxDelay;
   OS_EVENT*       uartTxSem;          // counting semaphore for how many times uartTxBuffer has been written.
   OS_EVENT*       uartTxDataLock;     // binary semaphore so that only one task can write to uartTxBuffer at a time.

   // uartRxTask
   OS_STK          uartRxTaskStack[TASK_APP_UART_RX_STK_SIZE];
   INT32U          uartRxChannelMemBuf[1+MAX_UART_TRX_CHNL_SIZE/sizeof(INT32U)];
   OS_MEM*         uartRxChannelMem;
   CH_DESC         uartRxChannel;
   INT8U           uartRxBuffer[MAX_UART_PACKET_SIZE];
} uart_app_vars_t;

#define ASCII2INT(ascii) (ascii - '0')
#define WRITE_HEX_BYTES(ptr, val)                                               \
{                                                                               \
   if(sizeof(val) == sizeof(INT64U)) {writeHexBytes64(ptr, (INT64U)(val));}     \
   else if(sizeof(val) == sizeof(INT32U)) {writeHexBytes32(ptr, (INT32U)(val));}\
   else if(sizeof(val) == sizeof(INT16U)) {writeHexBytes16(ptr, (INT16U)(val));}\
   else if(sizeof(val) == sizeof(INT8U)) {writeHexBytes8(ptr, (INT8U)(val));}   \
}

typedef void (*gui_comm_id_function_t)(INT8U** bufferPtrPtr, void* dataStructPtr);

//=========================== prototypes ======================================
static void lockData(void);
static void unlockData(void);

INT16U generateProgramStatusBytes(INT8U updateAllGuiDataCommand);
void writeHexBytes8(INT8U** buffer, INT8U value);
void writeHexBytes16(INT8U** buffer, INT16U value);
void writeHexBytes32(INT8U** buffer, INT32U value);
void writeHexBytes64(INT8U** buffer, INT64U value);

void guiCommIdRadioData(INT8U** bufferPtrPtr, void* dataStructPtr);
void guiCommIdSlaveMode(INT8U** bufferPtrPtr, void* dataStructPtr);
void guiCommIdAll(INT8U** bufferPtrPtr, void* dataStructPtr);
void guiCommIdSimpleScreen(INT8U** bufferPtrPtr, void* dataStructPtr);
void guiCommIdStatusScreen(INT8U** bufferPtrPtr, void* dataStructPtr);
void guiCommIdBatScreen(INT8U** bufferPtrPtr, void* dataStructPtr);
void guiCommIdNetworkConfig(INT8U** bufferPtrPtr, void* dataStructPtr);
void guiCommIdNetworkTest(INT8U** bufferPtrPtr, void* dataStructPtr);
void guiCommIdNetworkAutoVar(INT8U** bufferPtrPtr, void* dataStructPtr);
void guiCommIdVersion(INT8U** bufferPtrPtr, void* dataStructPtr);

//=========================== const ===========================================
const gui_comm_id_function_t gui_comm_cmd_id_table[GUI_COMM_NUM_CMD_IDS] =
{ // Table to know which function to call without a switch statement.
   NULL,                    // GUI_COMM_ID_POKE has no function
   guiCommIdRadioData,      // GUI_COMM_ID_RADIO_DATA
   guiCommIdSlaveMode,      // GUI_COMM_ID_SLAVE_MODE
   guiCommIdAll,            // GUI_COMM_ID_ALL
   guiCommIdSimpleScreen,   // GUI_COMM_ID_SIMPLE_SCREEN
   guiCommIdStatusScreen,   // GUI_COMM_ID_STATUS_SCREEN
   guiCommIdBatScreen,      // GUI_COMM_ID_BAT_SCREEN
   guiCommIdNetworkConfig,  // GUI_COMM_ID_NETWORK_CONFIG
   guiCommIdNetworkTest,    // GUI_COMM_ID_NETWORK_TEST
   guiCommIdNetworkAutoVar, // GUI_COMM_ID_NETWORK_AUTO_VAR
   NULL,                    // GUI_COMM_ID_10_SKIPPED has no function
   guiCommIdVersion         // GUI_COMM_ID_VERSION
};                          // GUI_COMM_NUM_CMD_IDS

//=========================== global variables ================================
uart_app_vars_t    uart_app_v;

//=========================== tasks ===========================================
// Task to fill RX buffer with received data
// The received data is stored in uart_app_v.uartRxBuffer
void uartRxTask(void* unused)
{
   dn_error_t           dnErr;
   INT32U               rxLen;
   INT32U               msgType;
   INT8U                i;

   while(1) { // this is a task, it executes forever

      // wait for UART messages
      dnErr = dn_readAsyncMsg(
         uart_app_v.uartRxChannel,          // chDesc
         uart_app_v.uartRxBuffer,           // msg
         &rxLen,                            // rxLen
         &msgType,                          // msgType
         MAX_UART_PACKET_SIZE,              // maxLen
         0                                  // timeout (0==never)
      );
      ASSERT(dnErr==DN_ERR_NONE);
      ASSERT(msgType==DN_MSG_TYPE_UART_NOTIF);

      parseReceivedData(rxLen, uart_app_v.uartRxBuffer, TRUE);

      // print message received
//      dnm_ucli_printf("uart RX (%d bytes)",rxLen);
//      for (i=0;i<rxLen;i++) {
//         dnm_ucli_printf(" %02x",uart_app_v.uartRxBuffer[i]);
//      }
//      dnm_ucli_printf("\r\n");
   }
}

// Task to send the TX buffer
// The transfer information is stored in uart_app_v
void uartTxTask(void* unused)
{
   INT8U      osErr;
   dn_error_t dnErr;
   INT8U      reply = 0;
   INT32U     replyLen;

   while(1) // this is a task, it executes forever
   {
      // wait for the semaphore to be posted
      OSSemPend(
         uart_app_v.uartTxSem,         // pevent
         0,                            // timeout
         &osErr                        // perr
      );
      ASSERT (osErr == OS_ERR_NONE);

      if(uart_app_v.uartTxLen > 0) {

         // make copy of uartTxLen when task starts so that we can detect if
         // higher priority task altered uartTxBuffer/uartTxLen as we were running.
         // Note this is an atomic operation so no need to stop OS/Ints.
         INT16U uartTxLen_start = uart_app_v.uartTxLen;

         // send packet
         dnErr = dn_sendSyncMsgByType(
             &uart_app_v.uartTxBuffer[uart_app_v.uartTxIdx],
             uartTxLen_start - uart_app_v.uartTxIdx,
             DN_MSG_TYPE_UART_TX_CTRL,
             (void*)&reply,
             sizeof(reply),
             &replyLen
         );
         ASSERT(reply==DN_ERR_NONE);

         // check if higher priority task altered uartTxBuffer/uartTxLen.
         // note we don't want uartTxLen to change between checking and clearing
         // so the OS/Ints are stopped.
         lockData();
         if (uartTxLen_start == uart_app_v.uartTxLen) {
            uart_app_v.uartTxLen = 0;  // no new data added, reset the buffer.
            uart_app_v.uartTxIdx = 0;
         }
         else {
            uart_app_v.uartTxIdx = uartTxLen_start; // new data added, set up to only write new data upon next semaphore.
         }
         unlockData();

         // wait a bit
         if (uart_app_v.uartTxDelay) {
            OSTimeDly(uart_app_v.uartTxDelay);
         }
      }
   }
}

//=========================== functions =======================================
// Create the UART TX and RX tasks
void guiCommInit()
{
   INT8U osErr;

   // initialize UART variables
   memset(&uart_app_v, 0x00, sizeof(uart_app_v));

   // uart Tx semaphores
   uart_app_v.uartTxSem = OSSemCreate(0);
   ASSERT (uart_app_v.uartTxSem!=NULL);
   uart_app_v.uartTxDataLock = OSSemCreate(1);
   ASSERT (uart_app_v.uartTxDataLock!=NULL);

   // uartTxTask task
   osErr  = OSTaskCreateExt(
      uartTxTask,
      (void *)0,
      (OS_STK*)(&uart_app_v.uartTxTaskStack[TASK_APP_UART_TX_STK_SIZE-1]),
      TASK_APP_UART_TX_PRIORITY,
      TASK_APP_UART_TX_PRIORITY,
      (OS_STK*)uart_app_v.uartTxTaskStack,
      TASK_APP_UART_TX_STK_SIZE,
      (void *)0,
      OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR
   );
   ASSERT(osErr == OS_ERR_NONE);
   OSTaskNameSet(TASK_APP_UART_TX_PRIORITY, (INT8U*)TASK_APP_UART_TX_NAME, &osErr);
   ASSERT(osErr == OS_ERR_NONE);

   // uartRxTask task
   osErr  = OSTaskCreateExt(
      uartRxTask,
      (void *)0,
      (OS_STK*)(&uart_app_v.uartRxTaskStack[TASK_APP_UART_RX_STK_SIZE-1]),
      TASK_APP_UART_RX_PRIORITY,
      TASK_APP_UART_RX_PRIORITY,
      (OS_STK*)uart_app_v.uartRxTaskStack,
      TASK_APP_UART_RX_STK_SIZE,
      (void *)0,
      OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR
   );
   ASSERT(osErr == OS_ERR_NONE);
   OSTaskNameSet(TASK_APP_UART_RX_PRIORITY, (INT8U*)TASK_APP_UART_RX_NAME, &osErr);
   ASSERT(osErr == OS_ERR_NONE);
}

// Initialize UART devices for GUI communication
void guiCommInit2()
{
   dn_error_t           dnErr;
   INT8U                osErr;
   dn_uart_open_args_t  uartOpenArgs;
   INT32S               err;

   // create the memory block for the UART channel
   uart_app_v.uartRxChannelMem = OSMemCreate(
      uart_app_v.uartRxChannelMemBuf,
      1,
      sizeof(uart_app_v.uartRxChannelMemBuf),
      &osErr
   );
   ASSERT(osErr==OS_ERR_NONE);

   // create an asynchronous notification channel
   dnErr = dn_createAsyncChannel(uart_app_v.uartRxChannelMem, &uart_app_v.uartRxChannel);
   ASSERT(dnErr==DN_ERR_NONE);

   // associate the channel descriptor with UART notifications
   dnErr = dn_registerChannel(uart_app_v.uartRxChannel, DN_MSG_TYPE_UART_NOTIF);
   ASSERT(dnErr==DN_ERR_NONE);

    // open the UART device
   uartOpenArgs.rxChId      = uart_app_v.uartRxChannel;
   uartOpenArgs.eventChId   = 0;
   uartOpenArgs.rate        = DN_UART_BAUD_9600;
   uartOpenArgs.mode        = DN_UART_MODE_M0;
   uartOpenArgs.ctsOutVal   = 0;
   uartOpenArgs.fNoSleep    = 0;
   err = dn_open(
      DN_UART_DEV_ID,
      &uartOpenArgs,
      sizeof(uartOpenArgs)
   );
   ASSERT(err>=0);

   uart_app_v.uartTxLen = 0;
   uart_app_v.uartTxDelay = GUI_COMM_DELAY;
}

// Function sends data to the GUI appropriate for the given commandID.
void guiCommSendData(INT8U commandID, void* dataStructPtr)
{
   // get pointer to temp buffer
   INT8U commandData[MAX_UART_PACKET_SIZE];
   INT8U* bufferPtrPtr = commandData;

   // verify we know what this command is.
   ASSERT(commandID < GUI_COMM_NUM_CMD_IDS);

   // write commandID to buffer
   WRITE_HEX_BYTES(&bufferPtrPtr, commandID);
   
   // populate the data for this commandID.
   if(gui_comm_cmd_id_table[commandID] != NULL) gui_comm_cmd_id_table[commandID](&bufferPtrPtr, dataStructPtr);

   // copy data to Tx buffer.
   // note we don't want a higher priority task to call guiCommSendData while we are in this
   // so the OS/Ints are stopped.
   lockData();
   INT16U msgLength = min(bufferPtrPtr - commandData, MAX_UART_PACKET_SIZE - uart_app_v.uartTxLen); // do not allow overwriting buffer.
   memcpy(&uart_app_v.uartTxBuffer[uart_app_v.uartTxLen], commandData, msgLength);
   uart_app_v.uartTxLen += msgLength;
   unlockData();

   // post semaphore to start Tx Task.
   INT8U osErr = OSSemPost(uart_app_v.uartTxSem);
   ASSERT(osErr == OS_ERR_NONE);
   
   if(sendGuiDataToManager) // if we also want to send the response to a manager on the network
   {
      join_app_vars.sendMessageLength = msgLength;
      memcpy(join_app_vars.sendBuffer, commandData, join_app_vars.sendMessageLength);
      
      INT8U osErr;
      osErr = OSSemPost(join_app_vars.sendSem); // send via radio
      ASSERT(osErr == OS_ERR_NONE);
      
      sendGuiDataToManager = FALSE;
   }
}

// Wait until UART TX buffer is unlocked and free for editing
// Getting through the pend re-enables the lock
static void lockData() {
   INT8U      osErr;

   OSSemPend(uart_app_v.uartTxDataLock, 0, &osErr);
   ASSERT(osErr == OS_ERR_NONE);
}

// Enable modifications to the UART TX buffer
static void unlockData() {
   OSSemPost(uart_app_v.uartTxDataLock);
}

// Parse the received UART data to decode the command and data
void parseReceivedData(INT32U length, INT8U* dataBuffer, INT8U receivedFromGUI) 
{                                 
   INT32U i = 0;
   while(i < length)
   {
      INT8U commandChar = dataBuffer[i++];

      if(commandChar == 'M') { // M = Mode, the GUI is asking the mote to confirm connection and also asking for current mode info
         guiCommSendData(GUI_COMM_ID_SLAVE_MODE, NULL);
      }
      else if(commandChar == 'D') { // D = Display, set mode
         INT8U newDisplayModeIndex = ASCII2INT(dataBuffer[i++]);

         if(newDisplayModeIndex != 3) { // then we update the display mode
            if(displayMode != newDisplayModeIndex)
              updateType = UPDATE_WITH_STAGES; // always update in stages for GUI button press
            
            displayMode = newDisplayModeIndex;
            displayCurrentScreen(TRUE);
         }
         else { // no update, the GUI just wants data
            sendScreenUpdate();
         }
      }
      else if(commandChar == 'A') { // automatic data send
         parseReceivedNetworkData(dataBuffer, &i, !receivedFromGUI); // uses generic function because this can also be set over the network
      }
      else if(commandChar == 'R') { // reset mote
         INT8U rc;
         dnm_ucli_printf("RESET SOFTWARE\n");
         dnm_loc_resetCmd(&rc); // mote waits ~5 seconds and then resets
      }
      else if(commandChar == 'S') { // set usb SLAVE mode
         commandChar = dataBuffer[i++];
         if(commandChar == 'M') { // manual
            i++;
            runStandardChecks();
            guiCommSendData(GUI_COMM_ID_ALL, NULL);
            usbSlaveMode = -1;
         }
         else if(commandChar == 'N') { // normal (not a slave)
            usbSlaveMode = 0;
         }
      }
      else if(commandChar == 'N') { // network
         commandChar = dataBuffer[i++];

         if(commandChar == 'S') { // set

            commandChar = dataBuffer[i++];

            if(commandChar == 'J') { // join
               network_config.joinNetwork = dataBuffer[i++];
               setJoinStatus(network_config.joinNetwork);
            }
            else if(commandChar == 'N') { // netID
               network_config.netID = unpackHexBytes16(dataBuffer, &i);
               setNetID();
            }
            syncToConfigFile(); // save the values to the config file
         }
         else if(commandChar == 'G') { // get
            sendNetworkConfig();
         }
      }
      else if(commandChar == 'T') { // transmit the received USB message

         join_app_vars.sendMessageLength = dataBuffer[i++];

         for(int hexStringIndex = 0; hexStringIndex < join_app_vars.sendMessageLength; hexStringIndex++) {
            join_app_vars.sendBuffer[hexStringIndex] = dataBuffer[i++];
         }
         
         INT8U      osErr;
         osErr = OSSemPost(join_app_vars.sendSem); // send via radio
         ASSERT(osErr == OS_ERR_NONE);
      }
      else if(commandChar == 'U') { // we just connected via USB, send display info
         OSTimeDly(150); // gui needs some time or it will miss this command during startup
         guiCommSendData(GUI_COMM_ID_NETWORK_AUTO_VAR, NULL);
         OSTimeDly(200);
         sendScreenUpdate(); // send display info for the current mode
         OSTimeDly(50);
         guiCommSendData(GUI_COMM_ID_VERSION, NULL);
      }
      else if(commandChar == 'C') { // clear CCs
         clearCCs();
      }
      else if(commandChar == 'I') { // send I2C data
         INT8U address = dataBuffer[i++];
         INT8U numCommandBytes = dataBuffer[i++];
         INT8U i2c_buffer[I2C_PAYLOAD_LENGTH];
         for(INT8U hexStringIndex = 0; hexStringIndex < min(numCommandBytes, sizeof(i2c_buffer)); hexStringIndex++) {
            i2c_buffer[hexStringIndex] = dataBuffer[i++];
         }
         I2CWrite(i2c_buffer, address, numCommandBytes);
      }
      else if(commandChar == 'W') { // toggle 'wireless' mode
         if(dataBuffer[i++] == 1)
            sendGuiDataToManager = TRUE;
         else
            sendGuiDataToManager = FALSE;
      }
      else if(commandChar == 'V') { // get version
         guiCommSendData(GUI_COMM_ID_VERSION, NULL);
      }
   }
}

// Returns an INT32U from a buffer
INT16U unpackHexBytes16(INT8U* buffer, INT8U* startIndex)
{
  INT16U value;
  value = buffer[(*startIndex)++] << 8;
  value |= buffer[(*startIndex)++];
  
  return value;
}

// Returns an INT32U from a buffer
INT32U unpackHexBytes32(INT8U* buffer, INT8U* startIndex)
{
  INT32U value;
  value = buffer[(*startIndex)++] << 24;
  value |= buffer[(*startIndex)++] << 16;
  value |= buffer[(*startIndex)++] << 8;
  value |= buffer[(*startIndex)++];
  
  return value;
}

// Convert program status information into two bytes to be sent to the GUI
INT16U generateProgramStatusBytes(INT8U updateAllGuiDataCommand)
{
   INT16U programStatusBytes = displayMode; // occupies 2 LSBs
   if(waitingToUpdate)
     programStatusBytes |= 0x04;
   if(autoUpdate)
     programStatusBytes |= 0x08;
   if(USBPowerPresent)
     programStatusBytes |= 0x10;
   if(powerSource)
     programStatusBytes |= 0x20;

   programStatusBytes |= joinStatus << 6; // 2 MSBs

   if(writeEPD)
     programStatusBytes |= 0x100;

   if(ccClearedGuiFlag && updateAllGuiDataCommand) // if this is the 'update all' command, send a flag indicating if the CCs were just cleared
   {
     programStatusBytes |= 0x200;
     ccClearedGuiFlag = FALSE; // only send this once per CC clear, then clear flag
   }
   
   return programStatusBytes;
}

// Adds an INT8U to a buffer
void writeHexBytes8(INT8U** buffer, INT8U value)
{
   (*buffer)[0] = value;
   *buffer += sizeof(value);
   return;
}

// Adds an INT16U to a buffer
void writeHexBytes16(INT8U** buffer, INT16U value)
{
   (*buffer)[0] = (INT8U)((value >> 8) & 0xFF);
   (*buffer)[1] = (INT8U)(value & 0xFF);
   *buffer += sizeof(value);
   return;
}

// Adds an INT32U to a buffer
void writeHexBytes32(INT8U** buffer, INT32U value)
{
   (*buffer)[0] = (INT8U)((value >> 24) & 0xFF);
   (*buffer)[1] = (INT8U)((value >> 16) & 0xFF);
   (*buffer)[2] = (INT8U)((value >> 8) & 0xFF);
   (*buffer)[3] = (INT8U)(value & 0xFF);
   *buffer += sizeof(value);
   return;
}

// Adds an INT64U to a buffer
void writeHexBytes64(INT8U** buffer, INT64U value)
{
   (*buffer)[0] = (INT8U)((value >> 56) & 0xFF);
   (*buffer)[1] = (INT8U)((value >> 48) & 0xFF);
   (*buffer)[2] = (INT8U)((value >> 40) & 0xFF);
   (*buffer)[3] = (INT8U)((value >> 32) & 0xFF);
   (*buffer)[4] = (INT8U)((value >> 24) & 0xFF);
   (*buffer)[5] = (INT8U)((value >> 16) & 0xFF);
   (*buffer)[6] = (INT8U)((value >> 8) & 0xFF);
   (*buffer)[7] = (INT8U)(value & 0xFF);
   *buffer += sizeof(value);
   return;
}

// Send network data received from the manager to the GUI
// Populates data for the GUI_COMM_ID_RADIO_DATA command
void guiCommIdRadioData(INT8U** bufferPtrPtr, void* dataStructPtr)
{
   gui_comm_id_radio_data_t* struct_ptr = (gui_comm_id_radio_data_t*) dataStructPtr;
   INT8U bytesReceived = struct_ptr->length - sizeof(dn_api_loc_notif_received_t);
   INT8U paddedSpaces = 32 - bytesReceived;
   WRITE_HEX_BYTES(bufferPtrPtr, bytesReceived);
   memcpy(*bufferPtrPtr, 0, paddedSpaces); // pad all leading bytes with zeros
   *bufferPtrPtr += paddedSpaces;
   memcpy(*bufferPtrPtr, struct_ptr->rxFrame->data, bytesReceived);
   *bufferPtrPtr += bytesReceived;
}

// Send slave mode data to the GUI
// Populates data for the GUI_COMM_ID_SLAVE_MODE command
void guiCommIdSlaveMode(INT8U** bufferPtrPtr, void* dataStructPtr)
{ 
   (void)(dataStructPtr);
   WRITE_HEX_BYTES(bufferPtrPtr, usbSlaveMode);
}

// Send all standard status data to the GUI
// Populates data for the GUI_COMM_ID_ALL command
void guiCommIdAll(INT8U** bufferPtrPtr, void* dataStructPtr)
{ 
   (void)(dataStructPtr);
   updateAllCCData();
   WRITE_HEX_BYTES(bufferPtrPtr, generateProgramStatusBytes(TRUE));
   WRITE_HEX_BYTES(bufferPtrPtr, getEHData());
   WRITE_HEX_BYTES(bufferPtrPtr, getSupplyV());
   WRITE_HEX_BYTES(bufferPtrPtr, getTemp());
   WRITE_HEX_BYTES(bufferPtrPtr, getCCData(PRI,CHG));
   WRITE_HEX_BYTES(bufferPtrPtr, getCCData(PRI,VLT));
   WRITE_HEX_BYTES(bufferPtrPtr, getCCData(SEC,CHG));
   WRITE_HEX_BYTES(bufferPtrPtr, getCCData(SEC,VLT));
   WRITE_HEX_BYTES(bufferPtrPtr, getSystemTime()); //time stamp
}

// Send data for a summary screen update to the GUI
// Populates data for the GUI_COMM_ID_SIMPLE_SCREEN command
void guiCommIdSimpleScreen(INT8U** bufferPtrPtr, void* dataStructPtr)
{ 
   gui_comm_id_summary_screen_t* struct_ptr = (gui_comm_id_summary_screen_t*) dataStructPtr;
   WRITE_HEX_BYTES(bufferPtrPtr, struct_ptr->currentEHData);
}

// Send data for an IC status screen update to the GUI
// Populates data for the GUI_COMM_ID_STATUS_SCREEN command
void guiCommIdStatusScreen(INT8U** bufferPtrPtr, void* dataStructPtr)
{ 
   gui_comm_id_status_screen_t* struct_ptr = (gui_comm_id_status_screen_t*) dataStructPtr;
   WRITE_HEX_BYTES(bufferPtrPtr, struct_ptr->currentEHData);
   WRITE_HEX_BYTES(bufferPtrPtr, struct_ptr->currentTemperature);
}

// Send data for a bat status screen update to the GUI
// Populates data for the GUI_COMM_ID_BAT_SCREEN command
void guiCommIdBatScreen(INT8U** bufferPtrPtr, void* dataStructPtr)
{ 
   gui_comm_id_bat_screen_t* struct_ptr = (gui_comm_id_bat_screen_t*) dataStructPtr;
   WRITE_HEX_BYTES(bufferPtrPtr, generateProgramStatusBytes(FALSE));
   WRITE_HEX_BYTES(bufferPtrPtr, struct_ptr->currentTemperature);
   WRITE_HEX_BYTES(bufferPtrPtr, struct_ptr->lastChargesDisplayed[PRI]);
   WRITE_HEX_BYTES(bufferPtrPtr, struct_ptr->lastCurrentsDisplayed[PRI]);
   WRITE_HEX_BYTES(bufferPtrPtr, struct_ptr->lastVoltagesDisplayed[PRI]);
   WRITE_HEX_BYTES(bufferPtrPtr, struct_ptr->lastChargesDisplayed[SEC]);
   WRITE_HEX_BYTES(bufferPtrPtr, struct_ptr->lastCurrentsDisplayed[SEC]);
   WRITE_HEX_BYTES(bufferPtrPtr, struct_ptr->lastVoltagesDisplayed[SEC]);
}

// Send network configuration data to the GUI
// Populates data for the GUI_COMM_ID_NETWORK_CONFIG command
void guiCommIdNetworkConfig(INT8U** bufferPtrPtr, void* dataStructPtr)
{ 
   gui_comm_id_network_config_t* struct_ptr = (gui_comm_id_network_config_t*) dataStructPtr;
   WRITE_HEX_BYTES(bufferPtrPtr, network_config.joinNetwork);
   WRITE_HEX_BYTES(bufferPtrPtr, struct_ptr->netID2);
   WRITE_HEX_BYTES(bufferPtrPtr, struct_ptr->netID3);
   memcpy(*bufferPtrPtr, network_config.macAddress, sizeof(network_config.macAddress));
   *bufferPtrPtr += sizeof(network_config.macAddress);
}

// Send a data measurement back to the GUI as a test (rather than to the manager over the network)
// Populates data for the GUI_COMM_ID_NETWORK_TEST command
void guiCommIdNetworkTest(INT8U** bufferPtrPtr, void* dataStructPtr)
{ 
   const INT8U networkTestMsgLength = 5; //1+4 bytes because dataToSend in sendAutoData() is 32-bit [network.c]
   (void)(dataStructPtr);
   memcpy(*bufferPtrPtr, join_app_vars.sendBuffer, networkTestMsgLength);
   *bufferPtrPtr += networkTestMsgLength;
}

// Send send the network measurement configuration to the GUI
// Populates data for the GUI_COMM_ID_NETWORK_AUTO_VAR command
void guiCommIdNetworkAutoVar(INT8U** bufferPtrPtr, void* dataStructPtr)
{ 
   (void)(dataStructPtr);
   WRITE_HEX_BYTES(bufferPtrPtr, auto_send_data_vars.repeatSend);
   WRITE_HEX_BYTES(bufferPtrPtr, auto_send_data_vars.sendInterval);
   WRITE_HEX_BYTES(bufferPtrPtr, auto_send_data_vars.data);
   WRITE_HEX_BYTES(bufferPtrPtr, auto_send_data_vars.i2cAddress);
   WRITE_HEX_BYTES(bufferPtrPtr, auto_send_data_vars.optData);
   WRITE_HEX_BYTES(bufferPtrPtr, auto_send_data_vars.i2cNumBytesReceived);
}

void guiCommIdVersion(INT8U** bufferPtrPtr, void* dataStructPtr)
{
   (void)(dataStructPtr);
   WRITE_HEX_BYTES(bufferPtrPtr, (INT8U)APP_VER_MAJOR);
   WRITE_HEX_BYTES(bufferPtrPtr, (INT8U)APP_VER_MINOR);
   WRITE_HEX_BYTES(bufferPtrPtr, (INT8U)APP_VER_PATCH);
}
