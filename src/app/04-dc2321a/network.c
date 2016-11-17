/**
Copyright (c) 2016, Dust Networks.  All rights reserved.

This file manages all of the network interaction.

Some network data is stored persistently in a configuration file.
This allows the mote to remember what it was doing in case it loses and regains power.

*/
#include <stdlib.h>
#include <string.h>
#include "loc_task.h"
#include "dn_system.h"
#include "dn_fs.h"        // for network config file
#include "dn_api_param.h" // for setting/getting parameters

#include "devices.h"
#include "gui_comm.h"
#include "coulomb_counter.h"
#include "cog_display.h"
#include "network.h"

//=========================== defines =========================================
#define APP_CONFIG_FILENAME "netCfg.cfg" // file name holding configuration
#define DEFAULT_JOIN_NETWORK JOIN_NO     // by default, do not join the network so that power consumption is lowest
#define DEFAULT_NETID 1228               // default dust netID is 1229, for the DC2321A/Gleanergy kit, the default is 1228
#define DEFAULT_UDPPORT 60000
#define DEFAULT_BANDWIDTH 9000

//=========================== prototypes ======================================
dn_error_t rxNotifCb(dn_api_loc_notif_received_t* rxFrame, INT8U length);
void loadConfigFile();

//=========================== variables =======================================
network_config_t network_config;           // the configuration of the network
auto_send_data_vars_t auto_send_data_vars; // settings to determine what data should be sent on an interval
join_app_vars_t join_app_vars;             // task/stack data for the network
join_status_t joinStatus = NOT_JOINED;     // the status of this mote on the network

INT8U sendGuiDataToManager = FALSE;
//=========================== tasks ===========================================
// Task which sends network data as it is posted
void sendTask(void* unused)
{
   dn_error_t      dnErr;
   INT8U           osErr;
   INT8U           pkBuf[sizeof(loc_sendtoNW_t)+PAYLOAD_LENGTH];
   loc_sendtoNW_t* pkToSend;
   INT8U           i;
   INT8U           rc;

   // wait for the loc_task to finish joining the network
   OSSemPend(join_app_vars.joinedSem, 0, &osErr);
   ASSERT(osErr==OS_ERR_NONE);

   dnm_ucli_printf("Done joining!\r\n");
   joinStatus = JOINED;

   while (1) {
      // wait for the semaphore to be posted
      OSSemPend(join_app_vars.sendSem, 0, &osErr);
      ASSERT (osErr == OS_ERR_NONE);

      // prepare packet to send
      pkToSend = (loc_sendtoNW_t*)pkBuf;
      pkToSend->locSendTo.socketId          = loc_getSocketId();
      pkToSend->locSendTo.destAddr          = DN_MGR_IPV6_MULTICAST_ADDR;
      pkToSend->locSendTo.destPort          = 60000;
      pkToSend->locSendTo.serviceType       = DN_API_SERVICE_TYPE_BW;
      pkToSend->locSendTo.priority          = DN_API_PRIORITY_MED;
      pkToSend->locSendTo.packetId          = 0xFFFF;

      // fill packet with data
      for (i=0; i < join_app_vars.sendMessageLength; i++) {
         pkToSend->locSendTo.payload[i] = join_app_vars.sendBuffer[i];
      }

      // send packet
      dnErr = dnm_loc_sendtoCmd(
         pkToSend,
         join_app_vars.sendMessageLength,
         &rc
      );
      ASSERT(dnErr==DN_ERR_NONE);
   }
}

// Create network task and semaphores
void network_init(void)
{
   INT8U osErr;

   // network join
   join_app_vars.joinedSem = OSSemCreate(0); // create semaphore for loc_task to indicate when joined
   join_app_vars.sendSem = OSSemCreate(0); // create semaphore for network send
   ASSERT (join_app_vars.sendSem != NULL);

   // initialize network notification task
   loc_task_init(JOIN_NO, NULL, 60000, join_app_vars.joinedSem, 9000, NULL);
   
   dnm_loc_registerRxNotifCallback(rxNotifCb); // register callback for received packet

   // create task for network join
   osErr = OSTaskCreateExt(
      sendTask,
      (void *) 0,
      (OS_STK*) (&join_app_vars.sendTaskStack[TASK_APP_SEND_STK_SIZE - 1]),
      TASK_APP_SEND_PRIORITY,
      TASK_APP_SEND_PRIORITY,
      (OS_STK*) join_app_vars.sendTaskStack,
      TASK_APP_SEND_STK_SIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR
   );
   ASSERT(osErr==OS_ERR_NONE);
   OSTaskNameSet(TASK_APP_SEND_PRIORITY, (INT8U*)TASK_APP_SEND_NAME, &osErr);
   ASSERT(osErr==OS_ERR_NONE);
}

// Initialize network configuration
void network_init2(void)
{
   loadConfigFile(); // load stored configuration settings
   setJoinStatus(network_config.joinNetwork);
}

// Tell the mote if it should join, not join, or leave the network
// Leaving the network results in a program reset
void setJoinStatus(INT8U shouldJoin)
{
   INT8U rc;
   if(shouldJoin && joinStatus == NOT_JOINED) { // if should join and not already joined.
      joinStatus = JOINING;
      dnm_loc_joinCmd(&rc);
      ASSERT(rc == DN_ERR_NONE);
   }
   else if(joinStatus == JOINING) { // if trying to connect.
      joinStatus = RESET;
      dnm_loc_resetCmd(&rc);
      ASSERT(rc == DN_ERR_NONE);
   }
   else if(joinStatus == JOINED) { // if already joined.
      joinStatus = RESET;
      dnm_loc_disconnectCmd(&rc);
      ASSERT(rc == DN_ERR_NONE);
   }
}

// Save the current network_config and auto_send_data_vars into persistent memory
void syncToConfigFile()
{
   dn_error_t          dnErr;
   dn_fs_handle_t      configFileHandle;

   // open file
   configFileHandle = dn_fs_open(
      APP_CONFIG_FILENAME,
      DN_FS_OPT_CREATE,
      sizeof(network_config_t),
      DN_FS_MODE_OTH_RW
   );
   ASSERT(configFileHandle >= 0);

   // write file
   INT32U offset = 0;
   dnErr = dn_fs_write(
      configFileHandle,
      offset, // offset
      (INT8U*)&(network_config),
      sizeof(network_config_t)
   );
   ASSERT(dnErr >= 0);

   offset += sizeof(network_config_t);

   dnErr = dn_fs_write(
      configFileHandle,
      offset, // offset
      (INT8U*)&(auto_send_data_vars),
      sizeof(auto_send_data_vars_t)
   );
   ASSERT(dnErr >= 0);

   // close file
   dn_fs_close(configFileHandle);
}

// Set the netID based on network_config.netID
void setNetID()
{
   dn_error_t dnErr;
   INT8U rc;

   INT8U netIDBytes[2];
   netIDBytes[0] = (INT8U)(network_config.netID >> 8);
   netIDBytes[1] = (INT8U)(network_config.netID);
   dnErr = dnm_loc_setParameterCmd(
      DN_API_PARAM_NETID,           // paramId
      netIDBytes,                   // payload
      sizeof(dn_netid_t),           // payload length
      &rc);                         // return code
   dnm_ucli_printf("err = %d\r\n", dnErr);
   ASSERT(dnErr == DN_ERR_NONE);
}

// Take a data measurement of the type of data indicated by auto_send_data_vars.data
// If sendToNetwork is true, send the measurement to the manager
// If sendToNetwork is false, this was just a sample and the data should be sent back to the GUI
void sendAutoData(INT8U sendToNetwork) //send data via radio to be received by API explorer
{
   if((joinStatus != JOINED) && (sendToNetwork != 0)) // if we are not joined to the network, don't do anything (UNLESS WE ARE JUST SENDING TO THE GUI)
      return;

   INT32U dataToSend;

   // take a data measurement and store the data in dataToSend
   switch(auto_send_data_vars.data)
   {
      case NETWORK_CMD_IC_STATUS: // encoded into bits
      {
         INT8U ICStatus = getEHData();
         dataToSend = ICStatus;
         break;
      }
      case NETWORK_CMD_SUPPLY_VOLTAGE: // in mVs
      {
         dataToSend = (INT32U)getSupplyV() / 10; // divided by 10 to make it in mV (rather than 100uV)
         break;
      }
      case NETWORK_CMD_TEMPERATURE: // in hundredths of a degree C
      {
         dataToSend = (INT32U)getTemp();
         break;
      }
      case NETWORK_CMD_PRI_VOLTAGE: // in mV
      {
         updateAllCCData();
         dataToSend = getCCData(PRI,VLT) / UV_PER_MV; // divided by UV_PER_MV to make it in mV
         break;
      }
      case NETWORK_CMD_PRI_CURRENT: // in uA
      {
         updateAllCCData();
         dataToSend = getCCData(PRI,CUR) / UA_PER_MA; // divided by UA_PER_MA to make it in mA
         break;
      }
      case NETWORK_CMD_PRI_CHARGE: // in uC
      {
         updateAllCCData();
         dataToSend = getCCData(PRI,CHG);
         break;
      }
      case NETWORK_CMD_SEC_VOLTAGE: // in mV
      {
         updateAllCCData();
         dataToSend = getCCData(SEC,VLT) / UV_PER_MV; // divided by UV_PER_MV to make it in mV
         break;
      }
      case NETWORK_CMD_SEC_CURRENT: // in uA
      {
         updateAllCCData();
         dataToSend = getCCData(SEC,CUR) / UA_PER_MA; // divided by UA_PER_MA to make it in mA
         break;
      }
      case NETWORK_CMD_SEC_CHARGE: // in uC
      {
         updateAllCCData();
         dataToSend = getCCData(SEC,CHG);
         break;
      }
      case NETWORK_CMD_TIME: // in seconds
      {
         dataToSend = getSystemTime() * MS_PER_SEC / TIMER_TICKS_PER_SEC;
         break;
      }
      case NETWORK_CMD_MOTE_CHARGE: // in mC (since this is the smallest resolution we can get anyway)
      {
         dn_error_t dnErr;
         INT32U charge;   // charge in mC since boot

         dnErr = dn_open(DN_CHARGE_DEV_ID, NULL, 0);
         ASSERT(dnErr == DN_ERR_NONE);
         dnErr = dn_read(DN_CHARGE_DEV_ID, &charge, sizeof(charge));
         ASSERT(dnErr >= DN_ERR_NONE);
         dnErr = dn_close(DN_CHARGE_DEV_ID);
         ASSERT(dnErr == DN_ERR_NONE);
         
         dataToSend = charge;
         break;
      }
      case NETWORK_CMD_ADC_0: // in mV
      case NETWORK_CMD_ADC_1:
      case NETWORK_CMD_ADC_2:
      case NETWORK_CMD_ADC_3:
      {
         dataToSend = (INT32U)readADC(auto_send_data_vars.data - NETWORK_CMD_ADC_0) / 10; // divided by 10 to make it in mV (rather than 100uV)
         break;
      }
      case NETWORK_CMD_I2C_DATA: // in raw data
      {
         INT8U readCommand[16];
         for(INT8U i = 0; i < auto_send_data_vars.i2cNumBytesSent; i++) {
           INT8U shiftValue = auto_send_data_vars.i2cNumBytesSent - 1 - i;
           readCommand[i] = (auto_send_data_vars.optData >> shiftValue) & 0xFF;
         }
         
         dnm_ucli_printf("readCommand[0] = 0x%02x\r\n", readCommand[0]);
         
         INT8U i2c_buffer_ptr[4];
         I2CWriteAndRead(auto_send_data_vars.i2cAddress, readCommand, auto_send_data_vars.i2cNumBytesSent, i2c_buffer_ptr, auto_send_data_vars.i2cNumBytesReceived);

         dataToSend = 0;
         for(INT8U i = 0; i < auto_send_data_vars.i2cNumBytesReceived; i++) {
            dataToSend <<= 8;
            dataToSend += i2c_buffer_ptr[i];
         }
         break;
      }
   case NETWORK_CMD_INCREMENT: // in discrete increments
      {
         auto_send_data_vars.optData += 1;
         dataToSend = auto_send_data_vars.optData;
         break;
      }
   }

   // package data to send to the network
   join_app_vars.sendMessageLength = 5; // hardcoded # of bytes per message
   // actual message length is set in guiCommIdNetworkTest() if data is going back to GUI
   join_app_vars.sendBuffer[0] = auto_send_data_vars.data;
   join_app_vars.sendBuffer[1] = (INT8U)((dataToSend >> 24) & 0xFF);
   join_app_vars.sendBuffer[2] = (INT8U)((dataToSend >> 16) & 0xFF);
   join_app_vars.sendBuffer[3] = (INT8U)((dataToSend >> 8) & 0xFF);
   join_app_vars.sendBuffer[4] = (INT8U)(dataToSend & 0xFF);

   if(sendToNetwork) { // send to manager via network
      INT8U osErr;
      osErr = OSSemPost(join_app_vars.sendSem); // send via radio
      ASSERT(osErr == OS_ERR_NONE);
   }
   else { // just a test, send back to GUI
      guiCommSendData(GUI_COMM_ID_NETWORK_TEST, NULL);
   }
}

// Send the network configuration to the GUI
void sendNetworkConfig()
{
   INT8U rc;
   dn_error_t dnErr;
   INT8U respLen;

   INT8U netID[4]; // the data goes into bytes starting at array index 2
   dnErr = dnm_loc_getParameterCmd(
      DN_API_PARAM_NETID,                 // paramId
      netID,                              // payload
      0,                                  // txPayloadLen
      &respLen,                           // rxPayloadLen
      &rc                                 // rc
   );
   ASSERT(dnErr==DN_ERR_NONE);
   ASSERT(rc==DN_ERR_NONE);

   gui_comm_id_network_config_t data_struct;
   data_struct.netID2 = netID[2];
   data_struct.netID3 = netID[3];
   guiCommSendData(GUI_COMM_ID_NETWORK_CONFIG, &data_struct);
}

// Network received transmission callback function
// Just sends the received data back to the GUI
dn_error_t rxNotifCb(dn_api_loc_notif_received_t* rxFrame, INT8U length)
{
   INT8U i;

   gui_comm_id_radio_data_t data_struct;
   data_struct.rxFrame = rxFrame;
   data_struct.length = length;
   
   if(data_struct.length != 0) { // if data was received
     if(data_struct.rxFrame->data[0] == 'W') { // only data starting with a 'W' will go through the parser
         parseReceivedData(data_struct.length, data_struct.rxFrame->data, FALSE);
     }
   }
   
   // send data to GUI
   guiCommSendData(GUI_COMM_ID_RADIO_DATA, &data_struct);
   
   return DN_ERR_NONE;
}

// Parse the data received over the network to decode the command
void parseReceivedNetworkData(INT8U* dataBuffer, INT8U* i, INT8U sendToGUI)
{
   INT8U commandChar = dataBuffer[(*i)++];
   
   // 'S' means stop repeat send, 'R' means repeat send, 'N' means just send once
   if(commandChar == 'S') { // S means stop, no other data passed (shouldRepeat set to 0)
      auto_send_data_vars.repeatSend = 0;
   }
   else { // new data to store
      INT8U shouldRepeat = 0; // should the message be repeated on a regular interval
      if(commandChar == 'R') shouldRepeat = 1;

      INT8U lastData = auto_send_data_vars.data; // store this to check if we are continuing an increment

      auto_send_data_vars.data = dataBuffer[(*i)++];

          // after the scaler and before the interval: IF data is NETWORK_CMD_I2C_DATA, that means that we are receiving additional i2c data: [address] + [numCommandBytes] + [command]
         if(auto_send_data_vars.data == NETWORK_CMD_I2C_DATA) // if not i2c where optData holds the address, set to 0 (needs to be 0 for increment function, doesn't matter for most)
         {
            auto_send_data_vars.i2cAddress = dataBuffer[(*i)++];
            auto_send_data_vars.i2cNumBytesSent = dataBuffer[(*i)++];
            auto_send_data_vars.optData = unpackHexBytes32(dataBuffer, i);
            auto_send_data_vars.i2cNumBytesReceived = dataBuffer[(*i)++];
         } // end additional data read for i2c

      if(shouldRepeat) {
         auto_send_data_vars.sendInterval = unpackHexBytes32(dataBuffer, i);
         auto_send_data_vars.repeatSend = 1;
      }
      else {
         auto_send_data_vars.repeatSend = 0;
      }

      if(auto_send_data_vars.data != NETWORK_CMD_I2C_DATA) { // if not i2c where optData holds the address, set to 0 (needs to be 0 for increment function, doesn't matter for most)
        if(auto_send_data_vars.data != lastData) // do not reset if we are continuing to a command
            auto_send_data_vars.optData = 0;
      }

      if(commandChar != 'T') { // if this is not a test, send data via radio
         sendAutoData(1);
         auto_send_data_vars.lastSendTime = getSystemTime(); // used for interval
      }
      else { // this is just a test, send data back to GUI
         sendAutoData(0);
      }
   }
   syncToConfigFile(); // save the auto send data to the config file
   
   if(sendToGUI) guiCommSendData(GUI_COMM_ID_NETWORK_AUTO_VAR, NULL); // only send the data back to the GUI if it was not sent from the GUI in the first place
}

// Recall stored config data file
// OR, if none, create default config data file
void loadConfigFile()
{
   dn_error_t        dnErr;
   dn_fs_handle_t    configFileHandle;

   configFileHandle = dn_fs_find(APP_CONFIG_FILENAME);

   if (configFileHandle >= 0) {
      // file found: read it

      // open file
      configFileHandle = dn_fs_open(
         APP_CONFIG_FILENAME,
         DN_FS_OPT_CREATE,
         sizeof(network_config_t),
         DN_FS_MODE_OTH_RW
      );
      ASSERT(configFileHandle >= 0);

      // read file
      INT32U offset = 0;
      dnErr = dn_fs_read(
         configFileHandle,
         offset, // offset
         (INT8U*)&(network_config),
         sizeof(network_config_t)
      );
      ASSERT(dnErr>=0);

      offset += sizeof(network_config_t);

      dnErr = dn_fs_read(
         configFileHandle,
         offset, // offset
         (INT8U*)&(auto_send_data_vars),
         sizeof(auto_send_data_vars_t)
      );
      ASSERT(dnErr>=0);
      
      // close file
      dn_fs_close(configFileHandle);
   }
   else { // file not found: create it
     
      // prepare file content
      network_config.joinNetwork = DEFAULT_JOIN_NETWORK;
      network_config.netID = DEFAULT_NETID;

      // if we are creating the file, set the netID to the default
      setNetID();
      
      auto_send_data_vars.repeatSend = 0; // by default, do not repeat send

      // create file
      configFileHandle = dn_fs_open(
         APP_CONFIG_FILENAME,
         DN_FS_OPT_CREATE,
         sizeof(network_config_t) + sizeof(auto_send_data_vars_t),
         DN_FS_MODE_SHADOW
      );
      ASSERT(configFileHandle>=0);

      // write file
      INT32U offset = 0;
      dnErr = dn_fs_write(
         configFileHandle,
         offset,
         (INT8U*)&(network_config),
         sizeof(network_config_t)
      );
      ASSERT(dnErr >= 0);

      offset += sizeof(network_config_t);

      dnErr = dn_fs_write(
         configFileHandle,
         offset,
         (INT8U*)&(auto_send_data_vars),
         sizeof(auto_send_data_vars_t)
      );
      ASSERT(dnErr >= 0);

      // close file
      dn_fs_close(configFileHandle);
   }

   // regardless of whether file exists or not, load mac address here
   INT8U rc;
   INT8U respLen;
   INT8U moteInfoBuf[2+sizeof(dn_api_rsp_get_moteinfo_t)];
   dn_api_rsp_get_moteinfo_t* moteInfo;

   dnErr = dnm_loc_getParameterCmd(
      DN_API_PARAM_MOTEINFO,
      &moteInfoBuf,
      0,
      &respLen, 
      &rc
      );
   moteInfo = (dn_api_rsp_get_moteinfo_t*) (&moteInfoBuf[0]);
   memcpy(network_config.macAddress, moteInfo->serialNumber, sizeof(network_config.macAddress));
   
   auto_send_data_vars.lastSendTime = getSystemTime(); //set the lastSendTime to the currentTime so that sending restarts normally
}
