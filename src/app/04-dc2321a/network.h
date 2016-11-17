/*
Copyright (c) 2016, Dust Networks.  All rights reserved.
*/

#ifndef NETWORK_H
#define NETWORK_H

#include "app_task_cfg.h"
#include "dn_typedef.h"
#include "loc_task.h"
#include "cli_task.h"

//=========================== enums  ==========================================
typedef enum {NOT_JOINED, JOINING, JOINED, RESET} join_status_t;

typedef enum { // These are the data packets that can be sent to the network.
   NETWORK_CMD_IC_STATUS,      // 0
   NETWORK_CMD_SUPPLY_VOLTAGE, // 1
   NETWORK_CMD_TEMPERATURE,    // 2
   NETWORK_CMD_PRI_VOLTAGE,    // 3
   NETWORK_CMD_PRI_CURRENT,    // 4
   NETWORK_CMD_PRI_CHARGE,     // 5
   NETWORK_CMD_SEC_VOLTAGE,    // 6
   NETWORK_CMD_SEC_CURRENT,    // 7
   NETWORK_CMD_SEC_CHARGE,     // 8
   NETWORK_CMD_TIME,           // 9
   NETWORK_CMD_MOTE_CHARGE,    // 10
   NETWORK_CMD_ADC_0,          // 11
   NETWORK_CMD_ADC_1,          // 12
   NETWORK_CMD_ADC_2,          // 13
   NETWORK_CMD_ADC_3,          // 14
   NETWORK_CMD_I2C_DATA,       // 15
   NETWORK_CMD_INCREMENT,      // 16
   NETWORK_NUM_CMDS
} network_cmd_t;

//=========================== defines =========================================
typedef struct
{
   INT8U joinNetwork; //JOIN_YES or JOIN_NO
   dn_netid_t netID;
   INT8U macAddress[8];
} network_config_t;


#define NET_ID 123 // network ID (this was the default of the manager)
#define NET_BANDWIDTH 9000 // radio bandwidth (this was the default on the join app)
#define PAYLOAD_LENGTH 32 // for network join

//=========================== structs =========================================
typedef struct // stores data which tells the mote what messages to send to the manager (and when it should send)
{
   INT8U       repeatSend;          // 0 = do not send on interval, 1 = send auto data
   INT64U      lastSendTime;        // the value of getSystemTime() when data last sent.
   INT32U      sendInterval;        // the interval (in seconds) between data sends
   INT8U       data;                // index to tell the type of data
   INT8U       i2cAddress;          // i2c address used only for i2c reads
   INT8U       i2cNumBytesSent;     // number of bytes to send for i2c read
   INT8U       i2cNumBytesReceived; // number of bytes that will be received
   INT32U      optData;             // optional data holder used for some functions
} auto_send_data_vars_t;

typedef struct // stores data for sending messages over the network
{
   OS_EVENT*       joinedSem;
   OS_EVENT*       sendSem;
   char            sendBuffer[PAYLOAD_LENGTH];
   INT8U           sendMessageLength;
   OS_STK          sendTaskStack[TASK_APP_SEND_STK_SIZE];
} join_app_vars_t;

//=========================== global variables ================================
extern network_config_t network_config;
extern auto_send_data_vars_t auto_send_data_vars;
extern join_app_vars_t join_app_vars;
extern join_status_t joinStatus;

extern INT8U sendGuiDataToManager;

//=========================== prototypes ======================================
void network_init(void);
void network_init2(void);

void setJoinStatus(INT8U shouldJoin);
void syncToConfigFile(void);
void setNetID();

void sendNetworkConfig();
void sendAutoData(INT8U sendToNetwork);

void parseReceivedNetworkData(INT8U* dataBuffer, INT8U* i, INT8U sendToGUI);

#endif 
