#ifndef GUI_COMM_H
#define GUI_COMM_H

#include "coulomb_counter.h"
#include "network.h"

//=========================== enums  ==========================================
typedef enum
{ // These are the data packets that can be sent to the GUI.
   GUI_COMM_ID_POKE,            // 0
   GUI_COMM_ID_RADIO_DATA,      // 1
   GUI_COMM_ID_SLAVE_MODE,      // 2
   GUI_COMM_ID_ALL,             // 3
   GUI_COMM_ID_SIMPLE_SCREEN,   // 4
   GUI_COMM_ID_STATUS_SCREEN,   // 5
   GUI_COMM_ID_BAT_SCREEN,      // 6
   GUI_COMM_ID_NETWORK_CONFIG,  // 7
   GUI_COMM_ID_NETWORK_TEST,    // 8
   GUI_COMM_ID_NETWORK_AUTO_VAR,// 9
   GUI_COMM_ID_10_SKIPPED,      // 10 (the number 10 is skipped because this is an ascii newline and can be confused with CLI messages)
   GUI_COMM_ID_VERSION,         // 11
   GUI_COMM_NUM_CMD_IDS
} gui_comm_cmd_id_t;

//=========================== structs =========================================
typedef struct
{ // struct of data used by GUI_COMM_ID_RADIO_DATA command id.
   dn_api_loc_notif_received_t* rxFrame;
   INT8U length;
} gui_comm_id_radio_data_t;

typedef struct
{ // struct of data used by GUI_COMM_ID_SIMPLE_SCREEN command id.
   INT8U currentEHData;
} gui_comm_id_summary_screen_t;

typedef struct
{ // struct of data used by GUI_COMM_ID_STATUS_SCREEN command id.
   INT8U currentEHData;
   INT16S currentTemperature;
} gui_comm_id_status_screen_t;

typedef struct
{ // struct of data used by GUI_COMM_ID_BAT_SCREEN command id.
   INT16S currentTemperature;
   INT32S lastChargesDisplayed[NUMCC];
   INT32S lastVoltagesDisplayed[NUMCC];
   INT32S lastCurrentsDisplayed[NUMCC];
} gui_comm_id_bat_screen_t;

typedef struct
{ // struct of data used by GUI_COMM_ID_NETWORK_CONFIG command id.
   INT8U netID2;
   INT8U netID3;
} gui_comm_id_network_config_t;

//=========================== global variables ================================
extern INT8U guiSentLastCommand;      // stores whether the last command was received from the GUI or a manager on the network

//=========================== prototypes ======================================
void guiCommInit();
void guiCommInit2();
void guiCommSendData(INT8U commandID, void* dataStructPtr);
INT16U unpackHexBytes16(INT8U* buffer, INT8U* startIndex);
INT32U unpackHexBytes32(INT8U* buffer, INT8U* startIndex);
void parseReceivedData(INT32U length, INT8U* dataBuffer, INT8U receivedFromGUI);

#endif