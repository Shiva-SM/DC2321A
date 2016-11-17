/*
Copyright (c) 2016, Dust Networks.  All rights reserved.
*/

#ifndef COG_DISPLAY_H
#define COG_DISPLAY_H

#include "dn_typedef.h"
#include "cog_driver.h"
#include "cog_images.h"
#include "coulomb_counter.h"

//=========================== enums  ==========================================
typedef enum {
   COG_DISPLAY_MODE_SUMMARY, //summary displayMode
   COG_DISPLAY_MODE_EH,      //EH status displayMode
   COG_DISPLAY_MODE_BAT,     //battery status displayMode
   COG_DISPLAY_NUM_MODES
} cog_display_mode_t;

typedef enum {
   UPDATE_BASIC,             //just send one screen to the display
   UPDATE_WITH_STAGES,       //update with stages according to COG documentation
   UPDATE_WITH_FLASHES       //just flash the screen black and white a few times
} cog_display_update_t;

//=========================== defines =========================================
#define SCREEN_Q_LENGTH   2  //the length of the queue for screen updates

//=========================== structs =========================================
typedef struct
{
  cog_display_mode_t displayMode;  //the display mode of the screen to be painted
  INT16S temperature;              //temperature for IC status or BAT status screens
  INT8U data;                      //holds IC status, or for holds autoUpdate BAT status screen
  INT32S ccData[NUMCC][NUMDATA];   //holds all CC data for the BAT status screen
  INT8U updateType;                //whether or not the display write should update in stages or flash
} screen_data_t;

typedef struct
{
  OS_STK         displayUpdateTaskStack[TASK_APP_DISPLAY_UPDATE_STK_SIZE];
  OS_EVENT*      displayUpdateQ;
  void           *screenQ[SCREEN_Q_LENGTH];     //an array of pointers for the queue
  screen_data_t  screenData[SCREEN_Q_LENGTH];   //the actual screen data queue storage array
  INT8U          screenDataIndex;               //the last index into screenDataBuffer that was posted to Queue.
  screen_data_t  currentScreenData;             //a global screen_data_t that can be modified from anywhere
  screen_data_t  previousScreenData;            //the last displayed screen_data_t, used for writing in stages
} display_app_vars_t;

//=========================== global variables ================================
extern display_app_vars_t display_app_v;

// Data to tell how to update the display
extern INT8U displayMode;
extern INT8U updateType;

// Data to tell if the display can be updated
extern INT8U waitingToUpdate;
extern INT8U writeEPD;

// The last values displayed on the EPD
extern INT8U lastReadEHData;
extern INT8U lastDisplayedEHData;
extern INT16S lastTempDisplayed;

//=========================== prototypes ======================================
void postDisplayUpdate();

INT8U updateSummaryScreenData(INT8U ehdata, INT8U forceUpdate);
void updateBatStatus();

void cogDisplayInit(void);
void cogDisplayInit2(void);
INT8U checkCanUpdateDisplay(INT8U updateIsForced);
void sendScreenUpdate(void);
void checkWriteEPD(void);

#endif
