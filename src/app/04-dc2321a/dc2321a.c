/*
Copyright (c) 2016, Dust Networks.  All rights reserved.

This is the 'main' file with the p2_init() entry point, main loop, and button input management.

All display updates are triggered through the main loop which checks all measurement data on the board.
*/

#include <stdlib.h>
#include "app_task_cfg.h"
#include "dn_system.h"
#include "dn_gpio.h"
#include "dn_exe_hdr.h"
#include "Ver.h"
#include <string.h>

#include "dc2321a.h"
#include "coulomb_counter.h"
#include "gui_comm.h"
#include "cog_display.h"
#include "devices.h"
#include "network.h"

//=========================== definitions =====================================
typedef struct {
  OS_STK         mainloopTaskStack[TASK_APP_MAINLOOP_STK_SIZE];
} mainloop_app_vars_t;

typedef struct {
   OS_STK         gpioNotifTaskStack[TASK_APP_GPIONOTIF_STK_SIZE];
   INT32U         gpioNotifChannelBuf[1+DN_CH_ASYNC_RXBUF_SIZE(sizeof(dn_gpio_notif_t))/sizeof(INT32U)];
} gpio_app_vars_t;

#define BTN_CLICK_INTERVAL_TICKS (0.5 * TIMER_TICKS_PER_SEC) // the maximum amount of time between consecutive button clicks for a command
#define MAIN_LOOP_DELAY 1000                                 // loop delay is 1 second by default
#define MIN_SUPPLY_VOLTAGE_TO_UPDATE_EPD 32300               // display update threshold voltage in 0.1 millivolts (3.23V)

typedef enum {
   TASKS_MAINLOOP,
   TASKS_GPIONOTIF,
   TASKS_CCALERT,
   TASKS_UART_TX,
   TASKS_UART_RX,
   TASKS_SEND,
   TASKS_DISPLAY,
   NUM_TASKS
} TASKS_ENUM;

//=========================== variables =======================================
mainloop_app_vars_t mainloop_app_v;
gpio_app_vars_t gpio_app_v;

//=========================== prototypes ======================================
void mainLoopTask(void* unused);
void buttonInterruptTask(void* unused);

void runStandardChecks(); // the main function where we read inputs depending on display mode
void manageButtonInput(); // evaluates button input and decides what to do
void checkPowerSource();  // checks whether USB or EHVCC is the source

//=========================== global variables ==================================
// PROGRAM CONTROL
INT8U usbSlaveMode = FALSE;             // in slave mode, mote only checks data when prompted

// DISPLAY CONTROL
INT8U autoUpdate = FALSE;               // true if the battery status display should auto-update at a user-defined interval
INT16U autoUpdateDelay = 0;         // the interval at which the battery status display should auto-update
INT32U lastAutoUpdateTime;          // the last time the display was updated due to an autoUpdate

// USB POWER
INT8U USBPowerPresent = FALSE;          // boolean, do we have USB power? (assumed NO USB power as start-up state)
power_source_t powerSource = EHVCC; // what is the power source?

// BUTTON CONTROL
INT32U lastBtnPressTime = 0;       // last times the button was pressed down
INT32U lastBtnReleaseTime = 0;     // last times the button was released
INT8U btnPressCounter = 0;         // how many times the button has been clicked for this command
INT8U btnState = 0;                // current state of the button, 1 = down, 0 = up

//=========================== initialization ==================================
// This is the entry point in the application code
// Create all tasks and semaphores
int p2_init(void) {
   INT8U osErr;

   // initialize cli task
   cli_task_init("OCSDK", NULL);
   
   // initialize network
   network_init();

   // initialize GUI comm
   guiCommInit();

   // initialize coulomb counter
   coulombCounterInit();

   // initialize display
   cogDisplayInit();

   // create the task for mainLoopTask
   osErr  = OSTaskCreateExt(
      mainLoopTask,
      (void *)0,
      (OS_STK*)(&mainloop_app_v.mainloopTaskStack[TASK_APP_MAINLOOP_STK_SIZE-1]),
      TASK_APP_MAINLOOP_PRIORITY,
      TASK_APP_MAINLOOP_PRIORITY,
      (OS_STK*)mainloop_app_v.mainloopTaskStack,
      TASK_APP_MAINLOOP_STK_SIZE,
      (void *)0,
      OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR
   );
   ASSERT(osErr==OS_ERR_NONE);
   OSTaskNameSet(TASK_APP_MAINLOOP_PRIORITY, (INT8U*)TASK_APP_MAINLOOP_NAME, &osErr);
   ASSERT(osErr==OS_ERR_NONE);

   //create the button task
   osErr = OSTaskCreateExt(
      buttonInterruptTask,
      (void *) 0,
      (OS_STK*) (&gpio_app_v.gpioNotifTaskStack[TASK_APP_GPIONOTIF_STK_SIZE - 1]),
      TASK_APP_GPIONOTIF_PRIORITY,
      TASK_APP_GPIONOTIF_PRIORITY,
      (OS_STK*) gpio_app_v.gpioNotifTaskStack,
      TASK_APP_GPIONOTIF_STK_SIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR
   );
   ASSERT(osErr == OS_ERR_NONE);
   OSTaskNameSet(TASK_APP_GPIONOTIF_PRIORITY, (INT8U*)TASK_APP_GPIONOTIF_NAME, &osErr);
   ASSERT(osErr == OS_ERR_NONE);

   return 0;
}

// The main loop of the program
// - checks power status of the board
// - manages data input from internal and external sensors
// - tells the display when it needs to update
// - manages input from the button
void mainLoopTask(void* unused)
{
   dnm_ucli_printf("DC2321A app, ver %d.%d.%d\r\n", APP_VER_MAJOR, APP_VER_MINOR, APP_VER_PATCH);
  
   dn_error_t dnErr;

   //CONFIGURE DC2234A DATA GPIOs
   initGPIO(BAT_OFF_LTC3107, GPIO_STATE_INPUT);
   initGPIO(EH_ON_LTC3330, GPIO_STATE_INPUT);
   initGPIO(EH_ON_LTC3331, GPIO_STATE_INPUT);
   initGPIO(PGOOD_LTC3106, GPIO_STATE_INPUT);
   initGPIO(PGOOD_LTC3107, GPIO_STATE_INPUT);
   initGPIO(PGOOD_LTC3330, GPIO_STATE_INPUT);
   initGPIO(PGOOD_LTC3331, GPIO_STATE_INPUT);
   initGPIO(SRC, GPIO_STATE_INPUT);

   // initialize network
   network_init2();

   // initialize GUI comm
   guiCommInit2();

   // initialize coulomb counter
   coulombCounterInit2();

   // initialize display
   cogDisplayInit2();

   // initialize devices
   initBatADC();
   
   while(1) {
      if(!usbSlaveMode) { // standalone mode (not slave to GUI)
        runStandardChecks(); // run all checks
      }
      else { // slave to GUI mode, still need to check a few things
         manageButtonInput();
         checkUSBPower();
      }

      if(auto_send_data_vars.repeatSend) { // if we are sending on an interval
         if((getSystemTime() - auto_send_data_vars.lastSendTime) >= auto_send_data_vars.sendInterval * TIMER_TICKS_PER_SEC) {
            sendAutoData(TRUE);
            auto_send_data_vars.lastSendTime += (auto_send_data_vars.sendInterval * TIMER_TICKS_PER_SEC);
         }
      }

      OSTimeDly(MAIN_LOOP_DELAY);
   }
}

// Run all of the checks that are necessary in standalone mode (not connected to GUI)
// When we are connected to the GUI, we only run these checks when the GUI says to
void runStandardChecks()
{
   manageButtonInput(); 
   checkUSBPower();    // check whether the USB is connected to the board
   checkPowerSource(); // check whether VCC is coming from EHVCC or USB (check this after checking USB)
   checkWriteEPD();    // check whether writing to the display is enabled

   displayCurrentScreen(FALSE); // check if the display should be updated, but do not force it to update
}

// Checks all data for the given displayMode and updates the display if necessary
void displayCurrentScreen(INT8U forceUpdate)
{
   forceUpdate |= waitingToUpdate; // if waitingToUpdate is true, it is because the previous forced update was denied, so try again

   INT8U forceNewDataToGUI = (forceUpdate && usbSlaveMode && !writeEPD); // if slave to GUI, but not updating display, send data to GUI anyway
   INT8U canUpdateDisplay = checkCanUpdateDisplay(forceUpdate);
   
   if(forceNewDataToGUI || canUpdateDisplay) { // check if we can even update before looking at data
              
     INT8U currentEHData = getEHData();
     INT16S currentTemp = getTemp();
     display_app_v.currentScreenData.temperature = currentTemp;
     
     INT8U shouldUpdateDisplay = FALSE;
     
     if(displayMode == COG_DISPLAY_MODE_SUMMARY) { // summary screen

        if(forceUpdate && (lastDisplayedEHData == EHDATA_NONE)) lastDisplayedEHData = EHDATA_INIT; //if we are forcing it to rewrite the startup screen

        if(lastDisplayedEHData == EHDATA_INIT) { // if we have not already shown the startup screen
          
           display_app_v.currentScreenData.data = EHDATA_INIT;
           
           if(lastReadEHData == EHDATA_INIT) // if this is the first update since power-on, flash black and white
              display_app_v.currentScreenData.updateType = UPDATE_WITH_FLASHES; // we do this since we don't know the previous image
           else
              display_app_v.currentScreenData.updateType = UPDATE_WITH_STAGES;
           
           // update variables for detecting changes.
           lastReadEHData = currentEHData;
           lastDisplayedEHData = EHDATA_NONE; // set to EHDATA_NONE (rather than EHDATA_INIT) so we do not write this screen again

           shouldUpdateDisplay = TRUE;
        }
        else if(forceUpdate || (currentEHData != EHDATA_NONE)) { // have already shown startup screen, now show EH data on summary screen
          
           if(currentEHData == EHDATA_NONE && lastDisplayedEHData != EHDATA_NONE) //if no signals are present, but we had displayed something previously...
             currentEHData = lastDisplayedEHData; // this ensures that the screen goes back to what it was if we leave this mode and come back
           
           shouldUpdateDisplay = updateSummaryScreenData(currentEHData, forceUpdate); //this updates the buffer and also ensures that data is different
            
           display_app_v.currentScreenData.updateType = UPDATE_WITH_STAGES;
           
           // remember the EH data to send to the GUI.
           lastReadEHData = currentEHData;
        }
     }
     else if(displayMode == COG_DISPLAY_MODE_EH) { //IC status screen

        if(forceUpdate || (currentEHData != lastReadEHData) || (abs((INT32S)currentTemp - lastTempDisplayed) > 100)) { // if data has changed or temp has changed within some limit

           display_app_v.currentScreenData.data = currentEHData;
           display_app_v.currentScreenData.updateType = updateType;

           // remember the data to send to the GUI
           lastReadEHData = currentEHData;
           lastTempDisplayed = currentTemp;

           shouldUpdateDisplay = TRUE;
        }
     }
     else { // battery status screen
        
       if(forceUpdate)
       {
         shouldUpdateDisplay = TRUE;
       }
       else if(autoUpdate) { // if autoUpdate is on, check if it is time to update
         INT32U currentTime = (INT32U)getSystemTime();
         INT32U deltaTime = (currentTime - lastAutoUpdateTime) / TIMER_TICKS_PER_SEC;
         if(deltaTime >= autoUpdateDelay) {
           shouldUpdateDisplay = TRUE;
           lastAutoUpdateTime = currentTime;
         }
       }
       
       if(shouldUpdateDisplay) { 

         // update the coulomb counter data.
         updateBatStatus();
         
         display_app_v.currentScreenData.data = autoUpdate;
         display_app_v.currentScreenData.updateType = updateType;

         // remember the data to send to the GUI
         lastReadEHData = currentEHData;
         lastTempDisplayed = currentTemp;
       }
     }
      
     if(forceNewDataToGUI) // if in slave mode, the display update checks are skipped the first time, check them now
       shouldUpdateDisplay &= canUpdateDisplay; // if the data is new and we CAN update, shouldUpdateDisplay remains true
     
     // if the display should update, send data to the GUI and post the update semaphore
     if(shouldUpdateDisplay) { 
       
       sendScreenUpdate(); // update the picture of the EPD in the GUI

       // post semaphore to update EPD
       display_app_v.currentScreenData.displayMode = displayMode;
       
       updateType = UPDATE_BASIC; //set to false as default for next time
       
       postDisplayUpdate(&display_app_v.currentScreenData);
     }
     else if(forceNewDataToGUI) // if we can't update the display, we may still be able to send data back to the GUI
     {
        waitingToUpdate = FALSE;
        sendScreenUpdate(); // update the picture of the EPD in the GUI
     }
   }

   return;
}

// Task which counts button presses based on an interrupt (more energy-efficient than polling)
// This only counts the presses, the outcome of the input is handled by mainLoopTask
static void buttonInterruptTask(void* unused) {
   dn_error_t                     dnErr;
   INT8U                          osErr;
   OS_MEM*                        notifChannelMem;
   CH_DESC                        notifChannel;
   dn_gpio_ioctl_cfg_in_t         gpioInCfg;
   dn_gpio_ioctl_notif_enable_t   gpioNotifEnable;
   dn_gpio_notif_t                gpioNotif;
   INT32U                         rxLen;
   INT32U                         msgType;
   INT32U                         maxLen;

   // allocate memory for GPIO notification channel
   notifChannelMem = OSMemCreate(gpio_app_v.gpioNotifChannelBuf, 1, DN_CH_ASYNC_RXBUF_SIZE(sizeof(dn_gpio_notif_t)), &osErr);
   ASSERT(osErr==OS_ERR_NONE);

   // create channel from memory
   dnErr = dn_createAsyncChannel(notifChannelMem, &notifChannel);
   ASSERT(dnErr == DN_ERR_NONE);

   // open pin
   dnErr = dn_open(PIN_NOTIF, NULL, 0);
   ASSERT(dnErr==DN_ERR_NONE);

   // enable pull down resistor
   gpioInCfg.pullMode = DN_GPIO_PULL_DOWN;
   dnErr = dn_ioctl(PIN_NOTIF, DN_IOCTL_GPIO_CFG_INPUT, &gpioInCfg, sizeof(gpioInCfg));
   ASSERT(dnErr==DN_ERR_NONE);

   // enable GPIO notification
   gpioNotifEnable.activeLevel    = 1;
   gpioNotifEnable.fEnable        = 1;
   gpioNotifEnable.notifChannelId = notifChannel;
   dnErr = dn_ioctl(PIN_NOTIF, DN_IOCTL_GPIO_ENABLE_NOTIF, &gpioNotifEnable, sizeof(gpioNotifEnable));
   ASSERT(dnErr == DN_ERR_NONE);

   while (1) { // this is a task, it executes forever

      // wait for a GPIO notification
      dnErr = dn_readAsyncMsg(notifChannel, &gpioNotif, &rxLen, &msgType, sizeof(gpioNotif), 0);
      ASSERT(dnErr==DN_ERR_NONE);

      if(gpioNotif.level == 1) //if button down
      {
          lastBtnPressTime = (INT32U)getSystemTime(); //hold current time value
          btnPressCounter++;
          btnState = 1; //OPTIMIZE THIS BY SETTING buttonState = gpioNotif.level AND CHECKING THAT BOOLEAN INSTEAD
      }
      else //if button up
      {
          lastBtnReleaseTime = (INT32U)getSystemTime(); //hold current time value, was currentTime
          btnState = 0;

      }//end 'if button up'

      // re-arm notification on opposite level
      gpioNotifEnable.activeLevel = 0x01 - gpioNotifEnable.activeLevel;
      dnErr = dn_ioctl(PIN_NOTIF, DN_IOCTL_GPIO_ENABLE_NOTIF, &gpioNotifEnable, sizeof(gpioNotifEnable));
      ASSERT(dnErr == DN_ERR_NONE);

   }//endTaskWhileLoop
}

// MainLoopTask function which checks the button input at an interval of MAIN_LOOP_DELAY
// Because double-clicks are utilized, the outcome of the input cannot be decided immediately after a press
// Some delay after the release is necessary, so this must be checked outside of the interrupt itself
void manageButtonInput()
{
   // if the button has been pressed since last check AND the button is currently released
   if(btnPressCounter != 0 && btnState == 0) { 
                                           
      // get time difference between last button release and now
      INT32U currentTime = (INT32U)getSystemTime();
      INT32U deltaTime = currentTime - lastBtnReleaseTime;
      
      // if time to double-click has expired, there is a valid input
      if(deltaTime > BTN_CLICK_INTERVAL_TICKS) { 
      
         // set deltaTime to the amount of time the button was held down
         deltaTime = lastBtnReleaseTime - lastBtnPressTime;

         if(btnPressCounter == 1) {
            if(deltaTime < (3*TIMER_TICKS_PER_SEC)) {
               if(displayMode == COG_DISPLAY_MODE_SUMMARY || (displayMode == COG_DISPLAY_MODE_EH && USBPowerPresent)) //increment display mode
                  displayMode++;
               else
                  displayMode = COG_DISPLAY_MODE_SUMMARY;

               updateType = UPDATE_WITH_STAGES; // flash display for new screen.
               displayCurrentScreen(TRUE); // go to the next screen, force it to update since this is user input
            }
         }
         else if(btnPressCounter == 2) { // 2 clicks
            if(deltaTime < TIMER_TICKS_PER_SEC) { // 2 clicks & no hold: refresh screen; if autoRefresh is on, turn it off
               autoUpdate = FALSE;
               displayCurrentScreen(TRUE); // refresh the screen but remove autoUpdate symbol, forced it to update
            }
            else { // 2 clicks & hold: set autoUpdate interval
               if(!autoUpdate) {
                  autoUpdate = TRUE;
                  displayCurrentScreen(TRUE); // refresh the screen but include autoUpdate symbol, forced it to update
               }
               autoUpdateDelay = DIV_W_ROUND(deltaTime, TIMER_TICKS_PER_SEC); //time in seconds
               lastAutoUpdateTime = currentTime;
            }
         }
         else if(deltaTime > (3*TIMER_TICKS_PER_SEC)) { // if more than two clicks and held at least 3 seconds
            if(btnPressCounter == 3) { // 3 clicks & 3 sec hold: reset coulomb counter
               clearCCs(); // resets charge register and sets reset flags
               if(displayMode == COG_DISPLAY_MODE_BAT)
                  displayCurrentScreen(TRUE); // if the bat screen is displayed, refresh it
            }
            else if(btnPressCounter == 4) { // 4 clicks & 3 sec hold: reset device
               dnm_ucli_printf("RESET SOFTWARE\n");
               INT8U      rc;
               dnm_loc_resetCmd(&rc);
            }
            else if(btnPressCounter == 5) { // 5 clicks & 3 sec hold: toggle network join
              network_config.joinNetwork = !network_config.joinNetwork;
              setJoinStatus(network_config.joinNetwork);
              syncToConfigFile();
            }
         }
         btnPressCounter = 0;
      }
   }
}

// Checks the supply voltage using the internal supply ADC
// When using the supercap (and no battery) on DC2344A or DC2509A, the energy stored in the system is directly related to this voltage
// This is based on the relation E = (1/2)(C)(V^2) where C is assumed known and V is measured
// As a result, the supercap must be used for this function to work properly
// If a battery is used, the supercap will always be full; if USB power is used, this function just returns true
INT8U enoughEnergyToUpdateEPD()
{
   if(powerSource == USB3V3) return TRUE; // USB voltage is always good.
   if(digitalRead(PGOOD_LTC3107)) return TRUE; // LTC3107 always uses battery, assume we have a good supply

   INT16U supplyV = getSupplyV(); // Get mote's Vcc supply
   if(supplyV > MIN_SUPPLY_VOLTAGE_TO_UPDATE_EPD)
      return TRUE;

   return FALSE;
}

// Checks whether USB power is present (does not check which source is actually powering the mote)
// USB power is used to power the coulomb counters
void checkUSBPower()
{
   INT8U USBPowerWasPresent = USBPowerPresent;
   USBPowerPresent = !digitalRead(USBVCC); //active low

   if(USBPowerPresent && !USBPowerWasPresent) { //if USB is detected but it was absent before
      initCCs(); //clear the CCs in case they were half-alive and junk was stored (this also resets the overflow counter which is the important part)
      toggleCCAlertInterrupt(TRUE); //if USB is NOW present, enable the CC Alert interrupt
   }
   else if(!USBPowerPresent && USBPowerWasPresent) { //if USB is not detected but it was present before
      usbSlaveMode = FALSE; //go back to standalone mode
      toggleCCAlertInterrupt(FALSE); //if USB is NOW absent, disable the CC Alert interrupt
   }
}

// Check which power source is actually powering the mote
// USB3V3 means power is coming from the USB
// EHVCC means power is coming from DC2344A
void checkPowerSource(void)
{
   if(USBPowerPresent) {
      if(digitalRead(SRC)) { // if SRC is high, the source is USB3V3
         powerSource = USB3V3;
         return;
      }
   }
   powerSource = EHVCC; // if USB Power is not present, the source must be EHVCC
}

// Checks all IC status inputs and returns a byte with the data
// CurrentEHData byte: first nybble = PGOOD in order, 2nd nybble = EH in order (unused bit for 3106 which has no EH_ON)
// (order is LTC3106, LTC3107, LTC3330, LTC3331)
INT8U getEHData()
{
   INT8U currentEHData = 0x00;

   //PGOOD_LTC3107
   if(digitalRead(PGOOD_LTC3107)) {
      currentEHData |= EHDATA_PGOOD_LTC3107;
   }

   //BAT_OFF_LTC3107
   if(digitalRead(BAT_OFF_LTC3107)) {
      currentEHData |= EHDATA_BAT_OFF_LTC3107;
   }

   //PGOOD LTC3106
   if(digitalRead(PGOOD_LTC3106)) {
      currentEHData |= EHDATA_PGOOD_LTC3106;
   }

   //PGOOD LTC3330
   if(digitalRead(PGOOD_LTC3330)) {
      currentEHData |= EHDATA_PGOOD_LTC3330;
   }

   //EH_ON LTC3330
   if(digitalRead(EH_ON_LTC3330)) {
      currentEHData |= EHDATA_EH_ON_LTC3330;
   }

   //PGOOD LTC3331
   if(digitalRead(PGOOD_LTC3331)) {
      currentEHData |= EHDATA_PGOOD_LTC3331;
   }

   //EH_ON LTC3331
   if(digitalRead(EH_ON_LTC3331)) {
      currentEHData |= EHDATA_EH_ON_LTC3331;
   }

   return currentEHData; // returns 0 if no EH data present
}


//=============================================================================
//=========================== install a kernel header =========================
//=============================================================================

/**
A kernel header is a set of bytes prepended to the actual binary image of this
application. Thus header is needed for your application to start running.
*/

DN_CREATE_EXE_HDR(DN_VENDOR_ID_NOT_SET,
                  DN_APP_ID_NOT_SET,
                  VER_MAJOR,
                  VER_MINOR,
                  VER_PATCH,
                  VER_BUILD);

