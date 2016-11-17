/**
Copyright (c) 2016, Dust Networks.  All rights reserved.

\brief COG Display Code Module

This code paints the screens to be written to the COG. Painted screens are then written to the
display using driving functions in the cog_driver.c file.

"Painting" a screen is when screen data is used to create a bitmap buffer for the current screen.
This data is stored in a screen_data_t struct which holds the data to update any screen, regardless
of the displayMode.

In short, this file takes a screen_data_t input from dc2321a.c and uses this data to update the buffer, and
then outputs the buffer to cog_driver.c to be written to the display.

*/

#include <stdlib.h>
#include <string.h>
#include "app_task_cfg.h"
#include "dnm_ucli.h"
#include "dn_common.h"

#include "dc2321a.h"
#include "devices.h"
#include "gui_comm.h"
#include "cog_display.h"
#include "cog_driver.h"
#include "coulomb_counter.h"

//=========================== defines =========================================
#define COG_DISPLAY_MAX    999     // the largest number displayed on the EPD.
#define COG_DISPLAY_MIN    0       // the smallest number displayed on the EPD.  Note that background color is used to indicate negative numbers.
#define COG_DISPLAY_RES    100     // the resolution in 1/COG_DISPLAY_RES of which numbers are displayed on the EPD.
#define COG_DISPLAY_C      9995    // the number of uC below which we display in mC instead of C.
#define COG_DISPLAY_A      9995    // the number of uA below which we display in uA instead of mA.

typedef enum {SELECTED_IC_LTC3106, SELECTED_IC_LTC3107, SELECTED_IC_LTC3330, SELECTED_IC_LTC3331, NUM_SELECTED_ICS} selected_ic_t;

//=========================== prototypes ======================================
void displayScreen(screen_data_t* screenData);
void paintScreen(COG_DRIVER_IMAGE* screen, screen_data_t* screenData);
void paintSummaryScreen(COG_DRIVER_IMAGE* screen, INT8U maskedEHStatus);
void paintEHStatusScreen(COG_DRIVER_IMAGE* screen, INT16S temperature, INT8U ehStatusBits);
void paintBatStatusScreen(COG_DRIVER_IMAGE* screen, screen_data_t* screenData);

void paintImage(COG_DRIVER_IMAGE data, const INT8U *image, INT8U x, INT8U y, INT8U width, INT8U height);
void paintNumber(COG_DRIVER_IMAGE screen, INT8U num, INT8U x, INT8U y);
void paintTemperature(COG_DRIVER_IMAGE data, INT16S temperature, BOOLEAN degF);
void invertPixels(COG_DRIVER_IMAGE data, INT8U x, INT8U y, INT8U widthInBytes, INT8U height); 

void paintEHData(COG_DRIVER_IMAGE screen, INT8U ehdata); 
void paintAutoUpdateSymbol(COG_DRIVER_IMAGE, INT8U autoUpdateOn); 

//=========================== variables =======================================
display_app_vars_t display_app_v; //holds task/stack data as well as screen queue data

// Data to tell how to update the display
INT8U displayMode = COG_DISPLAY_MODE_SUMMARY; //Determines which screen should be shown (3 screens: summary, IC status, and battery status)
INT8U updateType = TRUE; //Flag to tell that the display needs to be "flashed" (updated with solid black/white a few times)
INT8S selectedIC = -1; //Represents which IC has been selected as the power source for the summary screen

// Data to tell if the display can be updated
INT8U waitingToUpdate = TRUE; //Is the display waiting to update?
INT8U displayDataLocked = FALSE; //Used to ensure that we do not update the simple display's buffer before a screen write occurs (instead, no data is read until it is finished)
INT8U writeEPD = TRUE; //Holds the position of SW1 on DC2321A so we know if we can update the display

// The last values displayed on the EPD
INT8U lastReadEHData = EHDATA_INIT; //EHData byte can never = EHDATA_INIT after updating, therefore it will always update the screen upon powerup
INT8U lastDisplayedEHData = EHDATA_INIT; //contains only the bits of the data that was last written to the screen
INT16S lastTempDisplayed = 0; //Holds last temperature displayed

//=========================== tasks ===========================================
// Task to update the display
// This is done in its own OS task so we can still run other tasks while updating
void displayUpdateTask(void* unused)
{
   INT8U      osErr;
   INT8U      write_epd_pin_state = 1;

   // Init hardware here instead of in cogDisplayInit2() as mainLoop may pend before calling cogDisplayInit2() allowing this task to run.
   EPD_display_hardware_init();

   // Set the data that will initially be considered 'previous data'
   display_app_v.previousScreenData.displayMode = COG_DISPLAY_MODE_SUMMARY; //initialize the startup screen
   display_app_v.previousScreenData.data = EHDATA_INIT;
   
   while(1) // this is a task, it executes forever
   {
      // wait for the semaphore to be posted
      screen_data_t* newScreenDataPtr = (screen_data_t*)OSQPend(
         display_app_v.displayUpdateQ,      // pevent
         0,                                 // timeout
         &osErr                             // perr
      );
      ASSERT (osErr == OS_ERR_NONE);

      waitingToUpdate = FALSE;
      
      displayDataLocked = TRUE; //ensure nothing tries to change data while we are displaying
      displayScreen(newScreenDataPtr);
      displayDataLocked = FALSE; //unlock the buffer for future updates
   }
}

//=========================== functions ==========================================

// Init things that must be done before the OS is started.
void cogDisplayInit()
{
   INT8U osErr;

   // display update semaphore
   display_app_v.displayUpdateQ = OSQCreate(&display_app_v.screenQ, SCREEN_Q_LENGTH);
   ASSERT (display_app_v.displayUpdateQ != NULL);

   osErr  = OSTaskCreateExt(
      displayUpdateTask,
      (void *)0,
      (OS_STK*)(&display_app_v.displayUpdateTaskStack[TASK_APP_DISPLAY_UPDATE_STK_SIZE-1]),
      TASK_APP_DISPLAY_UPDATE_PRIORITY,
      TASK_APP_DISPLAY_UPDATE_PRIORITY,
      (OS_STK*)display_app_v.displayUpdateTaskStack,
      TASK_APP_DISPLAY_UPDATE_STK_SIZE,
      (void *)0,
      OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR
   );
   ASSERT(osErr == OS_ERR_NONE);
   OSTaskNameSet(TASK_APP_DISPLAY_UPDATE_PRIORITY, (INT8U*)TASK_APP_DISPLAY_UPDATE_NAME, &osErr);
   ASSERT(osErr == OS_ERR_NONE);
}

// Init things that must be done after the OS is started.
void cogDisplayInit2()
{
   // Initialize current measurements so that the first one displayed on the EPD is good.
   updateAllCCData();
   for(int CC = 0; CC < NUMCC; CC++) {
      display_app_v.currentScreenData.ccData[CC][CUR] = getCCData(CC, CUR);
   }
}

// Add a new screen_data_t to the queue to be painted when the display is ready
void postDisplayUpdate(screen_data_t* screen_data)
{ 
   // If we are not at capacity, point to the next empty spot in Queue.
   OS_Q* q = display_app_v.displayUpdateQ->OSEventPtr;
   if(q->OSQEntries < SCREEN_Q_LENGTH - 1) {
      display_app_v.screenDataIndex = (display_app_v.screenDataIndex + 1) % SCREEN_Q_LENGTH;
   }
   else {
      dnm_ucli_printf("Queue Overflow!\r\n"); 
   }
     
   display_app_v.screenData[display_app_v.screenDataIndex] = *screen_data; 

   // Only post to Queue if it's not full already.
   if(q->OSQEntries < SCREEN_Q_LENGTH - 1) {
      INT8U osErr = OSQPost(display_app_v.displayUpdateQ, (void*)&display_app_v.screenData[display_app_v.screenDataIndex]);
      ASSERT(osErr == OS_ERR_NONE);
   }
   
   return;
}

// Displays a screen given a screen_data_t struct
// Paints the screen and then displays it
void displayScreen(screen_data_t* screenData)
{
   COG_DRIVER_IMAGE screen; //the array that will be filled with the screen data
   
   // write the screen to the EPD
   // we do this in a custom manner so that we can allocate only one screen buffer at a time
   // this saves RAM since we do not need to store the previous screen and the current screen simultaneously
   // if this RAM does not need to be preserved, it is faster to store both buffers and use EPD_display_from_pointer()
   
   //paint initial screens before powering on the display to save energy
   if(screenData->updateType == UPDATE_WITH_STAGES)
      paintScreen(&screen, &display_app_v.previousScreenData);
   else
      paintScreen(&screen, screenData);
   
   EPD_power_on();
   EPD_initialize_driver();
   if(screenData->updateType == UPDATE_WITH_STAGES) { //if the screen should get a full update in stages rather than just one write
      stage_handle_array(&screen, Stage1);
      COG_Flash(TRUE); // Use quicker COG_Flash() function twice to replace this longer call -> stage_handle_array(previous_image_ptr, Stage2);
      COG_Flash(TRUE);
      paintScreen(&screen, screenData); //must paint the second screen during the update since we only use one buffer (to save RAM)
      stage_handle_array(&screen, Stage3);
   }
   else if(screenData->updateType == UPDATE_WITH_FLASHES) {
      const INT8U numFlashes = 3;
      for(INT8U i = 0; i < numFlashes; i++) {
         COG_Flash(TRUE);
         OSTimeDly(100);
         COG_Flash(FALSE);
         OSTimeDly(100);
      }
   }
   stage_handle_array(&screen, Stage4);
   EPD_power_off();
   
   display_app_v.previousScreenData = *screenData;
}

// Switch which calls the correct function to paint the screen buffer based on displayMode
void paintScreen(COG_DRIVER_IMAGE* screen, screen_data_t* screenData)
{
   //paint the screen based on the screenData displayMode
   switch(screenData->displayMode) {
   case COG_DISPLAY_MODE_SUMMARY:
       paintSummaryScreen(screen, screenData->data);
       break;
   case COG_DISPLAY_MODE_EH:
       paintEHStatusScreen(screen, screenData->temperature, screenData->data);
       break;
   case COG_DISPLAY_MODE_BAT:
       paintBatStatusScreen(screen, screenData);
       break;
   }
}

// Paints the summary screen's buffer
void paintSummaryScreen(COG_DRIVER_IMAGE* screen, INT8U maskedEHStatus)
{
   if(maskedEHStatus == EHDATA_INIT)
   {
      memcpy(screen, startupScreen, sizeof(COG_DRIVER_IMAGE)); // Copy starting screen to temporary memory
      return; //for the startup screen, this is all we need to do, so end here
   }
  
   const INT8U nameX = COG_NUM_DOTS - COG_IMAGE_LTC_WIDTH - 21;
   const INT8U nameY = (INT8U)(COG_NUM_LINES - COG_IMAGE_LTC_HEIGHT - 44);
   const INT8U nameW = COG_IMAGE_LTC_WIDTH;
   const INT8U nameH = COG_IMAGE_LTC_HEIGHT;

   const INT8U sourceX = COG_NUM_DOTS - COG_IMAGE_PS_WIDTH - 47;
   const INT8U sourceY = (INT8U)(COG_NUM_LINES - COG_IMAGE_PS_HEIGHT - 32);
   const INT8U sourceW = COG_IMAGE_PS_WIDTH;
   const INT8U sourceH = COG_IMAGE_PS_HEIGHT;

   const INT8U batTypeX = COG_NUM_DOTS - COG_IMAGE_BAT_WIDTH - 100;
   const INT8U batTypeY = (INT8U)(COG_NUM_LINES - COG_IMAGE_BAT_HEIGHT - 52);
   const INT8U batTypeW = COG_IMAGE_BAT_WIDTH;
   const INT8U batTypeH = COG_IMAGE_BAT_HEIGHT;
   
   // Update EH Data sources on summary screen
   memcpy(screen, summaryScreen, sizeof(COG_DRIVER_IMAGE)); // Copy starting screen to temporary memory.
   
   if(maskedEHStatus & EHDATA_PGOOD_LTC3106) { //LTC3106
      paintImage(screen, LTC3106Img, nameX, nameY, nameW, nameH); //"LTC3106"
      paintImage(screen, batOrSolarImg, sourceX, sourceY, sourceW, sourceH); //BATTERY
   }
   else if(maskedEHStatus & EHDATA_PGOOD_LTC3107) { //LTC3107
      paintImage(screen, LTC3107Img, nameX, nameY, nameW, nameH); //"LTC3107"
      if(maskedEHStatus & EHDATA_BAT_OFF_LTC3107) {
         paintImage(screen, thermalImg, sourceX, sourceY, sourceW, sourceH); //FIRE
      }
      else {
         paintImage(screen, batImg, sourceX, sourceY, sourceW, sourceH); //BATTERY (PRI)
         paintImage(screen, priImg, batTypeX, batTypeY, batTypeW, batTypeH);
      }
   }
   else if(maskedEHStatus & EHDATA_PGOOD_LTC3330) { //LTC3330
      paintImage(screen, LTC3330Img, nameX, nameY, nameW, nameH); //"LTC3330"
      if(maskedEHStatus & EHDATA_EH_ON_LTC3330) {
         paintImage(screen, piezoOrSolarImg, sourceX, sourceY, sourceW, sourceH); //PIEZO/SOLAR
      }
      else {
         paintImage(screen, batImg, sourceX, sourceY, sourceW, sourceH); //BATTERY (PRI)
         paintImage(screen, priImg, batTypeX, batTypeY, batTypeW, batTypeH);
      }
   }
   else if(maskedEHStatus & EHDATA_PGOOD_LTC3331) { //LTC3331
      paintImage(screen, LTC3331Img, nameX, nameY, nameW, nameH);  //"LTC3331"
      if(maskedEHStatus & EHDATA_EH_ON_LTC3331) {
         paintImage(screen, piezoOrSolarImg, sourceX, sourceY, sourceW, sourceH); //PIEZO/SOLAR
      }
      else {
         paintImage(screen, batImg, sourceX, sourceY, sourceW, sourceH); //BATTERY (SEC)
         paintImage(screen, secImg, batTypeX, batTypeY, batTypeW, batTypeH);
      }
   }
}

// Paints the EH status screen's buffer
void paintEHStatusScreen(COG_DRIVER_IMAGE* screen, INT16S temperature, INT8U ehStatusBits)
{
   memcpy(screen, icStatusScreen, sizeof(COG_DRIVER_IMAGE));
   paintTemperature(screen, temperature, FALSE);
   paintEHData(screen, ehStatusBits);
}

// Paints the battery status screen's buffer
void paintBatStatusScreen(COG_DRIVER_IMAGE* screen, screen_data_t* screenData)
{
   memcpy(screen, batStatusScreen, sizeof(COG_DRIVER_IMAGE));
   paintTemperature(screen, screenData->temperature, TRUE);
   paintAutoUpdateSymbol(screen, screenData->data);
   
   INT32S ccData;
   const COG_IMAGE_UNITS* unitPtr;
   const INT8U xCoordinate[NUMCC] =   {COG_NUM_DOTS - COG_IMAGE_NUM_WIDTH - 44,
                                       COG_NUM_DOTS - COG_IMAGE_NUM_WIDTH - 87};
   const INT8U yCoordinate[NUMDATA] = {COG_NUM_LINES - COG_IMAGE_NUM_HEIGHT - 82,
                                       COG_NUM_LINES - COG_IMAGE_NUM_HEIGHT - 50,
                                       COG_NUM_LINES - COG_IMAGE_NUM_HEIGHT - 66};

   // Update EPD for all of the data from the LTC2942s
   for(int CC = 0; CC < NUMCC; CC++) {
      for(int data = 0; data < NUMDATA; data++) {
      
        ccData = abs(screenData->ccData[CC][data]);

         if(data == CHG) { // CHARGE
            // display correct units and scale data appropriately.
            if(ccData < COG_DISPLAY_C) {
               unitPtr = &milliCoulombs;
               ccData = DIV_W_ROUND(ccData, UC_PER_MC/COG_DISPLAY_RES);
            }
            else {
               unitPtr = &coulombs;
               ccData = DIV_W_ROUND(ccData, UC_PER_C/COG_DISPLAY_RES);
            }
            paintImage(screen, unitPtr, xCoordinate[CC] - 32, yCoordinate[data], COG_IMAGE_UNITS_WIDTH, COG_IMAGE_UNITS_HEIGHT);
         }
         else if(data == VLT) { // VOLTAGE
            // voltage data is always displayed in 1/COG_DISPLAY_V of a Volt.
            ccData = DIV_W_ROUND(ccData, UV_PER_V/COG_DISPLAY_RES);
         }
         else { // CURRENT
            // display correct units and scale data appropriately.
            if(ccData < COG_DISPLAY_A) {
               unitPtr = &microAmps;
               ccData = DIV_W_ROUND(ccData, NA_PER_UA/COG_DISPLAY_RES);
            }
            else {
               unitPtr = &milliAmps;
               ccData = DIV_W_ROUND(ccData, NA_PER_MA/COG_DISPLAY_RES);
            }
            paintImage(screen, unitPtr, xCoordinate[CC] - 32, yCoordinate[data], COG_IMAGE_UNITS_WIDTH, COG_IMAGE_UNITS_HEIGHT);
         }

         // display graphics for each digit, with background color indicating negative/positive.
         ccData = max(ccData, COG_DISPLAY_MIN); // Bounds check the number
         ccData = min(ccData, COG_DISPLAY_MAX);
         const INT8U xShift[] = {21, 12, 0};
         for(int digit = 0; digit < LENGTH(xShift); digit++) {
            paintNumber(screen, ccData%10, xCoordinate[CC] - xShift[digit], yCoordinate[data]);
            ccData /= 10;
         }
      } // end data for
   } // end CC for
  
   // if positive CHG/CUR, invert the background (SEC CC only). This means current is going INTO the battery.
   // this needs to be checked after all data is in so we can compare both charge and current 
   const INT8U cellWidth = 3*COG_IMAGE_NUM_WIDTH + COG_IMAGE_UNITS_WIDTH + 8; // width in dots = 3*number width, width of units, width of decmial point + 1 dot right and left
   const INT8U cellHeight[] = // height in dots, sum of number height, 1 dot above and below number, and 1 dot for line 
    {0, COG_IMAGE_NUM_HEIGHT + 2, COG_IMAGE_NUM_HEIGHT + 2, 2*(COG_IMAGE_NUM_HEIGHT + 2) + 1};
   INT8U cellIndex = (screenData->ccData[SEC][CHG] < 0 ? 1 << 0 : 0) | (screenData->ccData[SEC][CUR] < 0 ? 1 << 1 : 0);

    // top of flip is top of CUR row, height of flip is two rows plus 1 for the table line
   if(cellHeight[cellIndex]) invertPixels(screen, xCoordinate[SEC] - 33, yCoordinate[cellIndex & 1 ? CHG : CUR]-1, cellWidth, cellHeight[cellIndex]);

   return;
}

// Checks whether we can update the display based on various conditions
// Updates can be 'triggered' naturally due to changes in data, or they can be 'forced' by the user
// If they are 'triggered' but we cannot update due to power limitations, cancel the request
// If they are 'forced' but we cannot update right now, wait until power is good
INT8U checkCanUpdateDisplay(INT8U updateIsForced) 
{
  INT8U canUpdate = TRUE; // set to 'true' initially; set to false if it does not pass all tests
  
  if(!digitalRead(WRITE_EPD)) { // if the EPD is turned off by the SW1
    canUpdate = FALSE; // do not set waitingToUpdate because when WRITE_EPD goes high again we will write automatically
  }
  else if(displayDataLocked) { // display data is locked during a write so that we do not check new data mid-write
    canUpdate = FALSE;
    
    if(updateIsForced) // if a forced update is disabled due to locked data, wait until we can write
      waitingToUpdate = TRUE;
  }
  else if(!enoughEnergyToUpdateEPD(displayMode != COG_DISPLAY_MODE_BAT)) { //if the supply is not sufficient
    canUpdate = FALSE;
    
    if(updateIsForced) // if a forced update is disabled due to low supply, wait until we can write
      waitingToUpdate = TRUE;
  }

  return canUpdate;
}

// Send the new display data over USB so that it can be displayed by the GUI
void sendScreenUpdate()
{
   if(displayMode == COG_DISPLAY_MODE_SUMMARY) {
      gui_comm_id_summary_screen_t data_struct;
      data_struct.currentEHData = lastReadEHData;
      guiCommSendData(GUI_COMM_ID_SIMPLE_SCREEN, &data_struct);
   }
   else if(displayMode == COG_DISPLAY_MODE_EH) {
      gui_comm_id_status_screen_t data_struct;
      data_struct.currentEHData = lastReadEHData;
      data_struct.currentTemperature = lastTempDisplayed;
      guiCommSendData(GUI_COMM_ID_STATUS_SCREEN, &data_struct);
   }
   else if(displayMode == COG_DISPLAY_MODE_BAT) {
      gui_comm_id_bat_screen_t data_struct;
      data_struct.currentTemperature = lastTempDisplayed;
      for(int CC = 0; CC < NUMCC; CC++) {
         data_struct.lastChargesDisplayed[CC] = display_app_v.currentScreenData.ccData[CC][CHG];
         data_struct.lastVoltagesDisplayed[CC] = display_app_v.currentScreenData.ccData[CC][VLT];
         data_struct.lastCurrentsDisplayed[CC] = display_app_v.currentScreenData.ccData[CC][CUR];
      }
      guiCommSendData(GUI_COMM_ID_BAT_SCREEN, &data_struct);
   }
}

// Check the position of the EPD power switch to determine if we can update
// If EPD power switch was OFF but is now ON, refresh screen
void checkWriteEPD()
{
   INT8U writeepd_last = writeEPD;
   writeEPD = digitalRead(WRITE_EPD);  // GPIO high means write
   if(!writeepd_last && writeEPD) {    // if we weren't writing before, do a write now so we know the state
         updateType = UPDATE_WITH_STAGES; // flash display as this is first write to screen.
         waitingToUpdate = FALSE;
         displayCurrentScreen(TRUE);      // overriding checks because you don't know what the last display was.
   }
}

// Paint an image into the selected screen's buffer at the selected position
void paintImage(COG_DRIVER_IMAGE screen, const INT8U *image, INT8U x, INT8U y, INT8U width, INT8U height)
{ // "x" is the horizontal position top left pixel of the image as it will be displayed on the screen
  // "y" is the vertical position top left pixel of the image as it will be displayed on the screen
  // "width" is the width of the NEW image in pixels
  // "height is the height of the NEW image in pixels
   INT8U temp_width, temp_x;
   INT8U temp_height, temp_y;
   INT8U offset = x % BITS_PER_BYTE;
   const INT8U* image_ptr = image;

   temp_height = height;
   temp_y = y;
   while(temp_height != 0) {
      temp_width = width;
      temp_x = x;
      while(temp_width != 0) {
         // first figure out which bits to AND out of the screen
         INT8U num_bits = min(temp_width, BITS_PER_BYTE - temp_x % BITS_PER_BYTE);
         INT8U num_shift = (temp_width >= BITS_PER_BYTE ? 0 : BITS_PER_BYTE - num_bits);
         INT8U mask = MASK(num_bits, num_shift);
         screen[temp_y][temp_x/BITS_PER_BYTE] &= ~mask;

         // next build the img bits to OR into the screen (note - this assumes unused bits at end of img bytes are 0!)
         INT8U img_bits = *image_ptr;
         if((temp_x - x) % BITS_PER_BYTE == 0) { // All data is in this byte, whether it's the first bits of row or a complete byte
            img_bits >>= offset;
         }
         else { // some data is in this byte, whether it's the last bits of row or data that spans two byes
            img_bits <<= (BITS_PER_BYTE - offset);
            if(temp_width > offset) {
               image_ptr++;                        // Done with bits from previous byte, point to next
               img_bits += *image_ptr >> offset;
            }
         }

         // OR img bits into the screen        
         screen[temp_y][temp_x/BITS_PER_BYTE] |= img_bits;
         
         // loop overhead
         temp_width -= num_bits;
         temp_x += num_bits;
         
         // we're done with this byte if we're screen and img bytes are aligned, or if this was the last fraction of byte in row
         if((offset == 0) || (temp_width == 0)) {
            image_ptr++;
         }
      }
 
      // loop overhead
      temp_height--;
      temp_y++;
   }
}

// Paint a number (0-9) into the desired coordinates in the image buffer
void paintNumber(COG_DRIVER_IMAGE screen, INT8U num, INT8U x, INT8U y)
{  // "data" contains the current state of the image buffer
   // "num" is an integer used to define which number image will be brought into the buffer
   // "x" is the horizontal position top left pixel of the image as it will be displayed on the screen
   // "y" is the vertical position top left pixel of the image as it will be displayed on the screen
   const COG_IMAGE_NUM *const imglist[] = {&num0, &num1, &num2, &num3, &num4, &num5, &num6, &num7, &num8, &num9};
   paintImage(screen, imglist[num], x, y, COG_IMAGE_NUM_WIDTH, COG_IMAGE_NUM_HEIGHT);
   return;
}

// Paint the temperature in the image buffer
// Assumes temperature is < 100 degrees F/C (because it uses 3 digits, XX.X format)
void paintTemperature(COG_DRIVER_IMAGE screen, INT16S temperature, BOOLEAN degF)
{ 
   // convert temperature in hundreths of degC to tenths of degC or deg F, depending upon which screen we're on
   if(degF) {
      temperature = ((INT32S) temperature * 90 + (temperature >= 0 ? 250 : -250)) / 500 + 320;
   }
   else {
      temperature = (temperature + 5) / 10;
   }

   INT8U neg = temperature < 0;
   temperature = abs(temperature);

   // bound temperature between max and min that can be stored on screen
   if(temperature > COG_DISPLAY_MAX)
      temperature = COG_DISPLAY_MAX;
   else if(temperature < COG_DISPLAY_MIN)
      temperature = COG_DISPLAY_MIN;

   // display the temperature
   const INT8U x_pos[] = {COG_NUM_DOTS - COG_IMAGE_NUM_WIDTH - 22,
                          COG_NUM_DOTS - COG_IMAGE_NUM_WIDTH - 10,
                          COG_NUM_DOTS - COG_IMAGE_NUM_WIDTH - 1};
   const INT8U y_pos = COG_NUM_LINES - COG_IMAGE_NUM_HEIGHT - 34;
   for(INT8U x_idx = 0; x_idx < LENGTH(x_pos); x_idx++) {
      paintNumber(screen, temperature%10, x_pos[x_idx], y_pos);
      temperature /= 10;
   }

   const INT8U unitsWidth = 11; // degrees C or F width
   const INT8U cellWidth = 3*COG_IMAGE_NUM_WIDTH + unitsWidth + 7; // width in dots = 3*number width, widht of units, width of decmial point + 1 dot right and left, + 1 dot past unit
   const INT8U cellHeight = COG_IMAGE_NUM_HEIGHT + 2; // height in dots = , number height + 1 dot above and below
   if(neg) invertPixels(screen, x_pos[0] - unitsWidth - 1, y_pos - 1, cellWidth, cellHeight);

   return;
}

// Checks all IC status signals (PGOOD and EH_ON) and selects an IC based on the newest signals
// Updates summary display buffer
// Returns true if it has updated the display data, otherwise returns false
INT8U updateSummaryScreenData(INT8U ehdata, INT8U forceUpdate) 
{
   // determine the selectedIC
   INT8U currentEHDataPGOOD = ehdata & EHDATA_PGOOD;
   if(currentEHDataPGOOD != 0) { // if any PGOOD was detected
      if(selectedIC != -1) { // if an IC was previously selected, pick newest
         if(currentEHDataPGOOD  != (EHDATA_PGOOD_LTC3331 << (NUM_SELECTED_ICS - 1 - selectedIC))) { // if different/multiple PGOODs are present
            INT8U lastReadEHDataPGOOD = lastReadEHData & EHDATA_PGOOD;

            if(currentEHDataPGOOD  != lastReadEHDataPGOOD) { // if something is new
               INT8U newestEHDataPGOOD = currentEHDataPGOOD  & ~(EHDATA_PGOOD_LTC3331 << (NUM_SELECTED_ICS - 1 - selectedIC)); // subtract out the bit of the currently selected IC

               if((newestEHDataPGOOD & ~lastReadEHDataPGOOD) != 0) { // make sure we don't erase the only remaining signal bits but...
                  newestEHDataPGOOD &= ~lastReadEHDataPGOOD;  // check for something that wasn't high last time
               }

               if(newestEHDataPGOOD & EHDATA_PGOOD_LTC3106) selectedIC = SELECTED_IC_LTC3106;
               else if(newestEHDataPGOOD & EHDATA_PGOOD_LTC3107) selectedIC = SELECTED_IC_LTC3107;
               else if(newestEHDataPGOOD & EHDATA_PGOOD_LTC3330) selectedIC = SELECTED_IC_LTC3330;
               else if(newestEHDataPGOOD & EHDATA_PGOOD_LTC3331) selectedIC = SELECTED_IC_LTC3331;
            }
            else { // nothing seems new, but lets verify old signal
               if((currentEHDataPGOOD & lastDisplayedEHData == EHDATA_NONE) || !(currentEHDataPGOOD  & (0x10 << (3 - selectedIC)))) { // if the old signal has gone low, pick a new one
                 if(ehdata & EHDATA_PGOOD_LTC3106) selectedIC = SELECTED_IC_LTC3106;
                 else if(ehdata & EHDATA_PGOOD_LTC3107) selectedIC = SELECTED_IC_LTC3107;
                 else if(ehdata & EHDATA_PGOOD_LTC3330) selectedIC = SELECTED_IC_LTC3330;
                 else if(ehdata & EHDATA_PGOOD_LTC3331) selectedIC = SELECTED_IC_LTC3331;
               }
            }
         }// else, keep the old selectedIC value
      }
      else { // else, this is the first IC selected, pick any
         if(ehdata & EHDATA_PGOOD_LTC3106) selectedIC = SELECTED_IC_LTC3106;
         else if(ehdata & EHDATA_PGOOD_LTC3107) selectedIC = SELECTED_IC_LTC3107;
         else if(ehdata & EHDATA_PGOOD_LTC3330) selectedIC = SELECTED_IC_LTC3330;
         else if(ehdata & EHDATA_PGOOD_LTC3331) selectedIC = SELECTED_IC_LTC3331;
      }

      INT8U currentWrittenEHDataMask = ((EHDATA_PGOOD_LTC3331|EHDATA_EH_ON_LTC3331) << (NUM_SELECTED_ICS - 1 - selectedIC)); // masks out bits of unselected ICs
      INT8U currentEHDataToWrite = ehdata & currentWrittenEHDataMask;
      
      // update the display with the currently selected IC
      if((currentEHDataToWrite != lastDisplayedEHData) || forceUpdate) { // if a PGOOD is present and anything has changed since last write
         display_app_v.currentScreenData.data = currentEHDataToWrite;
         lastDisplayedEHData = currentEHDataToWrite;
         return 1;
      }
   }

   return 0;
}

// Paints EH data to the screen buffer
void paintEHData(COG_DRIVER_IMAGE screen, INT8U ehdata)
{
   const struct {INT8U mask;INT8U x;INT8U y;} updateEHDataTable[] = {
      {EHDATA_PGOOD_LTC3107,     COG_NUM_DOTS - COG_IMAGE_YESNO_WIDTH - 53, (INT8U)(COG_NUM_LINES - COG_IMAGE_YESNO_HEIGHT - 63)},
      {EHDATA_BAT_OFF_LTC3107,   COG_NUM_DOTS - COG_IMAGE_YESNO_WIDTH - 95, (INT8U)(COG_NUM_LINES - COG_IMAGE_YESNO_HEIGHT - 63)},
      {EHDATA_PGOOD_LTC3106,     COG_NUM_DOTS - COG_IMAGE_YESNO_WIDTH - 53, (INT8U)(COG_NUM_LINES - COG_IMAGE_YESNO_HEIGHT - 51)},
      {EHDATA_PGOOD_LTC3330,     COG_NUM_DOTS - COG_IMAGE_YESNO_WIDTH - 53, (INT8U)(COG_NUM_LINES - COG_IMAGE_YESNO_HEIGHT - 75)},
      {EHDATA_EH_ON_LTC3330,     COG_NUM_DOTS - COG_IMAGE_YESNO_WIDTH - 95, (INT8U)(COG_NUM_LINES - COG_IMAGE_YESNO_HEIGHT - 75)},
      {EHDATA_PGOOD_LTC3331,     COG_NUM_DOTS - COG_IMAGE_YESNO_WIDTH - 53, (INT8U)(COG_NUM_LINES - COG_IMAGE_YESNO_HEIGHT - 87)},
      {EHDATA_EH_ON_LTC3331,     COG_NUM_DOTS - COG_IMAGE_YESNO_WIDTH - 95, (INT8U)(COG_NUM_LINES - COG_IMAGE_YESNO_HEIGHT - 87)}};

   for(INT8U eh = 0; eh < LENGTH(updateEHDataTable); eh++) {
      paintImage(screen, ehdata & updateEHDataTable[eh].mask ? &yesImg : &noImg,
                        updateEHDataTable[eh].x, updateEHDataTable[eh].y, COG_IMAGE_YESNO_WIDTH, COG_IMAGE_YESNO_HEIGHT);
   }
}

// Updates CC data for the battery status screen
void updateBatStatus()
{
   INT32S ccData;

   // update all data from the LTC2942s
   updateAllCCData();

   // update EPD for all of the data from the LTC2942s
   for(int CC = 0; CC < NUMCC; CC++) {
      for(int data = 0; data < NUMDATA; data++) {
         // get one piece of LTC2942 data to display on the EPD, scale it appopriately, display the correct units, and convert to graphics
         ccData = getCCData(CC, data);
         display_app_v.currentScreenData.ccData[CC][data] = ccData; //store the data in the current screen data
      }
   }
   return;
}

// Paints or removes autoUpdate symbol from display
void paintAutoUpdateSymbol(COG_DRIVER_IMAGE screen, INT8U autoUpdateOn)
{
   paintImage(screen, autoUpdateOn ? &auImg : &auImgOff,
                    COG_NUM_DOTS - COG_IMAGE_AU_WIDTH - 2,
                    COG_NUM_LINES - COG_IMAGE_AU_HEIGHT - 2,
                    COG_IMAGE_AU_WIDTH, COG_IMAGE_AU_HEIGHT);
}

// Invert the colors of an image within a given box of dimensions
// Width and height are in units of pixels
void invertPixels(COG_DRIVER_IMAGE screen, INT8U x, INT8U y, INT8U width, INT8U height)
{
   INT8U temp_width, temp_x;
   INT8U temp_height, temp_y;
   INT8U offset = x % BITS_PER_BYTE;

   temp_height = height;
   temp_y = y;
   while(temp_height != 0) {
      temp_width = width;
      temp_x = x;
      while(temp_width != 0) {
         // first figure out which bits to AND out of the screen
         INT8U num_bits = min(temp_width, BITS_PER_BYTE - temp_x % BITS_PER_BYTE);
         INT8U num_shift = (temp_width >= BITS_PER_BYTE ? 0 : BITS_PER_BYTE - num_bits);
         INT8U mask = MASK(num_bits, num_shift);
         
         // get the bits which need their polarity changed
         INT8U flip_bits = ~screen[temp_y][temp_x/BITS_PER_BYTE] & mask;

         // clear the bits in screen which need their polarity changed
         screen[temp_y][temp_x/BITS_PER_BYTE] &= ~mask;

         // OR changed bits into the screen     
         screen[temp_y][temp_x/BITS_PER_BYTE] |= flip_bits;
         
         // loop overhead
         temp_width -= num_bits;
         temp_x += num_bits;        
      }
 
      // loop overhead
      temp_height--;
      temp_y++;
   }
}