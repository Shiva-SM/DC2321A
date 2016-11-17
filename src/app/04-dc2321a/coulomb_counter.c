/*
Copyright (c) 2013, Dust Networks.  All rights reserved.

"Coulomb Counter" is often abbreviated to CC.

This file manages two LTC2942 coulomb counters on a single i2c and interrupt input
to the Dust processor, managed with the LTC4305 i2c bus multiplexer.

All datasheet references in this file refer to Linear Technology documents 2942fa.pdf
and LTC4305.pdf.

The coulomb counters are powered by the USB.
So, USB power (or at least 5V on the 5VUSB turret) must be present before CC communication is attempted.
Also, a coulomb counter is only powered if a power source is connected to its battery input.

*/

#include <string.h>
#include <stdio.h>
#include "dn_common.h"
#include "dn_gpio.h"
#include "dnm_ucli.h"

#include "dn_system.h"
#include "dn_i2c.h"
#include "app_task_cfg.h"

#include "dc2321a.h"
#include "devices.h"
#include "coulomb_counter.h"

//=========================== defines =========================================
#define LTC4305_ADDRESS       0x44 //x1000100
#define LTC2942_ADDRESS       0x64 //x1100100
#define LTC2942_AR_ADDRESS    0x0C //x0001100

#define I2C_TIMEOUT           0x0A // 10 ms

// These are the values from the LTC2942 datasheet.
#define LTC2942_REGA  ((INT8U)0x00) // Status Register
#define LTC2942_REGB  ((INT8U)0x01) // Control Register
#define LTC2942_REGCD ((INT8U)0x02) // Accumulated Charge Registers
#define LTC2942_REGEF ((INT8U)0x04) // Charge Threshold High Registers
#define LTC2942_REGGH ((INT8U)0x06) // Charge Threshold Low Registers
#define LTC2942_REGIJ ((INT8U)0x08) // Voltage Registers
#define LTC2942_REGK  ((INT8U)0x0A) // Voltage Threshold High Register
#define LTC2942_REGL  ((INT8U)0x0B) // Voltage Threshold Low Register
#define LTC2942_REGMN ((INT8U)0x0C) // Temperature Registers
#define LTC2942_REGOP ((INT8U)0x0E) // Temperature Threshold Registers

#define LTC2942_REGB_CHARGE_ALERT_LOW MASK(1, 2)
#define LTC2942_REGB_CHARGE_ALERT_HIGH MASK(1, 3)

#define LTC2942_CHG_M         (1 << LTC2942_PREBITS)                    // the value of M as per datasheet Table 3.
#define LTC2942_CHG_QLSB      (306*50/LTC2942_RSENSE*LTC2942_CHG_M/128) // the number of uC per bit, calculated from equation on datasheet page 11.
#define LTC2942_CHG_MID_SCALE 0x7FFF                                    // the default accumulated charge value
#define LTC2942_VLT_MULT      6                                         // full scale voltage from datasheet page 12.
#define LTC2942_VLT_DIV       65535                                     // full scale A/D reading from datasheet page 12.
#define LTC2942_VLT_DLY       10                                        // delay in ms between starting and reading the voltage measurement.

// These are the values to configure given the usage of the LTC2942.
#define LTC2942_PREBITS          0                                            // the value selected of B[5:3] as per datasheet Table 3.
#define LTC2942_RSENSE           3.32                                         // the sense resistor value on the DC2321A.
#define LTC2942_CHG_LIMIT        (LTC2942_CHG_MID_SCALE-16)                   // the distance from LTC2942_CHG_MID_SCALE to set the charge limits such that we don't lose counts while counting servicing alert.
#define LTC2942_CHG_LIMIT_HIGH   (LTC2942_CHG_MID_SCALE + LTC2942_CHG_LIMIT)  // the upper limit of the charge register before an alert is triggered
#define LTC2942_CHG_LIMIT_LOW    (LTC2942_CHG_MID_SCALE - LTC2942_CHG_LIMIT)  // the lower limit of the charge register before an alert is triggered

// These are precalculated scale factors so that the microprocessors can do minimal math.
#define LTC2942_CHG_MULT      (INT64S)((1LL << LTC2942_CHG_SHIFT) * LTC2942_CHG_QLSB + 0.5)
#define LTC2942_CHG_SHIFT     32

typedef struct
{
   OS_STK         ccAlertTaskStack[TASK_APP_CCALERT_STK_SIZE];
   INT32U         ccAlertChannelBuf[1+DN_CH_ASYNC_RXBUF_SIZE(sizeof(dn_gpio_notif_t))/sizeof(INT32U)];
   OS_EVENT*      ccAlertDataLock;        ///< Binary semaphore so that the coulomb count, which depends upon both the LTC2942 register and the counted number of over/underflows, can be retrieved as a unit.
} cc_alert_vars_t;

typedef struct {
   dn_ioctl_i2c_transfer_t i2cTransfer;
   INT8U                   i2cBuffer[I2C_PAYLOAD_LENGTH];
   OS_EVENT*               i2cDataLock;   ///< Binary semaphore so that only one task can write to i2c at a time.
   INT8U                   commErrFlags;  ///< Flags to record if comm with the i2c device was successful.
} i2c_app_vars_t;

//=========================== prototypes ======================================
static void i2cLockData(void);
static void i2cUnlockData(void);
static void ccLockData(void);
static void ccUnlockData(void);

INT16U getChargeData(coulomb_counter_source_t CC);
INT16U getVoltageData(coulomb_counter_source_t CC);
INT8U CCWriteAndRead8(coulomb_counter_source_t CC, INT8U registerValue);
INT16U CCWriteAndRead16(coulomb_counter_source_t CC, INT8U registerValue);

void selectCC(coulomb_counter_source_t CC);

//=========================== const ===========================================
// LTC4305 commands
const INT8U ltc4305_register_1[] = {0x01, 0xC0};
const INT8U ltc4305_register_2[] = {0x02, 0x24}; //register 2,'connection requirement' = 1 and leave the rest as default
const INT8U ltc4305_register_3_closed[] = {0x03, 0x00};
const INT8U ltc4305_register_3[NUMCC][2] = {{0x03, 0x80},   //register 3, bus 1 FET state = 1
                                            {0x03, 0x40}};  //register 3, bus 2 FET state = 1

// LTC2942 commands
const INT8U ltc2942_register_b_charge[] = {LTC2942_REGB, (LTC2942_PREBITS << 3) | 0x04};  //register B, LTC2942_PREBITS resolution, alert enabled, adc sleep mode
const INT8U ltc2942_register_b_voltage[] = {LTC2942_REGB, (LTC2942_PREBITS << 3) | 0x84};  //register B, LTC2942_PREBITS resolution, alert enabled, manual voltage mode
const INT8U ltc2942_register_cd_default[] = {LTC2942_REGCD, UPPER_BYTE(LTC2942_CHG_MID_SCALE), LOWER_BYTE(LTC2942_CHG_MID_SCALE)};//register C/D, half-scale MSB, half=-scale LSB
const INT8U ltc2942_register_ef[] = {LTC2942_REGEF, UPPER_BYTE(LTC2942_CHG_LIMIT_HIGH), LOWER_BYTE(LTC2942_CHG_LIMIT_HIGH)}; //register E/F, charge high limit MSB, charge high limit LSB
const INT8U ltc2942_register_gh[] = {LTC2942_REGGH, UPPER_BYTE(LTC2942_CHG_LIMIT_LOW), LOWER_BYTE(LTC2942_CHG_LIMIT_LOW)}; //register G/H, charge low limit MSB, charge low limit LSB

//=========================== variables =======================================
cc_alert_vars_t   cc_alert_v;
i2c_app_vars_t    i2c_app_v;

CH_DESC ccAlertNotifChannel; // channel for CC Alert interrupt stored globally for toggling

INT8U selectedCC = -1;
INT16U CCDataArray[NUMCC][NUMDATA-1]; // [0 = PRI, 1 = SEC][0 = CHG, 1 = VLT]. Note that current is calculated from CHG.
INT16S chargeOverflowCounter[NUMCC] = {0, 0}; // counts CC overflows
INT8U ccClearedGuiFlag = FALSE; // tell the GUI that a batch of data will be skewed due to clearing CCs

// Variables for calculating current.
INT32U measureTime;                       // time charge measurement was taken.
INT32U lastMeasureTime[NUMCC];            // time lastChargeValues[] charge measurements were taken.
INT32S lastChargeValues[NUMCC];           // the charge values the last time current measurement was taken.
INT32S lastCurrentValues[NUMCC] = {0, 0}; // last currents to display in case the CCs are cleared and new data is junk

//=========================== tasks ===========================================
// Coulomb counter alert interrupt task
// The coulomb counters are configured by default to count at the highest precision, which means their range is small
// As a result, they are likely to overflow in long-term tests
// When they overflow, the combined alert signal triggers the interrupt
// Upon receiving the interrupt, the CCs are checked to see which overflowed
// An overflowed CC is reset, and an overflow counter is incremented so the cumulative charge can still be computed
static void ccAlertTask(void* unused)
{
   dn_error_t                     dnErr;
   INT8U                          osErr;
   OS_MEM*                        notifChannelMem;
   dn_gpio_ioctl_cfg_in_t         gpioInCfg;
   dn_gpio_ioctl_notif_enable_t   gpioNotifEnable;
   dn_gpio_notif_t                gpioNotif;
   INT32U                         rxLen;
   INT32U                         msgType;
   INT32U                         maxLen;

   // allocate memory for GPIO notification channel
   notifChannelMem = OSMemCreate(cc_alert_v.ccAlertChannelBuf, 1, DN_CH_ASYNC_RXBUF_SIZE(sizeof(dn_gpio_notif_t)), &osErr);
   ASSERT(osErr==OS_ERR_NONE);

   // create channel from memory
   dnErr = dn_createAsyncChannel(notifChannelMem, &ccAlertNotifChannel);
   ASSERT(dnErr == DN_ERR_NONE);

   // enable pull down resistor
   gpioInCfg.pullMode = DN_GPIO_PULL_NONE;
   dnErr = dn_ioctl(CC_ALERT, DN_IOCTL_GPIO_CFG_INPUT, &gpioInCfg, sizeof(gpioInCfg));
   ASSERT(dnErr==DN_ERR_NONE);

   // enable GPIO notification
   gpioNotifEnable.activeLevel    = 1;
   gpioNotifEnable.fEnable        = USBPowerPresent; //enabled if USB is present (USB is checked before this task is run)
   gpioNotifEnable.notifChannelId = ccAlertNotifChannel;
   dnErr = dn_ioctl(CC_ALERT, DN_IOCTL_GPIO_ENABLE_NOTIF, &gpioNotifEnable, sizeof(gpioNotifEnable));
   ASSERT(dnErr == DN_ERR_NONE);

   while(1) // this is a task, it executes forever
   {
      // wait for a GPIO notification
      dnErr = dn_readAsyncMsg(ccAlertNotifChannel, &gpioNotif, &rxLen, &msgType, sizeof(gpioNotif), 0);
      ASSERT(dnErr==DN_ERR_NONE);

      INT8U USBAbsent = digitalRead(USBVCC);
      if(USBAbsent) {
        toggleCCAlertInterrupt(0); //disable interrupt
        return;
      }

      if(gpioNotif.level == 1) { //if the interrupt is triggered (active high)
         INT16U statusReg;

         ccLockData(); // wait to update if other code is in the middle of reading the charge/current
         measureTime = (INT32U)getSystemTime();

         for(INT8U CC = 0; CC < NUMCC; CC++) {
            selectCC(CC);
            statusReg = CCWriteAndRead8(CC, LTC2942_REGA); // read status register (also clears register)
            CCDataArray[CC][CHG] = getChargeData(CC);
            if(statusReg != 0) {
               if(statusReg & LTC2942_REGB_CHARGE_ALERT_LOW) { // low limit, meaning charge OUT of battery.
                  chargeOverflowCounter[CC]--;
                  CCDataArray[CC][CHG] += LTC2942_CHG_LIMIT;
               }
               else if (statusReg & LTC2942_REGB_CHARGE_ALERT_HIGH) { // high limit, meaning charge INTO battery.
                  chargeOverflowCounter[CC]++;
                  CCDataArray[CC][CHG] -= LTC2942_CHG_LIMIT;
               }
               I2CWrite(ltc2942_register_b_charge, LTC2942_ADDRESS, sizeof(ltc2942_register_b_charge));
               INT8U ltc2942_register_cd_new[] = {LTC2942_REGCD, UPPER_BYTE(CCDataArray[CC][CHG]), LOWER_BYTE(CCDataArray[CC][CHG])};
               I2CWrite(ltc2942_register_cd_new, LTC2942_ADDRESS, sizeof(ltc2942_register_cd_new));
               I2CWriteAndRead(LTC2942_AR_ADDRESS, NULL, 0, NULL, 1); //clear alert signal
               CCWriteAndRead8(CC, LTC2942_REGA); //read again after to clear the status register
            }
         }
         ccUnlockData(); // release lock so other code can read the charge/current again.
         toggleCCAlertInterrupt(1); // re-enable interrupt
      }
   }//end task while loop
}

// Enable or disable the CC alert interrupt
// This needs to be disabled if USB power is not present because CCs are powered from USB
// If enabled when USB power is absent, it will be triggered constantly
void toggleCCAlertInterrupt(INT8U enabled)
{
  dn_error_t                     dnErr;
  dn_gpio_ioctl_notif_enable_t   gpioNotifEnable;

  gpioNotifEnable.activeLevel    = 1;
  gpioNotifEnable.fEnable        = enabled;
  gpioNotifEnable.notifChannelId = ccAlertNotifChannel;

  dnErr = dn_ioctl(CC_ALERT, DN_IOCTL_GPIO_ENABLE_NOTIF, &gpioNotifEnable, sizeof(gpioNotifEnable));
  ASSERT(dnErr == DN_ERR_NONE);
}

//=========================== functions ==================================

// Init things that must be done before the OS is started.
// Create the CC alert task
void coulombCounterInit()
{
   INT8U osErr;

   //create the coulomb-counter overflow alert task
   osErr = OSTaskCreateExt(
      ccAlertTask,
      (void *) 0,
      (OS_STK*) (&cc_alert_v.ccAlertTaskStack[TASK_APP_CCALERT_STK_SIZE - 1]),
      TASK_APP_CCALERT_PRIORITY,
      TASK_APP_CCALERT_PRIORITY,
      (OS_STK*) cc_alert_v.ccAlertTaskStack,
      TASK_APP_CCALERT_STK_SIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR
   );
   ASSERT(osErr == OS_ERR_NONE);
   OSTaskNameSet(TASK_APP_CCALERT_PRIORITY, (INT8U*)TASK_APP_CCALERT_NAME, &osErr);
   ASSERT(osErr == OS_ERR_NONE);
}

// Init things that must be done after the OS is started.
void coulombCounterInit2(void)
{
   dn_error_t                   dnErr;
   dn_i2c_open_args_t           i2cOpenArgs;

   // Configure CC GPIO
   initGPIO(CC_ALERT, GPIO_STATE_INPUT);
   initGPIO(USBVCC, GPIO_STATE_INPUT);
   
   // create the binary semaphores if not already created.
   if(i2c_app_v.i2cDataLock == NULL) {
      i2c_app_v.i2cDataLock = OSSemCreate(1);
      ASSERT (i2c_app_v.i2cDataLock!=NULL);
   }

   if(cc_alert_v.ccAlertDataLock == NULL) {
      cc_alert_v.ccAlertDataLock = OSSemCreate(1);
      ASSERT (cc_alert_v.ccAlertDataLock!=NULL);
   }

   // open the I2C device
   i2cOpenArgs.frequency = DN_I2C_FREQ_92_KHZ;//DN_I2C_FREQ_184_KHZ;
   dnErr = dn_open(
      DN_I2C_DEV_ID,
      &i2cOpenArgs,
      sizeof(i2cOpenArgs)
   );
   ASSERT(dnErr==DN_ERR_NONE);

   //init LTC4305
   I2CWrite(ltc4305_register_1, LTC4305_ADDRESS, sizeof(ltc4305_register_1)); //LTC4305 register 1
   I2CWrite(ltc4305_register_2, LTC4305_ADDRESS, sizeof(ltc4305_register_2)); //LTC4305 register 2
   initCCs(); //clear charge register and init
   for(INT8U i = 0; i < NUMCC; i++) {
      selectCC(i);
      I2CWriteAndRead(LTC2942_AR_ADDRESS, NULL, 0, NULL, 1); //clear alert signal
   }
   
   return;
}

// Select which CC is connected to the i2c bus
// Because they both have the same address, the LTC4305 i2c mux is necessary to isolate one at a time
void selectCC(coulomb_counter_source_t CC)
{
   if(CC != selectedCC) {
      I2CWrite(&ltc4305_register_3[CC], LTC4305_ADDRESS, sizeof(ltc4305_register_3[CC]));
      selectedCC = CC;
   }
}

// Initialize both coulomb counters
void initCCs()
{
   for(int i = 0; i < NUMCC; i++) {
      selectCC(i);
      I2CWrite(ltc2942_register_b_charge, LTC2942_ADDRESS, sizeof(ltc2942_register_b_charge));
      I2CWrite(ltc2942_register_cd_default, LTC2942_ADDRESS, sizeof(ltc2942_register_cd_default));
      I2CWrite(ltc2942_register_ef, LTC2942_ADDRESS, sizeof(ltc2942_register_ef));
      I2CWrite(ltc2942_register_gh, LTC2942_ADDRESS, sizeof(ltc2942_register_gh));
      chargeOverflowCounter[i] = 0;
   }
}

// Re-initialize both CCs and set a flag to notify that the current measurements will be skewed
// Use this only when clearing the CCs while running, NOT when just initializing
void clearCCs()
{
  initCCs();
  ccClearedGuiFlag = TRUE;

  updateAllCCData(); // re-read currents so that they are not computed based on the cleared charge register
  getCCData(PRI, CUR);
  getCCData(SEC, CUR);
}

// Take charge and voltage measurements from the CCs and store them in CCDataArray
void updateAllCCData()
{
   ccLockData();  // dont allow these variables to be changed as we set them
   measureTime = (INT32U)getSystemTime();
   for(int CC = 0; CC < NUMCC; CC++) {
      selectCC(CC);
      CCDataArray[CC][CHG] = getChargeData(CC);
      CCDataArray[CC][VLT] = getVoltageData(CC);
   }
   ccUnlockData();
}

// Takes raw data from CCDataArray and outputs usable values at request
// This can also output an avg current based on the change in charge and the time between measurements
// Note that updateAllCCData() should be called before this function to update CCDataArray
INT32S getCCData(coulomb_counter_source_t CC, coulomb_counter_data_t data)
{
   INT32S retval;
   if(i2c_app_v.commErrFlags & MASK(1,CC)) return 0;  // return 0 if failure for IC to communicate.

   switch (data)
   {
      case CHG:
      {
         // Calculate Charge in uC
         ccLockData();  // dont allow these variables to be changed as we combine them
         retval = (INT32S)(LTC2942_CHG_MID_SCALE - CCDataArray[CC][data]) - chargeOverflowCounter[CC] * LTC2942_CHG_LIMIT;
         ccUnlockData();
         retval = SHIFT_W_ROUND((INT64S)retval * LTC2942_CHG_MULT, LTC2942_CHG_SHIFT);
         break;
      }
      case VLT:
      {
         // Calculate Charge in uV
         retval= CCDataArray[CC][data];
         retval = DIV_W_ROUND((INT64S)retval * LTC2942_VLT_MULT * UV_PER_V, LTC2942_VLT_DIV);
         break;
      }
      case CUR:
      {
         // Calculate Current in nA
         ccLockData();  // dont allow these variables to be changed as we combine them
         INT32U deltaTime = measureTime - lastMeasureTime[CC];
         lastMeasureTime[CC] = measureTime; // set lastMeasureTime for next measurement
         INT32S newChargeValue = (INT32S)(LTC2942_CHG_MID_SCALE - CCDataArray[CC][CHG]) - chargeOverflowCounter[CC] * LTC2942_CHG_LIMIT;
         ccUnlockData();
         INT64S deltaCoulombs = newChargeValue - lastChargeValues[CC];
         deltaCoulombs = SHIFT_W_ROUND(deltaCoulombs * LTC2942_CHG_MULT, LTC2942_CHG_SHIFT);
         retval = DIV_W_ROUND(deltaCoulombs * TIMER_TICKS_PER_SEC * NA_PER_UA, deltaTime);
         lastCurrentValues[CC] = retval; // store the last computed current to display in case the CCs are cleared
         lastChargeValues[CC] = newChargeValue;
         break;
      }
   }
   return retval;
}

// Reads an INT8U from a given register on the selected LTC2942
// Note that selectCC() should be called before this function to select the correct LTC2942
INT8U CCWriteAndRead8(coulomb_counter_source_t CC, INT8U registerValue)
{
   INT8U temp_buffer[sizeof(INT8U)];
   dn_error_t dnErr = I2CWriteAndRead(LTC2942_ADDRESS, &registerValue, sizeof(registerValue), temp_buffer, sizeof(temp_buffer));
   setErrFlag(CC, dnErr);
   return temp_buffer[0];
}

// Reads an INT16U from a given register on the selected LTC2942
// Note that selectCC() should be called before this function to select the correct LTC2942
INT16U CCWriteAndRead16(coulomb_counter_source_t CC, INT8U registerValue)
{
   INT8U temp_buffer[sizeof(INT16U)];
   dn_error_t dnErr = I2CWriteAndRead(LTC2942_ADDRESS, &registerValue, sizeof(registerValue), temp_buffer, sizeof(temp_buffer));
   setErrFlag(CC, dnErr);
   return ((INT16U)temp_buffer[0] << 8) + temp_buffer[1];
}

// Generic i2c read after sending a read command
dn_error_t I2CWriteAndRead(INT8U address, INT8U* readCommand, INT8U numBytesSent, INT8U* readBuffer, INT8U numBytesReceived)
{
   dn_error_t                   dnErr;

   i2cLockData();

   // prepare buffer
   memcpy(i2c_app_v.i2cBuffer, readCommand, min(numBytesSent,sizeof(i2c_app_v.i2cBuffer)));

   // initialize I2C communication parameters
   i2c_app_v.i2cTransfer.slaveAddress    = address;
   i2c_app_v.i2cTransfer.writeBuf        = i2c_app_v.i2cBuffer;
   i2c_app_v.i2cTransfer.readBuf         = i2c_app_v.i2cBuffer;
   i2c_app_v.i2cTransfer.writeLen        = min(numBytesSent,sizeof(i2c_app_v.i2cBuffer));
   i2c_app_v.i2cTransfer.readLen         = numBytesReceived;
   i2c_app_v.i2cTransfer.timeout         = I2C_TIMEOUT;
     
   // initiate transaction
   dnErr = dn_ioctl(
      DN_I2C_DEV_ID,
      DN_IOCTL_I2C_TRANSFER,
      &i2c_app_v.i2cTransfer,
      sizeof(i2c_app_v.i2cTransfer)
   );
   
   if(readBuffer != NULL) {
      memcpy(readBuffer, i2c_app_v.i2cBuffer, numBytesReceived);
   }

   i2cUnlockData();

   return dnErr;
}

// Generic i2c write where no response is expected
dn_error_t I2CWrite(INT8U* dataPtr, INT8U address, INT8U numBytes)
{
   dn_error_t                   dnErr; //not checked
   INT8U                        i;

   i2cLockData();

   memcpy(i2c_app_v.i2cBuffer, dataPtr, min(numBytes,sizeof(i2c_app_v.i2cBuffer)));

   // initialize I2C communication parameters
   i2c_app_v.i2cTransfer.slaveAddress    = address;
   i2c_app_v.i2cTransfer.writeBuf        = i2c_app_v.i2cBuffer;
   i2c_app_v.i2cTransfer.readBuf         = NULL;
   i2c_app_v.i2cTransfer.writeLen        = min(numBytes,sizeof(i2c_app_v.i2cBuffer));
   i2c_app_v.i2cTransfer.readLen         = 0;
   i2c_app_v.i2cTransfer.timeout         = I2C_TIMEOUT;

   // initiate transaction
   dnErr = dn_ioctl(
      DN_I2C_DEV_ID,
      DN_IOCTL_I2C_TRANSFER,
      &i2c_app_v.i2cTransfer,
      sizeof(i2c_app_v.i2cTransfer)
   );
   
   i2cUnlockData();

   return dnErr;
}

// Set an error flag if we encounter an error in i2c communication
// This is used to set CC data to '0' when we receive junk due to an error
void setErrFlag(coulomb_counter_source_t selectedCC, dn_error_t dnErr) {
   if(dnErr < DN_ERR_NONE) {
      i2c_app_v.commErrFlags |= MASK(1,selectedCC);
   }
   else {
      i2c_app_v.commErrFlags &= ~MASK(1,selectedCC);
   }
}

// Wait until CC data is unlocked and free for editing
// Getting through the pend re-enables the lock
static void ccLockData(void) {
   INT8U      osErr;
   OSSemPend(cc_alert_v.ccAlertDataLock, 0, &osErr);
   ASSERT(osErr == OS_ERR_NONE);
}

// Enable modifications to the CC data
static void ccUnlockData(void) {
   OSSemPost(cc_alert_v.ccAlertDataLock);
}

// Wait until i2c_app_v.i2cBuffer is unlocked and free for editing
// Getting through the pend re-enables the lock
static void i2cLockData(void) {
   INT8U      osErr;
   OSSemPend(i2c_app_v.i2cDataLock, 0, &osErr);
   ASSERT(osErr == OS_ERR_NONE);
}

// Enable modifications to i2c_app_v.i2cBuffer
static void i2cUnlockData(void) {
   OSSemPost(i2c_app_v.i2cDataLock);
}

// Take a charge reading from a CC
INT16U getChargeData(coulomb_counter_source_t CC)
{
   selectCC(CC);
   INT16U chargeData = CCWriteAndRead16(CC, LTC2942_REGCD);
   return chargeData;
}

// Take a voltage measurement from a CC
INT16U getVoltageData(coulomb_counter_source_t CC)
{
   selectCC(CC);
   I2CWrite(ltc2942_register_b_voltage, LTC2942_ADDRESS, sizeof(ltc2942_register_b_voltage));
   OSTimeDly(LTC2942_VLT_DLY);
   INT16U voltageData = CCWriteAndRead16(CC, LTC2942_REGIJ);
   return voltageData;
}
