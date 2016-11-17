/**
Copyright (c) 2016, Dust Networks.  All rights reserved.

Initialization and helper functions for mote devices:
 - ADCs
 - Temperatures measurements
 - System time
 - GPIOs
 - Supply voltage measurements
*/

#include "loc_task.h"
#include "dn_system.h"
#include "dn_gpio.h"
#include "dn_adc.h"
#include "dn_time.h"

#include "dc2321a.h"
#include "devices.h"

//=========================== public ==========================================
// Reads one of the four integrated ADCs on the mote
// Returns an INT16U in 100uVs (10,000ths of a volt)
INT16U readADC(INT8U adcIndex)
{
   adcIndex += 2; // ADC0 enum is defined as 2, the rest sequentially

   dn_error_t dnErr;
   int numBytesRead;
   dn_adc_drv_open_args_t  openArgs;
   INT16U adcVal; // each LSB is 1.8mV

   openArgs.rdacOffset  = 0;
   openArgs.vgaGain     = 0;
   openArgs.fBypassVga  = 1;
   dnErr = dn_open(adcIndex, &openArgs, sizeof(openArgs));
   ASSERT(dnErr == DN_ERR_NONE);

   numBytesRead = dn_read(adcIndex, &adcVal, sizeof(adcVal));
   ASSERT(numBytesRead == sizeof(adcVal));

   dnErr = dn_close(adcIndex);
   ASSERT(dnErr == DN_ERR_NONE);

   return adcVal;
}

// Returns temperature in hundredths of a degree celsius
INT16S getTemp() 
{
   int              dnErr;
   int                     numBytesRead;
   INT16S                  temperature;

   // open temperature sensor
   dnErr = dn_open(
      DN_TEMP_DEV_ID,             // device
      NULL,                       // args
      0                           // argLen
   );
   ASSERT(dnErr==DN_ERR_NONE);

   // read temperature value
   numBytesRead = dn_read(
      DN_TEMP_DEV_ID ,         // device
      &temperature,            // buf
      sizeof(temperature)      // bufSize
   );
   ASSERT(numBytesRead== sizeof(temperature));
  
   dnErr = dn_close(
      DN_TEMP_DEV_ID             // device
   );
   ASSERT(dnErr==DN_ERR_NONE);
   
   return temperature;
}

// Returns system time in (1/TIMER_TICKS_PER_SEC) seconds
// Time in seconds = (getSystemTime() / TIMER_TICKS_PER_SEC)
INT64U getSystemTime(void)
{
   INT64U temp_time;
   dn_getSystemTime(&temp_time);
   return temp_time;
}

// Sets a GPIO output high or low
// Must be configured as an output first
// Output is 1 for high, 0 for low
void digitalWrite(int pin, INT8U output)
{
   dn_error_t              dnErr;
   dnErr = dn_write(pin, &output, sizeof(output));
   ASSERT(dnErr==DN_ERR_NONE);
}

// Reads a GPIO input
// Must be configured as an input first
// Returns 1 for high, 0 for low
INT8U digitalRead(int pin)
{
   dn_error_t              dnErr;
   INT8U pinState; //was INT8U
   dnErr = dn_read(pin, &pinState, sizeof(pinState));
   ASSERT(dnErr==DN_ERR_NONE);  
   return pinState & 0x01;
}

// OPENS a gpio and then configures a GPIO as an output or an input
// 0utput = 0 and 1nput = 1
void initGPIO(int pin, INT8U pinState)
{
   dn_error_t              dnErr;

   // open pin
   dnErr = dn_open(pin, NULL, 0);  // device, args, argLen
   ASSERT(dnErr==DN_ERR_NONE);

   configGPIO(pin, pinState);
}

// Configures a GPIO as an output or an input AFTER it has already been opened
// Default state for outputs is 'low'
void configGPIO(int pin, INT8U pinState) //pin name, output = 0 and input = 1
{
   dn_error_t              dnErr;

   if(pinState == 0) { // configure as output
      dn_gpio_ioctl_cfg_out_t gpioOutCfg;
      gpioOutCfg.initialLevel = 0x00;
      dnErr = dn_ioctl(pin, DN_IOCTL_GPIO_CFG_OUTPUT, &gpioOutCfg, sizeof(gpioOutCfg)); // device, request, args, argLen
      ASSERT(dnErr==DN_ERR_NONE);
   }
   else { // configure as input
      dn_gpio_ioctl_cfg_in_t gpioInCfg;
      gpioInCfg.pullMode = DN_GPIO_PULL_NONE; // pull UP, DOWN, NONE
      dnErr = dn_ioctl(pin, DN_IOCTL_GPIO_CFG_INPUT, &gpioInCfg, sizeof(gpioInCfg)); // device, request, args, argLen
      ASSERT(dnErr==DN_ERR_NONE);
   }
}

// Gets the supply voltage of the mote
// Returns an INT16U in 100uVs (10,000ths of a volt)
INT16U getSupplyV(void)
{
   int                       numBytesRead;
   INT16U                    voltage;

   numBytesRead = dn_read(
      DN_BATT_DEV_ID,          // device
      &voltage,                // buf
      sizeof(voltage)          // bufSize
   );
   ASSERT(numBytesRead == sizeof(INT16U));

   if(powerSource == USB3V3) // if powered by USB, add compensation factor (reading is consistently low due to diode drop)
      return voltage + 1300;
   else
      return voltage + 300; // += 0.03 compensation (due to low voltage value readings)
}

// Initialize the ADC that reads the mote's supply voltage
void initBatADC()
{
   dn_error_t dnErr;

   // open battery sensor
   dn_adc_drv_open_args_t    openArgs;
   openArgs.loadBattery = DN_ADC_LOAD_BATT_NONE;
   openArgs.rdacOffset = 2;
   openArgs.vgaGain = 2;
   openArgs.fBypassVga = 0;
   dnErr = dn_open(DN_BATT_DEV_ID, &openArgs, sizeof(openArgs));
   ASSERT(dnErr==DN_ERR_NONE);
}
