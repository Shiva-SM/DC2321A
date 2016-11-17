/*
Copyright (c) 2016, Dust Networks.  All rights reserved.
*/

#ifndef DC2321_H
#define DC2321_H

#include "dn_system.h"

//=========================== enums  ==========================================
typedef enum {EHVCC, USB3V3, NUMPS} power_source_t;

//=========================== defines =========================================
// FW Revision
#define APP_VER_MAJOR            0x01
#define APP_VER_MINOR            0x00
#define APP_VER_PATCH            0x00

// GPIO Definition
#define WRITE_EPD                DN_GPIO_PIN_14_DEV_ID
#define BAT_OFF_LTC3107          DN_GPIO_PIN_17_DEV_ID
#define EH_ON_LTC3330            DN_GPIO_PIN_18_DEV_ID
#define EH_ON_LTC3331            DN_GPIO_PIN_19_DEV_ID
#define PGOOD_LTC3106            DN_GPIO_PIN_20_DEV_ID
#define PGOOD_LTC3107            DN_GPIO_PIN_21_DEV_ID
#define PGOOD_LTC3330            DN_GPIO_PIN_22_DEV_ID
#define PGOOD_LTC3331            DN_GPIO_PIN_23_DEV_ID
#define SRC                      DN_GPIO_PIN_0_DEV_ID
#define PIN_NOTIF                DN_GPIO_PIN_26_DEV_ID
#define CC_ALERT                 DN_GPIO_PIN_16_DEV_ID
#define USBVCC                   DN_GPIO_PIN_6_DEV_ID

// Pin Definitions for Driving EPD
#define PANEL_ON                 DN_GPIO_PIN_13_DEV_ID
#define BUSY                     DN_GPIO_PIN_0_DEV_ID
#define SSn                      DN_GPIO_PIN_12_DEV_ID
#define PWM                      DN_GPIO_PIN_16_DEV_ID
#define RESETn                   DN_GPIO_PIN_1_DEV_ID
#define DISCHARGE                DN_GPIO_PIN_15_DEV_ID
#define CLK                      DN_GPIO_PIN_9_DEV_ID
#define MOSI                     DN_GPIO_PIN_10_DEV_ID
#define MISO                     DN_GPIO_PIN_11_DEV_ID

// Definitions of bitmap of EH sources.
#define EHDATA_NONE              0x00
#define EHDATA_PGOOD_LTC3106     0x80
#define EHDATA_PGOOD_LTC3107     0x40
#define EHDATA_PGOOD_LTC3330     0x20
#define EHDATA_PGOOD_LTC3331     0x10
#define EHDATA_INIT              0x08
#define EHDATA_BAT_OFF_LTC3107   0x04
#define EHDATA_EH_ON_LTC3330     0x02
#define EHDATA_EH_ON_LTC3331     0x01
#define EHDATA_PGOOD             (EHDATA_PGOOD_LTC3106 | EHDATA_PGOOD_LTC3107 | EHDATA_PGOOD_LTC3330 | EHDATA_PGOOD_LTC3331)

// General purpose macros and constants
#define LENGTH(x)                (sizeof(x)/sizeof(*x))
#define BITS_PER_BYTE            8
#define BITS_TO_BYTES(x)         (x / 8 + (x % 8 ? 1 : 0))
#define MASK(size, shift)        (((1LL << (size)) - 1) << (shift))
#define UPPER_NIBBLE(x)          ((INT8U)((x) >> 4) & 0x0f)
#define LOWER_NIBBLE(x)          ((INT8U)(x) & 0x0f)
#define UPPER_BYTE(x)            ((INT8U)(((x) >> 8) & 0xff))
#define LOWER_BYTE(x)            ((INT8U)((x) & 0xff))
#define UPPER_WORD(x)            ((INT16U)(((x) >> 16) & 0xffff))
#define LOWER_WORD(x)            ((INT16U)((x) & 0xffff))
#define SHIFT_W_ROUND(n,s)       (((n) + (1LL<<((s)-1))) >> (s))
#define DIV_W_ROUND(n,d)         (((n) + ((n) >= 0 ? (d)/2 : -(d)/2)) / (d))

#define MC_PER_C                 1000
#define UC_PER_MC                1000
#define UC_PER_C                 1000000
#define MV_PER_V                 1000
#define UV_PER_MV                1000
#define UV_PER_V                 1000000
#define MA_PER_A                 1000
#define UA_PER_MA                1000
#define UA_PER_A                 1000000
#define NA_PER_UA                1000
#define NA_PER_MA                1000000
#define NA_PER_A                 1000000000
#define US_PER_SEC               1000000
#define MS_PER_SEC               1000
#define SEC_PER_HR               3600

//=========================== global variables ================================
extern INT8U USBPowerPresent;
extern INT8U usbSlaveMode;
extern power_source_t powerSource;
extern INT8U autoUpdate;
extern INT16U autoUpdateDelay;
extern INT32U lastAutoUpdateTime;

//=========================== prototypes ======================================
void displayCurrentScreen(INT8U forceUpdate);
void checkUSBPower();
INT8U enoughEnergyToUpdateEPD();
INT8U getEHData();
void runStandardChecks();

#endif
