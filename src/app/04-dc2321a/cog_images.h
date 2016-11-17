#ifndef COG_IMAGES_H
#define COG_IMAGES_H

#include "dn_typedef.h"
#include "dc2321a.h"
#include "cog_driver.h"

// Dimensions of images in bits.
#define COG_IMAGE_NUM_HEIGHT    13
#define COG_IMAGE_NUM_WIDTH     8
#define COG_IMAGE_YESNO_HEIGHT  7
#define COG_IMAGE_YESNO_WIDTH   24
#define COG_IMAGE_UNITS_HEIGHT  11
#define COG_IMAGE_UNITS_WIDTH   10
#define COG_IMAGE_AU_HEIGHT     8
#define COG_IMAGE_AU_WIDTH      11
#define COG_IMAGE_LTC_HEIGHT    11
#define COG_IMAGE_LTC_WIDTH     23
#define COG_IMAGE_PS_HEIGHT     61
#define COG_IMAGE_PS_WIDTH      79
#define COG_IMAGE_BAT_HEIGHT    11
#define COG_IMAGE_BAT_WIDTH     24

// 2D arrays containing image bits.
typedef INT8U COG_IMAGE_NUM[COG_IMAGE_NUM_HEIGHT][BITS_TO_BYTES(COG_IMAGE_NUM_WIDTH)];   // Images containing digits 0-9
typedef INT8U COG_IMAGE_YESNO[COG_IMAGE_YESNO_HEIGHT][BITS_TO_BYTES(COG_IMAGE_YESNO_WIDTH)];  // Images containing "YES" and "NO"
typedef INT8U COG_IMAGE_SIGN[COG_IMAGE_UNITS_HEIGHT][BITS_TO_BYTES(COG_IMAGE_UNITS_WIDTH)];   // Images containing + and -
typedef INT8U COG_IMAGE_UNITS[COG_IMAGE_UNITS_HEIGHT][BITS_TO_BYTES(COG_IMAGE_UNITS_WIDTH)]; // Images containing units.
typedef INT8U COG_IMAGE_AU[COG_IMAGE_AU_HEIGHT][BITS_TO_BYTES(COG_IMAGE_AU_WIDTH)]; // Image containing arrows for auto refresh
typedef INT8U COG_IMAGE_LTC[COG_IMAGE_LTC_HEIGHT][BITS_TO_BYTES(COG_IMAGE_LTC_WIDTH)]; // Images containing names of LTC ICs
typedef INT8U COG_IMAGE_PS[COG_IMAGE_PS_HEIGHT][BITS_TO_BYTES(COG_IMAGE_PS_WIDTH)]; // Images containing names of power sources
typedef INT8U COG_IMAGE_BAT[COG_IMAGE_BAT_HEIGHT][BITS_TO_BYTES(COG_IMAGE_BAT_WIDTH)]; // Images containing names of battery sources

//===============ENGINEERING DISPLAY======================================
 extern const COG_DRIVER_IMAGE icStatusScreen;
 extern const COG_DRIVER_IMAGE batStatusScreen;

 extern const COG_IMAGE_YESNO noImg;
 extern const COG_IMAGE_YESNO yesImg;

 extern const COG_IMAGE_UNITS microAmps;
 extern const COG_IMAGE_UNITS milliAmps;
 extern const COG_IMAGE_UNITS milliCoulombs;
 extern const COG_IMAGE_UNITS coulombs;

 extern const COG_IMAGE_NUM num0;
 extern const COG_IMAGE_NUM num1;
 extern const COG_IMAGE_NUM num2;
 extern const COG_IMAGE_NUM num3;
 extern const COG_IMAGE_NUM num4;
 extern const COG_IMAGE_NUM num5;
 extern const COG_IMAGE_NUM num6;
 extern const COG_IMAGE_NUM num7;
 extern const COG_IMAGE_NUM num8;
 extern const COG_IMAGE_NUM num9;

 extern const COG_IMAGE_AU auImg;
 extern const COG_IMAGE_AU auImgOff;

 //===============SIMPLE DISPLAY======================================
 extern const COG_DRIVER_IMAGE summaryScreen;
 extern const COG_DRIVER_IMAGE startupScreen;

 extern const COG_IMAGE_LTC LTC3106Img;
 extern const COG_IMAGE_LTC LTC3107Img;
 extern const COG_IMAGE_LTC LTC3330Img;
 extern const COG_IMAGE_LTC LTC3331Img;

 extern const COG_IMAGE_PS batImg;
 extern const COG_IMAGE_PS solarImg;
 extern const COG_IMAGE_PS piezoOrSolarImg;
 extern const COG_IMAGE_PS thermalImg;
 extern const COG_IMAGE_PS batOrSolarImg;

 extern const COG_IMAGE_BAT priImg;
 extern const COG_IMAGE_BAT secImg;

#endif