/*
Copyright (c) 2016, Dust Networks.  All rights reserved.

This file contains the code to:
 - Implement a SPI interface.
 - COG (chip on glass) Driver for 1.44" EPD (E-PAPER DISPLAY) from PERVASIVE DISPLAYS, E1144CS021
   All references to the COG datasheet refer to Doc No. 4P018-00 from Pervasive Displays.

This file contains all functions used to interface directly with the COG on the display.
This includes power up, initialization, writing a screen, and powering down.
To write a screen, a bitmap buffer is passed to this file by the cog_display.c file.
*/

#include <string.h>

#include "loc_task.h"
#include "dn_system.h"
#include "dn_gpio.h"
#include "dn_spi.h"

#include "dc2321a.h"
#include "cog_driver.h"
#include "cog_images.h"
#include "devices.h"

//=========================== defines =========================================
#define SPI_READ_ENABLED     FALSE                       // SPI read is disabled so that we can drive MISO high/low to prevent leakage currents
#define LOW_POWER_DELAY      FALSE                       // If low-power, uses OSTimeDly() for all display driving delays
#define COG_NUM_SCAN_BYTES   (COG_NUM_LINES / (8 / 2))   // Scan byte contain 2 bits per line

#define SPI_BUFFER_LENGTH    64                          // Set as high as the most bytes that need to be sent at once.
typedef struct {
   INT8U                     spiTxBuffer[SPI_BUFFER_LENGTH];
   INT8U                     spiRxBuffer[SPI_BUFFER_LENGTH];
   INT8U                     spiTxBufferCounter;
   INT8U                     spi_flag;
} spi_app_vars_t;

#define EPD_IsBusy()          FALSE // Not using BUSY -> BOOLEAN)input_get(EPD_BUSY_PORT,EPD_BUSY_PIN)
#define EPD_cs_high()         digitalWrite(SSn, GPIO_OUTPUT_HIGH) //set_gpio_high(EPD_CS_PORT,EPD_CS_PIN)
#define EPD_cs_low()          digitalWrite(SSn, GPIO_OUTPUT_LOW) //set_gpio_low(EPD_CS_PORT,EPD_CS_PIN)
#define EPD_rst_high()        digitalWrite(RESETn, GPIO_OUTPUT_HIGH) //set_gpio_high(EPD_RST_PORT,EPD_RST_PIN)
#define EPD_rst_low()         digitalWrite(RESETn, GPIO_OUTPUT_LOW) //set_gpio_low(EPD_RST_PORT,EPD_RST_PIN)
#define EPD_discharge_high()  digitalWrite(DISCHARGE, GPIO_OUTPUT_HIGH) //set_gpio_high(EPD_DISCHARGE_PORT,EPD_DISCHARGE_PIN)
#define EPD_discharge_low()   digitalWrite(DISCHARGE, GPIO_OUTPUT_LOW) //set_gpio_low(EPD_DISCHARGE_PORT,EPD_DISCHARGE_PIN)
#define EPD_Vcc_turn_on()     digitalWrite(PANEL_ON, GPIO_OUTPUT_HIGH) //set_gpio_high(EPD_PANELON_PORT,EPD_PANELON_PIN)
#define EPD_Vcc_turn_off()    digitalWrite(PANEL_ON, GPIO_OUTPUT_LOW) //set_gpio_low(EPD_PANELON_PORT,EPD_PANELON_PIN)
#define EPD_border_high()     // BORDER is hardwired -> set_gpio_high(EPD_BORDER_PORT,EPD_BORDER_PIN)
#define EPD_border_low()      // BORDER is hardwired -> set_gpio_low(EPD_BORDER_PORT,EPD_BORDER_PIN)
#define EPD_pwm_high()        // No PWM -> set_gpio_high(PWM_PORT,PWM_PIN)
#define EPD_pwm_low()         // No PWM -> set_gpio_low(PWM_PORT,PWM_PIN)
#define SPIMISO_low()         // -> set_gpio_low(SPIMISO_PORT,SPIMISO_PIN)
#define SPIMOSI_low()         // -> set_gpio_low(SPIMOSI_PORT,SPIMOSI_PIN)
#define SPICLK_low()          // -> set_gpio_low(SPICLK_PORT,SPICLK_PIN)

#define sys_delay_ms(ms)      delay_ms(ms)
#define Wait_10us()           // our application writes to SPI so slowly that there is never a need to delay between SPI transactions.

  /**
 * \brief The definition for driving stage to compare with for getting Odd and Even data  */
#define BLACK0   (INT8U)(0x03) /**< getting bit1 or bit0 as black color(11) */
#define BLACK1   (INT8U)(0x0C) /**< getting bit3 or bit2 as black color(11) */
#define BLACK2   (INT8U)(0x30) /**< getting bit5 or bit4 as black color(11) */
#define BLACK3   (INT8U)(0xC0) /**< getting bit7 or bit6 as black color(11) */
#define WHITE0   (INT8U)(0x02) /**< getting bit1 or bit0 as white color(10) */
#define WHITE1   (INT8U)(0x08) /**< getting bit3 or bit2 as white color(10) */
#define WHITE2   (INT8U)(0x20) /**< getting bit5 or bit4 as white color(10) */
#define WHITE3   (INT8U)(0x80) /**< getting bit7 or bit6 as white color(10) */
#define NOTHING0 (INT8U)(0x01) /**< getting bit1 or bit0 as nothing input(01) */
#define NOTHING1 (INT8U)(0x04) /**< getting bit3 or bit2 as nothing input(01) */
#define NOTHING2 (INT8U)(0x10) /**< getting bit5 or bit4 as nothing input(01) */
#define NOTHING3 (INT8U)(0x40) /**< getting bit7 or bit6 as nothing input(01) */
#define NOTHING  (INT8U)(0x55) /**< sending Nothing frame, 01=Nothing, 0101=0x5 */

#define ALL_BLACK          (INT8U)(0xFF)
#define ALL_WHITE          (INT8U)(0xAA)
#define BORDER_BYTE_B      (INT8U)(0xFF)
#define BORDER_BYTE_W      (INT8U)(0xAA)
#define ERROR_BUSY         (INT8U)(0xF0)
#define ERROR_COG_ID       (INT8U)(0xF1)
#define ERROR_BREAKAGE     (INT8U)(0xF2)
#define ERROR_DC           (INT8U)(0xF3)
#define ERROR_CHARGEPUMP   (INT8U)(0xF4)
#define RES_OK             (INT8U)(0x00)

#define COG_DRIVER_UPDATE_SPEED       (0.5) // Decreases update rate of COG from recommended D_PartialUpdate_StageTime
#define D_PartialUpdate_StageTime     (480 * COG_DRIVER_UPDATE_SPEED)  // ms

#define Line_Offset(index) COG_parameters[index].horizontal_size;
/**
 * \brief The COG Driver uses a buffer to update the EPD line by line.
   \note Use the 2.7" maximum data(66)+scan(44)+dummy(1) bytes as line buffer size=111.*/
#define LINE_BUFFER_DATA_SIZE 111
#define COG_line_Max_Size     33

/**
 * \brief Line data structure of 1.44 inch Aurora_Ma EPD with G2 COG
 * \note
 * Refer to COG G2 document Section 5.3 - 1.44" Input Data Order.
 */
struct COG_144_line_data_t {
   INT8U even[COG_NUM_DOTS_PACKED]; /**< 1.44" even byte array */
   INT8U scan[COG_NUM_SCAN_BYTES]; /**< 1.44" scan byte array */
   INT8U odd [COG_NUM_DOTS_PACKED]; /**< 1.44" odd byte array */
   INT8U border_byte; /**< Internal border_control*/
} ;

/**
 * \brief Packet structure of a line data */
typedef union {
   union {
      struct COG_144_line_data_t line_data_for_144; /**< line data structure of 1.44" EPD */
    } line_data_by_size; /**< the line data of specific EPD size */
   INT8U uint8[LINE_BUFFER_DATA_SIZE]; /**< the maximum line buffer data size as length */
} COG_line_data_packet_type;

/**
 * \brief Define the COG driver's parameters */
struct COG_parameters_t {
   INT8U   channel_select[8]; /**< the SPI register data of Channel Select */
   INT8U   voltage_level;     /**< the SPI register data of Voltage Level */
   INT16U  horizontal_size;   /**< the bytes of width of EPD */
   INT16U  vertical_size;     /**< the bytes of height of EPD */
   INT8U   data_line_size;    /**< Data + Scan + Dummy bytes */
   INT16U  frame_time_offset; /**< the rest of frame time in a stage */
   INT16U  stage_time;        /**< defined stage time */
} ;

//=========================== prototypes ======================================
static INT8U spiWriteReadByte(INT8U dataSent);
static INT8U spiSendBuffer(INT8U length); 
void epd_spi_send(INT8U register_index, INT8U *register_data, INT16U length);
void epd_spi_send_byte(INT8U index, INT8U data);
INT8U SPI_R(INT8U Register, INT8U Data);

void EPD_initialize_gpio(void);
void epd_spi_init(void);
void epd_spi_attach(void);
void epd_spi_detach(void);
void EPD_display_from_array_prt(INT8U *previous_image_ptr,INT8U *new_image_ptr, INT8U flash);

void set_temperature_factor(INT8S temperature);
void border_dummy_line(void);
void delay_ms(INT16U ms);

//=========================== global variables ==================================
static spi_app_vars_t                spi_app_v;
static dn_ioctl_spi_transfer_t       spiTransfer;

static COG_line_data_packet_type    COG_Line;
static INT16U                       current_frame_time;
static INT16U                       stage_time;
static INT8U                        *data_line_even;
static INT8U                        *data_line_odd;
static INT8U                        *data_line_scan;
static INT8U                        *data_line_border_byte;
INT16U                              PartialUpdate_StageTime = D_PartialUpdate_StageTime;

/**
 * \brief The COG parameters of different EPD size
 */
const struct COG_parameters_t COG_parameters =
{
   // FOR 1.44"
   {0x00,0x00,0x00,0x00,0x00,0x0F,0xFF,0x00},  // channel_select[8]
   0x03,                                       // voltage_level
   (128/8),                                    // horizontal_size
   96,                                         // vertical_size
   ((((128+96)*2)/8)+1),                       // data_line_size
   0,                                          // frame_time_offset
   D_PartialUpdate_StageTime                   // stage_time
};

const INT8U SCAN_TABLE[4] = {0xC0,0x30,0x0C,0x03};

/* Temperature factor combines with stage time (in ms) for each driving stage */
const INT16U temperature_table[8] = {
   (D_PartialUpdate_StageTime*17),
   (D_PartialUpdate_StageTime*12),
   (D_PartialUpdate_StageTime*8),
   (D_PartialUpdate_StageTime*4),
   (D_PartialUpdate_StageTime*3),
   (D_PartialUpdate_StageTime*2),
   (D_PartialUpdate_StageTime*1),
   (D_PartialUpdate_StageTime*0.7) // 1.44"
};

//=========================== functions ==================================

/**
 * \brief Initialize the EPD hardware setting
 */
void EPD_display_hardware_init(void) {
   EPD_initialize_gpio();
   EPD_Vcc_turn_off();
   epd_spi_init();
   configGPIO(MISO, GPIO_STATE_INPUT); // reconfigure MISO as an input after initializing SPI
   
   // we're not temperature compensating -> initialize_temperature();

   EPD_cs_low();
   EPD_pwm_low();
   EPD_rst_low();
   EPD_discharge_low();
   EPD_border_low();
}

/**
 * \brief Update image on display from the pointer of memory array by completed global update cycle
 *
 * \param EPD_type_index The defined EPD size
 * \param previous_image_ptr The pointer of memory that stores previous image
 * \param new_image_ptr The pointer of memory that stores new image
 */
void EPD_display_from_pointer(INT8U *previous_image_ptr, INT8U *new_image_ptr, INT8U flash) {
   /* Power on COG Driver */
   EPD_power_on();

   /* Initialize COG Driver */
   EPD_initialize_driver();

   /* Display image data on EPD from image array */
   EPD_display_from_array_prt(previous_image_ptr, new_image_ptr, flash);

   /* Power off COG Driver */
   EPD_power_off();
}

// Read a byte from SPI after sending a byte SPI command
static INT8U spiWriteReadByte(INT8U dataSent) {
  
   // Setup to transmit and receive 1 byte of data.
   spi_app_v.spiTxBuffer[0] = dataSent;

   // Return received data.
   return spiSendBuffer(sizeof(dataSent));
}

// Send the whole spi buffer to a specified length
static INT8U spiSendBuffer(INT8U length) {
  
   // Setup to transmit length bytes of data.
   spiTransfer.transactionLen = length;
   
   // Send the data.
   int err = dn_ioctl(
      DN_SPI_DEV_ID,
      DN_IOCTL_SPI_TRANSFER,
      &spiTransfer,
      sizeof(spiTransfer)
   );
   ASSERT(err >= DN_ERR_NONE);

   // Mark data as received.
   spi_app_v.spiTxBufferCounter = 0;

   return spi_app_v.spiRxBuffer[0];
}

// Send a standard COG command with a register index and data
void epd_spi_send_byte(INT8U register_index, INT8U register_data) { //standard SPI send to COG, sends index then command (implements flowchart on page 17 for special case where bytes = 1)

   spi_app_v.spiTxBuffer[0] = 0x70; // header of Register Index
   spi_app_v.spiTxBuffer[1] = register_index;
   EPD_cs_low ();
   spiSendBuffer(2);
   EPD_cs_high ();

   Wait_10us ();

   spi_app_v.spiTxBuffer[0] = 0x72; // header of Register Data
   spi_app_v.spiTxBuffer[1] = register_data;
   EPD_cs_low ();
   spiSendBuffer(2);
   EPD_cs_high ();
}

/**
* \brief SPI command if register data is larger than two bytes
*
* \param register_index The Register Index as SPI command to COG
* \param register_data The Register Data for sending command data to COG
* \param length The number of bytes of Register Data which depends on which
* Register Index is selected.
*/
void epd_spi_send (INT8U register_index, INT8U *register_data, INT16U length) {
  
   spi_app_v.spiTxBuffer[0] = 0x70; // header of Register Index
   spi_app_v.spiTxBuffer[1] = register_index; // header of Register Index
   EPD_cs_low ();
   spiSendBuffer(2);
   EPD_cs_high ();

   Wait_10us ();

   spi_app_v.spiTxBuffer[0] = 0x72; // header of Register Data of write command
   memcpy(&spi_app_v.spiTxBuffer[1], register_data, length);
   EPD_cs_low ();
   spiSendBuffer(length+1);
   EPD_cs_high ();
}

/**
* \brief SPI command
*
* \param Register The Register Index as SPI Data to COG
* \param Data The Register Data for sending command data to COG
* \return the SPI read value
*/
// We do not read from the EPD so that we can drive MISO to prevent a sneak path through the EPD
// This is done because the SPI port can only be opened, and cannot be closed after a write
INT8U SPI_R(INT8U Register, INT8U Data) { 
  
   INT8U result;

   spi_app_v.spiTxBuffer[0] = 0x70; // header of Register Index
   spi_app_v.spiTxBuffer[1] = Register;
   EPD_cs_low ();
   spiSendBuffer(2);
   EPD_cs_high ();

   Wait_10us ();

   EPD_cs_low ();
   spiWriteReadByte(0x73); // header of Register Data of read command
   result = spiWriteReadByte(Data);
   EPD_cs_high ();

   return result;
}

/**
* \brief Configure GPIO
*/
void EPD_initialize_gpio(void) {
  // This is what we have available:
   // This is not hooked up on our board. -> config_gpio_dir_i( EPD_BUSY_PORT,EPD_BUSY_PIN);
   initGPIO(SSn, GPIO_STATE_OUTPUT);           // config_gpio_dir_o( EPD_CS_PORT,EPD_CS_PIN);
   initGPIO(RESETn, GPIO_STATE_OUTPUT);        // config_gpio_dir_o( EPD_RST_PORT,EPD_RST_PIN);
   initGPIO(PANEL_ON, GPIO_STATE_OUTPUT);      // config_gpio_dir_o( EPD_PANELON_PORT,EPD_PANELON_PIN);
   initGPIO(DISCHARGE, GPIO_STATE_OUTPUT);     // config_gpio_dir_o( EPD_DISCHARGE_PORT,EPD_DISCHARGE_PIN);
   // We have VCOM_PANEL, BORDER, VCOM_DRIVER, and BORDER_DRIVER tied instead of controlled. -> config_gpio_dir_o( EPD_BORDER_PORT,EPD_BORDER_PIN);
   // No PRM on our board -> config_gpio_dir_o( PWM_PORT,PWM_PIN);
   initGPIO(WRITE_EPD, GPIO_STATE_INPUT);     // EPD power switch indicator
   initGPIO(MISO, GPIO_STATE_INPUT);          // MISO is unused and can supply current to the EPD when not wanted
   EPD_border_low();
}

/**
 * \brief Configure SPI
 */
void epd_spi_init(void) {

   int                          err;
   dn_spi_open_args_t           spiOpenArgs;

  //===== initialize SPI
   // open the SPI device
   spiOpenArgs.maxTransactionLenForCPHA_1 = 0;
   err = dn_open(
      DN_SPI_DEV_ID,
      &spiOpenArgs,
      sizeof(spiOpenArgs)
   );
   ASSERT((err == DN_ERR_NONE) || (err == DN_ERR_STATE));

   // Initialize spi communication parameters
   spiTransfer.txData             = spi_app_v.spiTxBuffer;
   spiTransfer.rxData             = spi_app_v.spiRxBuffer;
   spiTransfer.numSamples         = 1;
   spiTransfer.startDelay         = 0;
   spiTransfer.clockPolarity      = DN_SPI_CPOL_0;
   spiTransfer.clockPhase         = DN_SPI_CPHA_0;
   spiTransfer.bitOrder           = DN_SPI_MSB_FIRST;
   spiTransfer.slaveSelect        = DN_SPIM_SS_2n;
   spiTransfer.clockDivider       = DN_SPI_CLKDIV_2;
   spi_app_v.spiTxBufferCounter   = 0;

}

/**
 * \brief Power on COG Driver
 * \note For detailed flow and description, please refer to the COG G1 document Section 3.
 */
void EPD_power_on(void) {
   
   // Deviation from driving document: we leave the display 'discharging' indefinitely between writes
   // So, here, we stop discharging to perform a new write
   EPD_discharge_low ();
  
   /* Initial state */
   EPD_Vcc_turn_on();
   epd_spi_attach();
   EPD_cs_high();
   EPD_border_high();
   EPD_rst_high();
   delay_ms(5);
   EPD_rst_low();
   delay_ms(5);
   EPD_rst_high();
   delay_ms(5);
   // Sense temperature to determine Temperature Factor
   set_temperature_factor(25);
}

/**
 * \brief Initialize COG Driver
 * \note For detailed flow and description, please refer to the COG G2 document Section 4.
 */
INT8U EPD_initialize_driver (void) {
   
   // Empty the Line buffer
   memset(COG_Line.uint8, 0x00, LINE_BUFFER_DATA_SIZE);
   
   // Determine the EPD size for driving COG
   data_line_even = &COG_Line.line_data_by_size.line_data_for_144.even[0];
   data_line_odd  = &COG_Line.line_data_by_size.line_data_for_144.odd[0];
   data_line_scan = &COG_Line.line_data_by_size.line_data_for_144.scan[0];
   data_line_border_byte = &COG_Line.line_data_by_size.line_data_for_144.border_byte;

   INT16U i = 0;

   while (EPD_IsBusy()) {
      if((i++) >= 0x0FFF) return ERROR_BUSY;
   }

   //Check COG ID
   #if SPI_READ_ENABLED == TRUE
   if((SPI_R(0x72,0x00) & 0x0f) != 0x02) { 
     return ERROR_COG_ID;
   }
   #endif

   //Disable OE
   epd_spi_send_byte(0x02,0x40);

   //Check Breakage
   #if SPI_READ_ENABLED == TRUE
   if((SPI_R(0x0F,0x00) & 0x80) != 0x80) return ERROR_BREAKAGE;
   #endif
   
   //Power Saving Mode
    epd_spi_send_byte(0x0B, 0x02);

   //Channel Select
   epd_spi_send(0x01, (INT8U *)&COG_parameters.channel_select, 8);

   //High Power Mode Osc Setting
   epd_spi_send_byte(0x07,0xD1);

   //Power Setting
   epd_spi_send_byte(0x08,0x02);

   //Set Vcom level
   epd_spi_send_byte(0x09,0xC2);

   //Power Setting
   epd_spi_send_byte(0x04,0x03);

   //Driver latch on
   epd_spi_send_byte(0x03,0x01);

   //Driver latch off
   epd_spi_send_byte(0x03,0x00);

   delay_ms(5);

   //Chargepump Start
   i=0;
   do {
      //Start chargepump positive V
      //VGH & VDH on
      epd_spi_send_byte(0x05,0x01);

      delay_ms(240);

      //Start chargepump neg voltage
      //VGL & VDL on
      epd_spi_send_byte(0x05,0x03);

      delay_ms(40);

      //Set chargepump
      //Vcom_Driver to ON
      //Vcom_Driver on
      epd_spi_send_byte(0x05,0x0F);

      delay_ms(40);

      //Check DC/DC 
      #if SPI_READ_ENABLED == TRUE
      if((SPI_R(0x0F,0x00) & 0x40) != 0x00)
      {
         //Output enable to disable
         epd_spi_send_byte(0x02,0x40);
         break;
      }
      #else
         epd_spi_send_byte(0x02,0x40);
         break;
      #endif

   }while((i++) != 4);
   
   if(i>=4) return ERROR_CHARGEPUMP;
   else return RES_OK;
}

/**
 * \brief Write image data from memory array to the EPD
 * \note
 * - There are 4 stages to complete an image update on EPD.
 * - The same stage time for Each of the 4 stages.
 * - For more detail on driving stages, please refer to COG G2 document Section 5.
 *
 * \param previous_image_ptr The pointer of memory that stores previous image
 * \param new_image_ptr The pointer of memory that stores new image
 */
void EPD_display_from_array_prt(INT8U *previous_image_ptr, INT8U *new_image_ptr, INT8U updateInStages) {
  
  if(updateInStages) {
      stage_handle_array(previous_image_ptr, Stage1);
      // Use quicker COG_Flash() function twice to replace this longer call -> stage_handle_array(previous_image_ptr, Stage2);
      COG_Flash(TRUE);
      OSTimeDly(10);
      COG_Flash(TRUE);
      OSTimeDly(10);
      stage_handle_array(new_image_ptr, Stage3);
  }
  stage_handle_array(new_image_ptr, Stage4);
}

/**
 * \brief The Global Update driving stages for getting Odd/Even data and writing the data
 * from memory array to COG
 *
 * \note
 * - One dot/pixel is comprised of 2 bits which are White(10), Black(11) or Nothing(01 or 00).
 *   The image data bytes must be divided into Odd and Even bytes.
 * - For 1.44", 2" and 2.7", the line data flow is half_Data-Scan-half_Data
 * - For more details on the driving stages, please refer to the COG G2 document Section 5.
 *
 * \param image_ptr The pointer of memory that stores image that will send to COG
 * \param stage_no The assigned stage number that will proceed
 */
void stage_handle_array(INT8U *image_prt, INT8U stage_no) {
  
   /* x for horizontal_size loop, y for vertical_size loop, which are EPD pixel size */
   INT16U x,y,k;
   INT16S scanline_no = 0;
   INT8U temp_byte; // Temporary storage for image data check
   INT8U *backup_image_prt; // Backup image address pointer
   INT8U byte_array[COG_line_Max_Size];
   INT32U start_EPD_timer = getSystemTime();    /* Start a system SysTick timer to ensure the same duration of each stage  */

   backup_image_prt = image_prt;

   /* Do while total time of frames exceed stage time
   * Per frame */
   do {
      image_prt = backup_image_prt;

      /* Per data line (vertical size) */
      for (y = 0; y < COG_parameters.vertical_size; y++) {
         k = COG_parameters.horizontal_size-1;
         for (x = 0; x < COG_parameters.horizontal_size; x++) {
            temp_byte = *image_prt++;
            
            switch(stage_no) {
               case Stage1: // Compensate, Inverse previous image
               data_line_odd[x]     = ((temp_byte & 0x40) ? BLACK3  : WHITE3);
               data_line_odd[x]    |= ((temp_byte & 0x10) ? BLACK2  : WHITE2);
               data_line_odd[x]    |= ((temp_byte & 0x04) ? BLACK1  : WHITE1);
               data_line_odd[x]    |= ((temp_byte & 0x01) ? BLACK0  : WHITE0);

               data_line_even[k]    = ((temp_byte & 0x80) ? BLACK0  : WHITE0);
               data_line_even[k]   |= ((temp_byte & 0x20) ? BLACK1  : WHITE1);
               data_line_even[k]   |= ((temp_byte & 0x08) ? BLACK2  : WHITE2);
               data_line_even[k--] |= ((temp_byte & 0x02) ? BLACK3  : WHITE3);
                  break;
               case Stage2: // White
               data_line_odd[x]     = ((temp_byte & 0x40) ?  WHITE3 : NOTHING3);
               data_line_odd[x]    |= ((temp_byte & 0x10) ?  WHITE2 : NOTHING2);
               data_line_odd[x]    |= ((temp_byte & 0x04) ?  WHITE1 : NOTHING1);
               data_line_odd[x]    |= ((temp_byte & 0x01) ?  WHITE0 : NOTHING0);

               data_line_even[k]    = ((temp_byte & 0x80) ?  WHITE0 : NOTHING0);
               data_line_even[k]   |= ((temp_byte & 0x20) ?  WHITE1 : NOTHING1);
               data_line_even[k]   |= ((temp_byte & 0x08) ?  WHITE2 : NOTHING2);
               data_line_even[k--] |= ((temp_byte & 0x02) ?  WHITE3 : NOTHING3);
                  break;
               case Stage3: // Inverse new image
               data_line_odd[x]     = ((temp_byte & 0x40) ? BLACK3  : NOTHING3);
               data_line_odd[x]    |= ((temp_byte & 0x10) ? BLACK2  : NOTHING2);
               data_line_odd[x]    |= ((temp_byte & 0x04) ? BLACK1  : NOTHING1);
               data_line_odd[x]    |= ((temp_byte & 0x01) ? BLACK0  : NOTHING0);

               data_line_even[k]    = ((temp_byte & 0x80) ? BLACK0  : NOTHING0);
               data_line_even[k]   |= ((temp_byte & 0x20) ? BLACK1  : NOTHING1);
               data_line_even[k]   |= ((temp_byte & 0x08) ? BLACK2  : NOTHING2);
               data_line_even[k--] |= ((temp_byte & 0x02) ? BLACK3  : NOTHING3);
                  break;
               case Stage4: // New image
               data_line_odd[x]     = ((temp_byte & 0x40) ? WHITE3  : BLACK3 );
               data_line_odd[x]    |= ((temp_byte & 0x10) ? WHITE2  : BLACK2 );
               data_line_odd[x]    |= ((temp_byte & 0x04) ? WHITE1  : BLACK1 );
               data_line_odd[x]    |= ((temp_byte & 0x01) ? WHITE0  : BLACK0 );

               data_line_even[k]    = ((temp_byte & 0x80) ? WHITE0  : BLACK0 );
               data_line_even[k]   |= ((temp_byte & 0x20) ? WHITE1  : BLACK1 );
               data_line_even[k]   |= ((temp_byte & 0x08) ? WHITE2  : BLACK2 );
               data_line_even[k--] |= ((temp_byte & 0x02) ? WHITE3  : BLACK3 );
                  break;
            }

         }

         scanline_no = (COG_parameters.vertical_size-1) - y;

         /* Scan byte shift per data line */
         data_line_scan[(scanline_no>>2)] = SCAN_TABLE[(scanline_no%4)];

         /* Sending data */
         epd_spi_send (0x0A, (INT8U *)&COG_Line.uint8, COG_parameters.data_line_size);

         /* Turn on Output Enable */
         epd_spi_send_byte (0x02, 0x07);

         data_line_scan[(scanline_no>>2)]=0;
      }
                  
      /* Count the frame time with offset */
      current_frame_time = (getSystemTime() - start_EPD_timer) * MS_PER_SEC / TIMER_TICKS_PER_SEC;
      
   } while (stage_time > current_frame_time);
   
   return;
}

/**
 * \brief Power Off COG Driver
 * \note For detailed flow and description, please refer to the COG G2 document Section 6.
 */
INT8U EPD_power_off (void) {
   
   border_dummy_line();
   
   // Check DC/DC
   #if SPI_READ_ENABLED == TRUE
   if((SPI_R(0x0F,0x00) & 0x40) == 0x00) return ERROR_DC;
   #endif
   
   // Turn on Latch Reset
   epd_spi_send_byte (0x03, 0x01);
   // Turn off OE
   epd_spi_send_byte (0x02, 0x05);
   // Power off charge pump Vcom
   epd_spi_send_byte (0x05, 0x03);
   // Power off charge pump neg voltage
   epd_spi_send_byte (0x05, 0x01);
   delay_ms(120);
   // Discharge internal SPI
   epd_spi_send_byte (0x04, 0x80);
   // Turn off all charge pump
   epd_spi_send_byte (0x05, 0x00);
   // Turn off OSC
   epd_spi_send_byte (0x07, 0x01);
   delay_ms(50);

   epd_spi_detach ();
   EPD_Vcc_turn_off ();
   EPD_border_low();
   delay_ms (10);

   EPD_cs_low();
   EPD_rst_low();
   
   delay_ms (250); // Not mentioned in driving document, but this helps 'washing out' of the display
   
   EPD_discharge_high ();
   
   // Deviation from driving document: we leave the display 'discharging' indefinitely between writes
   // This consumes no extra power, saves delay time, and ensures that it is fully discharged
   //delay_ms (150);
   //EPD_discharge_low ();
   
   configGPIO(MISO, GPIO_STATE_INPUT); //set MISO to an input so that it does not supply power to the EPD when idle

   return RES_OK;
}

// Re-start the SPI at the beginning of the display write
// Since SPI cannot be closed and reopened, we instead leave it open and drive MISO high during the write
// Driving MISO high ensures that charge pump power is not drained by the mote during a write
void epd_spi_attach(void) {
  
   EPD_cs_high();
   
   configGPIO(MISO, GPIO_STATE_OUTPUT); //set MISO to an output
   digitalWrite(MISO, GPIO_OUTPUT_HIGH); //set MISO high so that it doesn't take power from the EPD during the write
}

// Dummy function to detach SPI
// This does nothing since we cannot reopen the SPI after closing it
void epd_spi_detach(void) {
   
   SPIMISO_low();
   SPIMOSI_low();
   SPICLK_low();
}

// Colors the whole screen white or black in one "output enable"
void COG_Flash(INT8U white) {

   INT8U border_byte_old = *data_line_border_byte; // Save the old border byte.

   // Pack and send the Odd Data Bytes, Scan Bytes, and Even Data Bytes together.
   memset(data_line_even, white ? (WHITE0 | WHITE1 | WHITE2 | WHITE3) : (BLACK0 | BLACK1 | BLACK2 | BLACK3), COG_NUM_DOTS_PACKED);
   memset(data_line_scan, 0xFF, COG_NUM_SCAN_BYTES);
   memset(data_line_odd , white ? (WHITE0 | WHITE1 | WHITE2 | WHITE3) : (BLACK0 | BLACK1 | BLACK2 | BLACK3), COG_NUM_DOTS_PACKED);
   *data_line_border_byte = 0x00;

   // Send data
   epd_spi_send (0x0A, (INT8U *)&COG_Line.uint8, COG_parameters.data_line_size);
   
   // Output from COG driver to panel
   epd_spi_send_byte(0x02, 0x2F); 

   // Return scane lines and border byte to value before function was run.
   *data_line_border_byte = border_byte_old;
   memset(data_line_scan, 0x00, COG_NUM_SCAN_BYTES);
}

/**
 * \brief According to EPD size and temperature to get stage_time
 * \note Refer to COG G2 document Section 5.3 for more details
 */
static void set_temperature_factor(INT8S temperature) {
  
   if (temperature <= -10) {
      stage_time = temperature_table[0];
      } else if (-5 >= temperature && temperature > -10) {
      stage_time = temperature_table[1];
      } else if (5 >= temperature && temperature > -5) {
      stage_time = temperature_table[2];
      } else if (10 >= temperature && temperature > 5) {
      stage_time = temperature_table[3];
      } else if (15 >= temperature && temperature > 10) {
      stage_time = temperature_table[4];
      } else if (20 >= temperature && temperature > 15) {
      stage_time = temperature_table[5];
      } else if (40 >= temperature && temperature > 20) {
      stage_time = temperature_table[6];
   } else stage_time = temperature_table[7];
}

/**
 * \brief BORDER Dummy Line for 1.44",1.9", 2" or 2.6"
 */
void border_dummy_line(void) {
  
   memset(COG_Line.uint8, 0x00, COG_parameters.data_line_size);

   *data_line_border_byte = BORDER_BYTE_B;
   //Write a Border(B) Dummy Line
   epd_spi_send (0x0a, (INT8U*)&COG_Line.uint8, COG_parameters.data_line_size);
   //Turn on OE
   epd_spi_send_byte (0x02, 0x07);

   sys_delay_ms(40);

   *data_line_border_byte = BORDER_BYTE_W;
   //Write a Borde(B) Dummy Line
   epd_spi_send (0x0a, (INT8U*)&COG_Line.uint8, COG_parameters.data_line_size);
   //Turn on OE
   epd_spi_send_byte (0x02, 0x07);

   sys_delay_ms(200);
}

// The delay used for display driving functions
// If LOW_POWER_DELAY is turned on, all delays are done with an OSTimeDly(), which uses less energy
// Otherwise, a loop is entered to ensure accurate delays, which results in a slightly faster update
void delay_ms(INT16U ms) {

   #if LOW_POWER_DELAY == TRUE //using an OSTimeDly() takes a little less energy than an accurate delay
      OSTimeDly(ms);
   #else //monitoring the system time ensures accurate (faster) delays, but takes a little emore energy
      INT32U start_time = getSystemTime();
      INT32U new_time;

      do {
       new_time = getSystemTime();
      } while (((new_time - start_time) * MS_PER_SEC) < (ms * TIMER_TICKS_PER_SEC));
   #endif
}
