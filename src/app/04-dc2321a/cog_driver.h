#ifndef COG_DRIVER_H
#define COG_DRIVER_H

#include "dn_typedef.h"

//=========================== defines =========================================
// Dimensions of EPD from section 5 of COG datasheet.
#define COG_NUM_LINES               96U                     // COG displays 96 pixels high
#define COG_NUM_DOTS                128U                    // COG displays 128 pixels long
#define COG_NUM_DOTS_PACKED         (INT8U)(COG_NUM_DOTS/8) // COG displays 128 pixels long, packed into bytes

// A block of memory defining COG one screen.
typedef INT8U COG_DRIVER_IMAGE[COG_NUM_LINES][COG_NUM_DOTS_PACKED];

/**
 * \brief Four driving stages */
enum Stage {
   Stage1, /**< Inverse previous image */
   Stage2, /**< White */
   Stage3, /**< Inverse new image */
   Stage4, /**< New image */
   NumStages
};

//=========================== prototypes ==================================
void EPD_display_hardware_init(void);
void EPD_display_from_pointer(INT8U *previous_image_ptr,INT8U *new_image_ptr, INT8U flash);

//functions for writing to display (public for updating in a custom manner)
void EPD_power_on(void);
void stage_handle_array(INT8U *image_prt, INT8U stage_no);
INT8U EPD_initialize_driver(void);
INT8U EPD_power_off(void);
void COG_Flash(INT8U white);

#endif