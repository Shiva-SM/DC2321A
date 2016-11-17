#ifndef COULOMB_COUNTER_H
#define COULOMB_COUNTER_H

//=========================== enums  ==========================================
typedef enum {PRI, SEC, NUMCC} coulomb_counter_source_t;       // index for the primary or secondary CC
typedef enum {CHG, VLT, CUR, NUMDATA} coulomb_counter_data_t;  // index for the type of CC data

//=========================== defines =========================================
#define I2C_PAYLOAD_LENGTH    5

//=========================== prototypes ======================================
void coulombCounterInit();

void coulombCounterInit2();
dn_error_t I2CWrite(INT8U* dataPtr, INT8U address, INT8U numBytes);
dn_error_t I2CWriteAndRead(INT8U address, INT8U* readCommand, INT8U numBytesSent, INT8U* readBuffer, INT8U numBytesReceived);
void setErrFlag(coulomb_counter_source_t selectedCC, dn_error_t dnErr);

void initCCs();
void clearCCs();
void updateAllCCData();
void toggleCCAlertInterrupt(INT8U enabled);

// Returns the data from one LTC2942 in uC, uV, or nA
INT32S getCCData(coulomb_counter_source_t CC,   // selects which LTC2942 from which to get data
                 coulomb_counter_data_t data);  // which data to retrieve.

//=========================== variables =======================================
extern INT8U ccClearedGuiFlag;

#endif