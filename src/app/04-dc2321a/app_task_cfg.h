#ifndef APP_TASK_CFG_H
#define APP_TASK_CFG_H

// LOWER NUMBER = HIGHER PRIORITY

//MAIN
#define TASK_APP_MAINLOOP_NAME            "DC2321A"
#define TASK_APP_MAINLOOP_PRIORITY        53
#define TASK_APP_MAINLOOP_STK_SIZE        256

//BUTTON
#define TASK_APP_GPIONOTIF_NAME           "gpioNotif"
#define TASK_APP_GPIONOTIF_PRIORITY       54
#define TASK_APP_GPIONOTIF_STK_SIZE       64

//COULOMB COUNTER ALERT
#define TASK_APP_CCALERT_NAME             "ccAlert"
#define TASK_APP_CCALERT_PRIORITY         55
#define TASK_APP_CCALERT_STK_SIZE         128

//UART TX
#define TASK_APP_UART_TX_NAME             "uartTx"
#define TASK_APP_UART_TX_PRIORITY         56
#define TASK_APP_UART_TX_STK_SIZE         128

//UART RX
#define TASK_APP_UART_RX_NAME             "uartRx"
#define TASK_APP_UART_RX_PRIORITY         57
#define TASK_APP_UART_RX_STK_SIZE         832

//NETWORK JOIN
#define TASK_APP_SEND_NAME                "send"
#define TASK_APP_SEND_PRIORITY            58
#define TASK_APP_SEND_STK_SIZE            256

//DISPLAY UPDATE
#define TASK_APP_DISPLAY_UPDATE_NAME      "displayUpdate"
#define TASK_APP_DISPLAY_UPDATE_PRIORITY  59
#define TASK_APP_DISPLAY_UPDATE_STK_SIZE  832

#endif