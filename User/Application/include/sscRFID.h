#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>

void SscSplitTypeAutoModeGetData_Init(void);

void SendKeyRFIDMessageAup(uint8_t rfid_data);
void SendKeyRFIDMessageAdown(void);
void SendKeyRFIDMessageBup(uint8_t rfid_data);
void SendKeyRFIDMessageBdown(void);
void SscRadioFreq_Init(void);
void RfidHandle(uint8_t * uartx_rf_buff, uint8_t interface, bool enable_rfid, uint8_t rfid_type);
