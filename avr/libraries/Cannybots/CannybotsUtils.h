//
//  CannybotsUtils.h
//
//  Created by Wayne Keenan
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#ifndef CannybotsUtils_h
#define CannybotsUtils_h

// helpers
#define hiByteFromInt(x)  (uint8_t)((x &0xff00) >>8)
#define loByteFromInt(x)  (uint8_t)(x & 0xff)
#define mk16bit(lo,hi) ( (lo&0xFF) + ((hi&0xFF)<<8))



#ifdef DEBUG
#ifdef ARDUINO
static char dbg_buffer[128];
#define CB_DBG(FMT, ...) snprintf(dbg_buffer, 128, FMT, __VA_ARGS__); Serial.println(dbg_buffer); Serial.flush(); dbg_buffer[/*CB_MAX_MSG_SIZE-CB_MSG_OFFSET_DATA*/ 5]=0; Cannybots::getInstance().callMethod(_CB_SYS_LOG, dbg_buffer);
#else
static char dbg_buffer[256];
#define CB_DBG(FMT, ...) printf("!!!implement iOS logging hook\n"); //printf(FMT, __VA_ARGS__);
#endif //ARDUINO
#else
#define LOG(...)
#define CB_DBG(...)
#endif // DEBUG


#ifdef __RFduino__
#define WATCHDOG_SETUP(seconds) NRF_WDT->CRV = 32768 * seconds; NRF_WDT->TASKS_START = 1;
#define WATCHDOG_RELOAD() NRF_WDT->RR[0] = WDT_RR_RR_Reload;
#define BLE_BEGIN_CRITICAL_SECTION() while (!RFduinoBLE.radioActive); while (RFduinoBLE.radioActive);
#define BLE_END_CRITICAL_SECTION()
#else
#define WATCHDOG_SETUP(seconds)
#define WATCHDOG_RELOAD()
#define BLE_BEGIN_CRITICAL_SECTION()
#define BLE_END_CRITICAL_SECTION()
#endif

#endif
