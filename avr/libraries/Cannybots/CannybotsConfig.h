//
//  CannybotsConfig.h
//
//  Created by Wayne Keenan on 01/08/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#ifndef CannybotsConfig_h
#define CannybotsConfig_h

#define DEBUG 1

// Pinouts...
#define INFO_LED 13


////////////////////////////////////////////////////////////////////////
// BLE advertising, (when compiled for and run on RFduino)

#define BLE_UUID                 "7e400001-b5a3-f393-e0a9-e50e24dcca9e"

#ifndef BLE_LOCALNAME
#define BLE_LOCALNAME            "Cannybots"
#endif

#ifndef BLE_ADVERTISEMENT_DATA
#define BLE_ADVERTISEMENT_DATA   "CB_DEF_001"
#endif
// Note: BLE_ADVERTISEMENT_DATA must be < 16 bytes.

#define BLE_TX_POWER_LEVEL  4



////////////////////////////////////////////////////////////////////////
// Message payload offsets
#define CB_MAX_MSG_SIZE 20

#define CB_MSG_OFFSET_CMD  4
#define CB_MSG_OFFSET_DATA 6

#define CB_MAX_MSG_DATA_SIZE (CB_MAX_MSG_SIZE-CB_MSG_OFFSET_DATA)

// Exchanged variables info
#define CB_MAX_SYS_DESCRIPTORS  2
#define CB_MAX_DESCRIPTORS 255

/// used to determin the tye of the command
#define CB_MIN_CMD_METHOD_TYPE 0
#define CB_MAX_CMD_METHOD_TYPE 63
#define CB_MIN_CMD_CONFIG_TYPE 64
#define CB_MAX_CMD_CONFIG_TYPE 127
#define CB_MIN_CMD_VARIABLE_TYPE 128
#define CB_MAX_CMD_VARIABLE_TYPE 247
// 8 unused

#define CB_CMD_IS_METHOD(c) (c<CB_MAX_CMD_METHOD_TYPE)


////////////////////////////////////////////////////////////////////////
// Message Queuing

// Transport buffers
#define SERIAL_BUF_SIZE 32
#define CB_MAX_OUT_Q_DEPTH 8
#define CB_MAX_IN_Q_DEPTH 4

// Connection settings
#ifdef __RFduino__
// this is really just for debugging, the message will come from BLE_onReceive
#define CB_INBOUND_SERIAL_PORT Serial
#else
// Assume A* , atmega32u4 or other AVR with Serial1
#define CB_INBOUND_SERIAL_PORT Serial1
#endif

#define CB_INBOUND_SERIAL_BAUD 9600





#endif
