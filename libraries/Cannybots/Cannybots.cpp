
#include "Cannybots.h"

#include <NTUtils.h>
#include <NTProtocol.h>
#include <NTMessaging.h>



// class: CAnnybots
// define statics
const cb_publish_type Cannybots::PUBLISH_UPDATE_ONCHANGE = 1;

// 20 bytes
// msg[0]     = 'C'
// msg[1]     = 'B'
// msg[2]     = 1 byte CRC
// msg[3]     = seq[7..4] .. type[3...0]
// msg[4]     = cmd < 192  (>192 is sys command)
// msg[5]     = param count (MAYBE TODO: > max len implies mulit packet)
// msg[6-7]   = int16
// msg[8-9]   = int16
// msg[10-11] = int16
// msg[12-13] = int16
// msg[14-15] = int16
// msg[16-17] = int16
// msg[18-19] = int16

// type:
/*
 byte/char  = 1 byte
 uint/int   = 2 bytes
 ulong/long = 4 bytes
 float      = 4 bytes
 
 0  void
 1  scalar: byte
 2  scalar: int
 3  scalar: uint
 4  scalar: long
 5  scalar: ulong
 6  scalar: float
 7  array:  byte[]               - arbitrary data, e.g. custom binary protocol such as EasyTransfer
 8 string (0 terminated)
 9 int16[7]                 // short
 10 uint16[7]                // word
 11 long32[3]
 12 ulong32[3]
 13 float32[3]
 14 int16,int16,int16
 15 float,float,float
 
 */

const cb_type Cannybots::CB_VOID=0;
const cb_type Cannybots::CB_BYTE=1;
const cb_type Cannybots::CB_INT16=2;
const cb_type Cannybots::CB_UINT16=3;
const cb_type Cannybots::CB_INT32=4;
const cb_type Cannybots::CB_UINT32=5;
const cb_type Cannybots::CB_FLOAT=6;
const cb_type Cannybots::CB_BYTE_ARRAY=7;
const cb_type Cannybots::CB_STRING=8;
const cb_type Cannybots::CB_INT16_ARRAY=9;
const cb_type Cannybots::CB_UINT16_ARRAY=10;
const cb_type Cannybots::CB_INT32_ARRAY=11;
const cb_type Cannybots::CB_UINT32_ARRAY=12;
const cb_type Cannybots::CB_FLOAT_ARRAY=13;
const cb_type Cannybots::CB_INT16_3=14;
const cb_type Cannybots::CB_FLOAT_3=15;


Cannybots Cannybots::instance;

void Cannybots::setConfigStorage(const char* magic, uint16_t start) {
    
}

// Scalar
void Cannybots::registerVariable(const cb_id& _id, int16_t* var, const bool isNonVolatile, const int16_t defaultValue) {
    
    uint16_t offset =_id.cid;
    descriptors[offset].cid = _id;
    descriptors[offset].data = var;
    descriptors[offset].size = sizeof(int16_t*);
    descriptors[offset].isMethod = false;
    descriptors[offset].isPublished = false;
    descriptors[offset].isNV = isNonVolatile;
    descriptors[offset].type = CB_INT16;
}

void Cannybots::registerVariable(const cb_id& _id, bool*    var, const bool isNonVolatile, const bool    defaultValue) {
    uint16_t offset =_id.cid;
    descriptors[offset].cid = _id;
    descriptors[offset].data = var;
    descriptors[offset].size = sizeof(bool*);
    descriptors[offset].isPublished = false;
    descriptors[offset].isMethod = false;
    descriptors[offset].isNV = isNonVolatile;
    descriptors[offset].type = CB_BYTE; // BOOL really
}

// Arrays
void Cannybots::registerArray(const cb_id _id, int16_t list[], const uint16_t length) {
    uint16_t offset =_id.cid;
    descriptors[offset].cid = _id;
    descriptors[offset].data = list;
    descriptors[offset].size = length * sizeof(uint16_t);
    descriptors[offset].isPublished = false;
    descriptors[offset].isMethod = false;
    descriptors[offset].isNV = false;
    descriptors[offset].type = CB_INT16_ARRAY;
}

// Published values
void Cannybots::registerPublisher(const cb_id _id, bool *var, const cb_publish_type pubType) {
    uint16_t offset =_id.cid;
    descriptors[offset].isPublished = true;
    // TODO: add to watch list
}


// Function handlers
void Cannybots::registerHandler(const cb_id _id, cb_callback_int16_int16_int16 callback) {
    uint16_t offset =_id.cid;
    descriptors[offset].cid = _id;
    descriptors[offset].data = (void*)callback;
    descriptors[offset].size = 0;
    descriptors[offset].isMethod = true;
    descriptors[offset].type = CB_INT16_3;

    descriptors[offset].isPublished = false;
    descriptors[offset].isNV = false;
}

// abritrary (e..g iOS blcok handlers0
void Cannybots::registerHandler(const cb_id _id, uint8_t type, void* callback) {
    uint16_t offset =_id.cid;
    descriptors[offset].cid = _id;
    descriptors[offset].data = (void*)callback;
    descriptors[offset].size = 0;
    descriptors[offset].isMethod = true;
    descriptors[offset].type = type;
    
    descriptors[offset].isPublished = false;
    descriptors[offset].isNV = false;
}

// Scripting

void Cannybots::registerScritableVariable(const cb_id _id, const char* name) {
    
}
#ifdef ARDUINO
void Cannybots::registerScritableVariable(const cb_id _id, const __FlashStringHelper* name) {
    Cannybots::registerScritableVariable(_id, (const char *) name);
}
#endif

// GUI


// utils

void Cannybots::gui_addButton(const int16_t x, const int16_t y, const char* buttonText,  cb_callback_gui callback) {
    
}

void Cannybots::gui_addLevelMeter(const int16_t x, const int16_t y, const char* label,  int16_t* variable, const int16_t min, const int16_t max) {
    
}

bool Cannybots::validate() {
    return true;
}

uint16_t Cannybots::getLastError() {
    return 0;
}

void Cannybots::begin() {
#ifdef ARDUINO

    CB_INBOUND_SERIAL_PORT.begin(CB_INBOUND_SERIAL_BAUD);
    
#ifdef ARDUINO_AVR_A_STAR_32U4
    // brownout detection
    // from:  http://www.pololu.com/docs/0J61/7
    pinMode(13, OUTPUT);
    if (MCUSR & (1 << BORF))
    {
        // A brownout reset occurred.  Blink the LED
        // quickly for 2 seconds.
        for (uint8_t i = 0; i < 5; i++)
        {
            digitalWrite(13, HIGH);
            delay(100);
            digitalWrite(13, LOW);
            delay(100);
        }
    }
    MCUSR = 0;
#endif
#endif

}

void Cannybots::update() {
#ifdef ARDUINO
    readSerial(Serial1);
#endif
    processOutboundMessageQueue();
    processInboundMessageQueue();
}


/////////////////////////////////////////////////////////////////////
// comms

#ifdef ARDUINO
void Cannybots::readSerial(HardwareSerial &ser) {
    while (ser.available()>0) {
        lastChar = c;
        c =  ser.read();
        Serial.print(" ");
        Serial.print(c, HEX);
        
        if (foundStart && (serialBufPtr<SERIAL_BUF_SIZE)) {
            serialBuffer[serialBufPtr++] = c;
        } else if ( ('>' == c) && ('>' == lastChar) ) {
            foundStart=true;
            serialBufPtr=0;
        }
        
        if (serialBufPtr>=CB_MAX_MSG_SIZE) {
            Serial.println("<EOC>");
            foundStart = false;
            Message* msg = new Message(serialBuffer, CB_MAX_MSG_SIZE);
            inboundMsgFIFO.enqueue(msg);
            
            serialBufPtr=0;
        }
    }
}
#endif



void Cannybots::processInboundMessageQueue() {
    
    for (int i = 0; i < inboundMsgFIFO.count(); i++) {
        //CB_DBG("processInboundMessageQueue(%d)", i+1);

        Message* msg = inboundMsgFIFO.dequeue();
        processMessage(msg);
        delete msg;
    }
}


const char* Cannybots::getDeviceId() {
    return NULL;
// iOS
//
//    char  *currentDeviceId = [[[[UIDevice currentDevice] identifierForVendor]UUIDString] UTF8String];
    
    // RFduino
    //uint64_t id = getDeviceId();
    //Serial.println(getDeviceIdLow(), HEX);
    //Serial.println(getDeviceIdHigh(), HEX);
}

void Cannybots::processOutboundMessageQueue() {

    for (int i = 0; i < outboundMsgFIFO.count(); i++) {
        CB_DBG("processOutboundMessageQueue(%d)", i+1);
        Message* msg = outboundMsgFIFO.dequeue();
#ifdef ARDUINO
#if defined(ARDUINO_AVR_LEONARDO)  || defined(ARDUINO_AVR_A_STAR_32U4)
        Serial1.write(">>");
        Serial1.write(msg->payload, CB_MAX_MSG_SIZE);
#else
        Serial.write(">>");
        Serial.write(msg->payload, CB_MAX_MSG_SIZE);
#endif // ARDUINO_AVR_LEONARDO
#endif // ARDUINO
        
        delete msg;
    }
}

// Command processing



void Cannybots::processMessage(Message* msg ) {
    //CB_DBG("processMessage()",0);
	
    uint8_t cmd = msg->payload[CB_MSG_OFFSET_CMD];
    //CB_DBG("cmd= %d", cmd);
    cb_descriptor desc = descriptors[cmd];
    
    if (desc.isMethod) {
        //CB_DBG("isMethod",0);

        if (desc.type == CB_INT16_3) {
            //CB_DBG("is CB_INT16_3",0);
            ((cb_callback_int16_int16_int16)desc.data)( mk16bit( msg->payload[CB_MSG_OFFSET_DATA+1],msg->payload[CB_MSG_OFFSET_DATA+0]),
                                                        mk16bit( msg->payload[CB_MSG_OFFSET_DATA+3],msg->payload[CB_MSG_OFFSET_DATA+2]),
                                                        mk16bit( msg->payload[CB_MSG_OFFSET_DATA+5],msg->payload[CB_MSG_OFFSET_DATA+4]));
        }
    }
}

int16_t Cannybots::getFreeMemory() {
#ifdef ARDUINO
    // see: http://playground.arduino.cc/Code/AvailableMemory
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
#else
    // see: http://stackoverflow.com/questions/5012886/determining-the-available-amount-of-ram-on-an-ios-device
    mach_port_t host_port;
    mach_msg_type_number_t host_size;
    vm_size_t pagesize;
    
    host_port = mach_host_self();
    host_size = sizeof(vm_statistics_data_t) / sizeof(integer_t);
    host_page_size(host_port, &pagesize);
    
    vm_statistics_data_t vm_stat;
    
    if (host_statistics(host_port, HOST_VM_INFO, (host_info_t)&vm_stat, &host_size) != KERN_SUCCESS) {
        return 0;
        //NSLog(@"Failed to fetch vm statistics");
    }
    
    /* Stats in bytes */
    //natural_t mem_used = (vm_stat.active_count +
    //                      vm_stat.inactive_count +
    //                      vm_stat.wire_count) * pagesize;
    natural_t mem_free = vm_stat.free_count * pagesize;
    //natural_t mem_total = mem_used + mem_free;
    return mem_free;
#endif
}


