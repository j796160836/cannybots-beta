#include "Cannybots.h"

//#define USE_SPI

#ifdef __RFduino__
#include <RFduinoBLE.h>
#endif

#ifdef ARDUINO
#include <Arduino.h>
#include <EEPROM.h>
#ifdef USE_SPI
#include <SPI.h>
#endif USE_SPI
#endif //ARDUINO


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
const cb_type Cannybots::CB_INT16_1=14;
const cb_type Cannybots::CB_INT16_2=15;
const cb_type Cannybots::CB_INT16_3=16;
const cb_type Cannybots::CB_FLOAT_1=17;
const cb_type Cannybots::CB_FLOAT_2=18;
const cb_type Cannybots::CB_FLOAT_3=19;


const cb_publish_type Cannybots::PUBLISH_UPDATE_ONCHANGE = 1;


Cannybots Cannybots::instance;

void Cannybots::setConfigStorage(const char* magic, uint16_t start) {
    
}

#define CB_ALLOC_DESC()  (cb_descriptor*) calloc(1, sizeof(cb_descriptor));
//new cb_descriptor;
//


// Scalar
void Cannybots::registerVariable(cb_id* _id, int16_t* var, const bool isNonVolatile, const int16_t defaultValue) {
    
    cb_descriptor* desc =  CB_ALLOC_DESC();
    desc->cid_t.cidMT = _id;
    desc->data = var;
    desc->type = CB_INT16;
    methods.add(desc);
}

void Cannybots::registerVariable(cb_id* _id, bool*    var, const bool isNonVolatile, const bool    defaultValue) {
    cb_descriptor* desc =  CB_ALLOC_DESC();
    desc->cid_t.cidMT = _id;
    desc->data = var;
    desc->type = CB_BYTE;
    methods.add(desc);
}

// Arrays
void Cannybots::registerArray(cb_id* _id, int16_t list[], const uint16_t length) {
    cb_descriptor* desc =  CB_ALLOC_DESC();
    desc->cid_t.cidMT = _id;
    desc->data = list;
    desc->type = CB_INT16_ARRAY;
    methods.add(desc);
}

// Published values
void Cannybots::registerPublisher(cb_id* _id, bool *var, const cb_publish_type pubType) {
    //uint16_t offset =_id.cid;
    //descriptors[offset].isPublished = true;
    // TODO: add to watch list
}


// Function handlers
void Cannybots::registerHandler(cb_id* _id, cb_callback_int16_int16_int16 callback) {
    cb_descriptor* desc =  CB_ALLOC_DESC();
    desc->cid_t.cidMT = _id;
    desc->data = (void*)callback;
    desc->type = CB_INT16_3;
    methods.add(desc);

}

// Function handlers
void Cannybots::registerHandler(cb_id* _id, cb_callback_int16_int16 callback) {
    cb_descriptor* desc =  CB_ALLOC_DESC();
    desc->cid_t.cidMT = _id;
    desc->data = (void*)callback;
    desc->type = CB_INT16_2;
    methods.add(desc);

}

void Cannybots::registerHandler(cb_id* _id, cb_callback_int16 callback) {
    cb_descriptor* desc =  CB_ALLOC_DESC();
    desc->cid_t.cidMT = _id;
    desc->data = (void*)callback;
    desc->type = CB_INT16_1;
    methods.add(desc);

}

void Cannybots::registerHandler(cb_id* _id, cb_callback_string callback) {
    cb_descriptor* desc =  CB_ALLOC_DESC();
    desc->cid_t.cidMT = _id;
    desc->data = (void*)callback;
    desc->type = CB_STRING;
    methods.add(desc);

}

// abritrary client registration (e.g. used fore registering the iOS 'block' handlers)
void Cannybots::registerHandler( cb_id* _id, uint8_t type, void* callback) {
    cb_descriptor* desc =  CB_ALLOC_DESC();
    desc->cid_t.cidMT = _id;
    desc->data = (void*)callback;
    desc->type = type;
    methods.add(desc);

}


void Cannybots::deregisterHandler( cb_id* _id) {

    cb_descriptor* desc = getDescriptorForCommand(_id->cid);
    if (desc) {
        int16_t i = getIndexForDescriptor(methods, desc);
        if (i!=-1) {
            methods.remove(i);
            //free(desc);
        }
    }
}


cb_descriptor* Cannybots::getDescriptorForCommand(uint8_t commandId) {
    cb_descriptor *desc=NULL;
    // find mathicg methods
    // TODO: optimise by searching the 'bands' defined in config.h:  CB_[MIN|MAX]_CMD_[METHOD|CONFIG|VARIABLE]_TYPE
    for(int i = 0; i < methods.size(); i++){
        desc = methods.get(i);
        printf("%d, %x, %x, %x, %d, %s\n", i, desc, desc->cid_t, desc->cid_t.cidMT, desc->cid_t.cidMT->cid, desc->cid_t.cidMT->name);
        if(commandId == desc->cid_t.cidMT->cid){
            return desc;
        }
    }
    return NULL;
}

int16_t Cannybots::getIndexForDescriptor(LinkedList<cb_descriptor*> list, cb_descriptor* desc) {
    for(int16_t i = 0; i < list.size(); i++){
        if (desc == methods.get(i)) {
            return i;
        }
    }
    return -1;
}




// Config

void Cannybots::registerConfigParameter(cb_nv_id* _id) {
    cb_descriptor* desc =  CB_ALLOC_DESC();
    desc->cid_t.cidNV = _id;
    configVars.add(desc);
}


//metadata:

void getConfigParameterList() {
    

}

/*
//getter/setters
setConfigParameterValue {
// if not on ARDUION build messag eand send
// if on arduon use EEPROMex library
    
}
getConfigParameterValue
*/


// Scripting

void Cannybots::registerScritableVariable(cb_id* _id, const char* name) {
    
}
#ifdef ARDUINO
void Cannybots::registerScritableVariable( cb_id* _id, const __FlashStringHelper* name) {
    Cannybots::registerScritableVariable(_id, (const char *) name);
}
#endif

// GUI utils

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
    
#ifdef ARDUINO_AVR_A_STAR_32U4
    // this depends on the Pololu bootloader being present on  the device for brownout detection
    // from:  http://www.pololu.com/docs/0J61/7
    pinMode(13, OUTPUT);
    if (MCUSR & (1 << BORF))
    {
        // A brownout reset occurred.  Blink the LED
        for (uint8_t i = 0; i < 5; i++)
        {
            digitalWrite(13, HIGH);
            delay(100);
            digitalWrite(13, LOW);
            delay(100);
        }
    }
    MCUSR = 0;
#endif // ARDUINO_AVR_A_STAR_32U4
    
#ifdef USE_SPI
    pinMode(SCK, INPUT);
    pinMode(MISO, OUTPUT);
    pinMode(MOSI, INPUT);
    pinMode(SS, INPUT);
    
    // turn on SPI in slave mode
    SPCR |= _BV(SPE);
    SPI.attachInterrupt();
#else
    CB_INBOUND_SERIAL_PORT.begin(CB_INBOUND_SERIAL_BAUD);

#endif // USE_SPI
#endif // Arduino
    
#ifdef __RFduino__
    RFduinoBLE.customUUID = BLE_UUID;
    RFduinoBLE.deviceName = BLE_LOCALNAME;
    RFduinoBLE.advertisementData = BLE_ADVERTISEMENT_DATA;
    RFduinoBLE.begin();
    
#endif
}


// decalred and instatiated here as they are shared by both the C++ class (UART mode) an Interrupt handler (SPI mode) and Cannybots::Update

// serial (UART & SPI) buffer
uint8_t serialBuffer[SERIAL_BUF_SIZE+1];
int serialBufPtr=0;
bool foundStart=false;

char c, lastChar;




void Cannybots::update() {
#ifdef ARDUINO
#ifdef __RFduino__
    // not polled, event driven in: RFduinoBLE_onReceive()
#else
    // Arduino...
#ifdef USE_SPI
    // not polled, event driven using AVR SPI interrupt handler
    if (serialBufPtr>=CB_MAX_MSG_SIZE) {
        Message* msg = new Message(serialBuffer, CB_MAX_MSG_SIZE);
        serialBufPtr=0;
        foundStart = false;
        inboundMsgFIFO.enqueue(msg)
        Serial.println("<EOC>");

    }
#else
    readSerial(Serial1);
#endif
#endif
#endif
    processOutboundMessageQueue();
    processInboundMessageQueue();
}


/////////////////////////////////////////////////////////////////////
// comms



#ifdef ARDUINO
#ifdef USE_SPI

// interrupt handler called on recept of SPI data
ISR (SPI_STC_vect)
{
     c = SPDR;  // grab byte from SPI Data Register
    if (foundStart && (serialBufPtr<SERIAL_BUF_SIZE)) {
        serialBuffer[serialBufPtr++] = c;
    } else if ( ('>' == c) && ('>' == lastChar) ) {
        foundStart=true;
        serialBufPtr=0;
    }
}


#else
void Cannybots::readSerial(HardwareSerial &ser) {
    while (ser.available()>0) {
        lastChar = c;
        c =  ser.read();
        //Serial.print(" ");
        //Serial.print(c, HEX);
        
        if (foundStart && (serialBufPtr<SERIAL_BUF_SIZE)) {
            serialBuffer[serialBufPtr++] = c;
        } else if ( ('>' == c) && ('>' == lastChar) ) {
            foundStart=true;
            serialBufPtr=0;
        }
        
        if (serialBufPtr>=CB_MAX_MSG_SIZE) {
            //Serial.println("<EOC>");
            foundStart = false;
            Message* msg = new Message(serialBuffer, CB_MAX_MSG_SIZE);
            inboundMsgFIFO.enqueue(msg);
            
            serialBufPtr=0;
        }
    }
}
#endif // USE_SPI
#endif // ARDUINO

#ifdef __RFduino__
void RFduinoBLE_onReceive(char *data, int len) {
    Message* msg = new Message((uint8_t*)data, len);
    Cannybots& cb = Cannybots::getInstance();

    cb.addInboundMessage(msg);
}
#endif // __RFduino__





/////////

void Cannybots::processInboundMessageQueue() {
    
    for (int i = 0; i < inboundMsgFIFO.count(); i++) {
        //§       CB_DBG("processInboundMessageQueue(%d)", i+1);
        
        Message* msg = inboundMsgFIFO.dequeue();
#ifdef ARDUINO
        lastInboundMessageTime = millis();
#endif
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
        //CB_DBG("processOutboundMessageQueue(%d)", i+1);
        Message* msg = outboundMsgFIFO.dequeue();
#ifdef ARDUINO
#if defined(ARDUINO_AVR_LEONARDO)  || defined(ARDUINO_AVR_A_STAR_32U4)
        Serial1.write(">>");
        Serial1.write(msg->payload, CB_MAX_MSG_SIZE);
#else
        
#ifdef __RFduino__
        CB_DBG("BLE len = %d", msg->size);
         RFduinoBLE.send((char*)msg->payload, msg->size);
#else
        Serial.write(">>");
        Serial.write(msg->payload, CB_MAX_MSG_SIZE);
#endif //__RFduino__

#endif // ARDUINO_AVR_LEONARDO
#endif // ARDUINO
        
        delete msg;
    }
}

// Command processing

// ok on Arduino 1.5.6r2 RFd SDK v2.0.3 but not the latest ARM gcc for Arduino IDE 1.5.7 + RFd SDK v2.1
//#import <cstring>

void Cannybots::processMessage(Message* msg ) {
    //CB_DBG("processMessage()",0);
	
    // just print non-CB messages as strings
    if ( (msg->payload[0] != 'C'  ||  (msg->payload[1] != 'B' ) ) ) {
        //CB_DBG("Non CB message", 0);
        msg->payload[ (msg->size)-1]=0;
        CB_DBG("PM:%s", msg->payload);
        return;
    }
    // TODO: check CRC, payload length and if this is a contination
    
    uint8_t cmd = msg->payload[CB_MSG_OFFSET_CMD];
    //CB_DBG("cmd= %d", cmd);
    cb_descriptor* desc = getDescriptorForCommand(cmd);
    
    if (desc && CB_CMD_IS_METHOD(cmd)) {
        //CB_DBG("isMethod",0);
        switch (desc->type) {
            case CB_STRING:
                ((cb_callback_string)desc->data)((const char*)& msg->payload[CB_MSG_OFFSET_DATA+0]);
                break;
            case CB_INT16_3:
                ((cb_callback_int16_int16_int16)desc->data)(
                                                           mk16bit( msg->payload[CB_MSG_OFFSET_DATA+1],msg->payload[CB_MSG_OFFSET_DATA+0]),
                                                           mk16bit( msg->payload[CB_MSG_OFFSET_DATA+3],msg->payload[CB_MSG_OFFSET_DATA+2]),
                                                           mk16bit( msg->payload[CB_MSG_OFFSET_DATA+5],msg->payload[CB_MSG_OFFSET_DATA+4]));
                break;
            case CB_INT16_2:
                ((cb_callback_int16_int16)desc->data)(
                                                     mk16bit( msg->payload[CB_MSG_OFFSET_DATA+1],msg->payload[CB_MSG_OFFSET_DATA+0]),
                                                     mk16bit( msg->payload[CB_MSG_OFFSET_DATA+3],msg->payload[CB_MSG_OFFSET_DATA+2]));
                break;
            case CB_INT16_1:
                ((cb_callback_int16)desc->data)(
                                               mk16bit( msg->payload[CB_MSG_OFFSET_DATA+1],msg->payload[CB_MSG_OFFSET_DATA+0]));
                break;
            default:
                break;
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
    unsigned long mem_free = vm_stat.free_count * pagesize;
    //unsigned long mem_total = mem_used + mem_free;
    return mem_free;
#endif
}



// Message Creation Helpers
// TODO generalise, and make use of para count
void Cannybots::createMessage(Message* msg, cb_id* cid, int16_t p1, int16_t p2, int16_t p3) {
    uint8_t tmpMsg[CB_MAX_MSG_SIZE] = {
        'C', 'B', 0,
        Cannybots::CB_INT16_3, cid->cid,
        3,
        hiByteFromInt(p1),loByteFromInt(p1),
        hiByteFromInt(p2),loByteFromInt(p2),
        hiByteFromInt(p3),loByteFromInt(p3),
        0,0,  0,0,  0,0, 0,0
    };
    memcpy((char*)msg->payload, (char*)tmpMsg, CB_MAX_MSG_SIZE);
    msg->size = CB_MAX_MSG_SIZE;
}
void Cannybots::createMessage(Message* msg, cb_id* cid, int16_t p1, int16_t p2) {
    uint8_t tmpMsg[CB_MAX_MSG_SIZE] = {
        'C', 'B', 0,
        Cannybots::CB_INT16_2, cid->cid,
        2,
        hiByteFromInt(p1),loByteFromInt(p1),
        hiByteFromInt(p2),loByteFromInt(p2),
        0,0,
        0,0,  0,0,  0,0, 0,0
    };
    memcpy((char*)msg->payload, (char*)tmpMsg, CB_MAX_MSG_SIZE);
    msg->size = CB_MAX_MSG_SIZE;
}
void Cannybots::createMessage(Message* msg, cb_id* cid, int16_t p1) {
    uint8_t tmpMsg[CB_MAX_MSG_SIZE] = {
        'C', 'B', 0,
        Cannybots::CB_INT16_2, cid->cid,
        1,
        hiByteFromInt(p1),loByteFromInt(p1),
        0,0,
        0,0,
        0,0,  0,0,  0,0, 0,0
    };
    memcpy((char*)msg->payload, (char*)tmpMsg, CB_MAX_MSG_SIZE);
    msg->size = CB_MAX_MSG_SIZE;
}

void Cannybots::createMessage(Message* msg, cb_id* cid, const char* p1) {
    uint8_t tmpMsg[CB_MAX_MSG_SIZE] = {
        'C', 'B', 0,
        Cannybots::CB_STRING, cid->cid,
        14,
        0,0,
        0,0,
        0,0,
        0,0,  0,0,  0,0, 0,0
    };
    strncpy((char*)& tmpMsg[CB_MSG_OFFSET_DATA], p1, strlen(p1));
    
    
    memcpy((char*)msg->payload, (char*)tmpMsg, CB_MAX_MSG_SIZE);
    msg->size = CB_MAX_MSG_SIZE;
}


// REmote invocation

void Cannybots::callMethod(cb_id* cid, int16_t p1) {
    Message* msg = new Message();
    createMessage(msg, cid, p1, 0, 0);
    addOutboundMessage(msg);
}

void Cannybots::callMethod(cb_id* cid, int16_t p1,int16_t p2,int16_t p3) {
    Message* msg = new Message();
    createMessage(msg, cid, p1, p2, p3);
    addOutboundMessage(msg);
}

// String (null terminated) type
void Cannybots::callMethod(cb_id* cid, const char* p1) {
    
    uint16_t charsRemaining = strlen(p1);
    
    if (charsRemaining < CB_MAX_MSG_DATA_SIZE) {
        Message* msg = new Message();
        createMessage(msg, cid, p1);
        addOutboundMessage(msg);
        
    } else {
        char tmpBuf[CB_MAX_MSG_DATA_SIZE];
        char* ptr = (char*)p1;
        
        while (charsRemaining>0) {
        
            uint8_t len = CB_MAX_MSG_DATA_SIZE-1;
            
            if (strlen(ptr) <CB_MAX_MSG_DATA_SIZE) {
                len = strlen(ptr);
            }
            memcpy(tmpBuf,ptr, len);
            tmpBuf[len]=0;
            
            charsRemaining-=len;
            ptr+=len;
            Message* msg = new Message();
            createMessage(msg, cid, tmpBuf);
            addOutboundMessage(msg);
        };
    }
    
}


///////////////////////////////////////////////////////////////////////
// Non-volatile methods

// limit the number of writes, just in case we get stuck in a loop during dev!
int nvWritesSinceBoot=0;
bool  Cannybots::nvSetByte(uint16_t address, uint8_t b) {
    nvWritesSinceBoot++;
    if (nvWritesSinceBoot < 1000) {
        
#ifdef ARDUINO
        
#ifdef __RFduino__
        
        // see:  https://github.com/RFduino/RFduino/tree/master/libraries/RFduinoNonBLE/examples/Flash/FlashInteger
#else
        EEPROM.write(address, b);
#endif
#endif
    } else {
        //WARN("exceeded max EEPROM writes for this session");
    }
    return 0;
}


uint8_t  Cannybots::nvGetByte(uint16_t address) {
    uint8_t b = 0;
#ifdef ARDUINO
    
#ifdef __RFduino__
    // see:  https://github.com/RFduino/RFduino/tree/master/libraries/RFduinoNonBLE/examples/Flash/FlashInteger
#else
    b = EEPROM.read(address);
#endif
#endif
    return b;
}



bool Cannybots::nvSetInt(uint16_t address, uint16_t b) {
    uint8_t b1 = hiByteFromInt(b);
    uint8_t b2 = loByteFromInt(b);
    
    nvSetByte(address,b1);
    nvSetByte(address+1,b2);
    return 0;
}



uint16_t Cannybots::nvGetInt(uint16_t address) {
    uint8_t b1 = nvGetByte(address);
    uint8_t b2 = nvGetByte(address+1);
    int16_t b = mk16bit(b2,b1);
    return b;
}


bool Cannybots::nvSetByte(cb_nv_id* address, uint8_t b) {
    return nvSetByte((uint16_t)address->offset, b);
}

uint8_t Cannybots::nvGetByte(cb_nv_id* address) {
    return nvGetByte((uint16_t)0);
    
}

bool Cannybots::nvSetInt(cb_nv_id* address, uint16_t b) {
    return nvSetInt((uint16_t)address->offset, b);

}

uint16_t Cannybots::nvGetInt(cb_nv_id* address) {
    return nvGetInt((uint16_t)address->offset);
}



bool Cannybots::nvSetupConfig() {
    nvSetByte((uint16_t)0, 'C');
    nvSetByte(1, 'N');
    nvSetByte(2, 'Y');
    nvSetByte(3, 'B');
    return 0;
}

bool Cannybots::nvIsValidConfig() {
    uint8_t b1 = nvGetByte((uint16_t)0);
    uint8_t b2 = nvGetByte(1);
    uint8_t b3 = nvGetByte(2);
    uint8_t b4 = nvGetByte(3);
    
    return (  (b1 == 'C') && (b2 == 'N') && (b3 == 'Y') && (b4 == 'B') );
}

