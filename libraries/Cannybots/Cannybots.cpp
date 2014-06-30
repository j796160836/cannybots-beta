
#include "Cannybots.h"

// class: cb_id

uint16_t cb_id::nextId = 1;





// class: CAnnybots
// define statics
const cb_publish_type Cannybots::PUBLISH_UPDATE_ONCHANGE = 1;




Cannybots::Cannybots () {

}


void Cannybots::setConfigStorage(const char* magic, uint16_t start) {
    
}

// Scalar
void Cannybots::registerVariable(const cb_id& _id, int16_t* var, const bool isNonVolatile, const int16_t defaultValue) {
    
}

void Cannybots::registerVariable(const cb_id& _id, bool*    var, const bool isNonVolatile, const bool    defaultValue) {
    
}

// Arrays
void Cannybots::registerArray(const cb_id _id, int16_t list[], const uint16_t length) {
    
}

// Published values
void Cannybots::registerPublisher(const cb_id _id, bool *var, const cb_publish_type pubType) {
    
}


// Function handlers
void Cannybots::registerHandler(const cb_id _id, cb_callback_int16_int16) {
    
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
    
}

void Cannybots::update() {
    
}