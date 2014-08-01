//
//  CannybotsInt.h
//
//  Created by Wayne Keenan
//  Copyright (c) 2014 CannyBots. All rights reserved.
//
// FIFO based on:  https://github.com/rambo/SimpleFIFO

#ifndef CannybotsInt_h
#define CannybotsInt_h

#include <CannybotsTypes.h>

template<typename T, int rawSize>
class CBFIFO {
public:
	const char size;				//speculative feature, in case it's needed
    
	CBFIFO();
    
	T dequeue();				//get next element
	bool enqueue( T element );	//add an element
	T peek() const;				//get the next element without releasing it from the FIFO
	void flush();				//[1.1] reset to default state
    
	//how many elements are currently in the FIFO?
	char count() { return numberOfElements; }
    
private:
#ifndef SimpleFIFO_NONVOLATILE
	volatile char numberOfElements;
	volatile char nextIn;
	volatile char nextOut;
	volatile T raw[rawSize];
#else
	char numberOfElements;
	char nextIn;
	char nextOut;
	T raw[rawSize];
#endif
};

template<typename T, int rawSize>
CBFIFO<T,rawSize>::CBFIFO() : size(rawSize) {
	flush();
}
template<typename T, int rawSize>
bool CBFIFO<T,rawSize>::enqueue( T element ) {
	if ( count() >= rawSize ) { return false; }
	numberOfElements++;
	nextIn %= size;
	raw[nextIn] = element;
	nextIn++; //advance to next index
	return true;
}
template<typename T, int rawSize>
T CBFIFO<T,rawSize>::dequeue() {
	numberOfElements--;
	nextOut %= size;
	return raw[ nextOut++];
}
template<typename T, int rawSize>
T CBFIFO<T,rawSize>::peek() const {
	return raw[ nextOut % size];
}
template<typename T, int rawSize>
void CBFIFO<T,rawSize>::flush() {
	nextIn = nextOut = numberOfElements = 0;
}



class Message {
public:
    Message(): size(0) {
    };
    Message(uint8_t* buffer, uint16_t len) {
        memcpy(payload, buffer, len<CB_MAX_MSG_SIZE?len:CB_MAX_MSG_SIZE);
        size=len;
    };
    uint8_t payload[CB_MAX_MSG_SIZE];
    uint8_t size;
};




#endif
