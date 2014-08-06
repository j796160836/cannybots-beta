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





/*
 LinkedList.h - V1.1 - Generic LinkedList implementation
 Works better with FIFO, because LIFO will need to
 search the entire List to find the last one;
 
 For instructions, go to https://github.com/ivanseidel/LinkedList
 
 Created by Ivan Seidel Gomes, March, 2013.
 Released into the public domain.
 */



template<class T>
struct ListNode
{
	T data;
	ListNode<T> *next;
};

template <typename T>
class LinkedList{
    
protected:
	int _size;
	ListNode<T> *root;
	ListNode<T>	*last;
    
	// Helps "get" method, by saving last position
	ListNode<T> *lastNodeGot;
	int lastIndexGot;
	// isCached should be set to FALSE
	// everytime the list suffer changes
	bool isCached;
    
	ListNode<T>* getNode(int index);
    
public:
	LinkedList();
	~LinkedList();
    
	/*
     Returns current size of LinkedList
     */
	virtual int size();
	/*
     Adds a T object in the specified index;
     Unlink and link the LinkedList correcly;
     Increment _size
     */
	virtual bool add(int index, T);
	/*
     Adds a T object in the end of the LinkedList;
     Increment _size;
     */
	virtual bool add(T);
	/*
     Adds a T object in the start of the LinkedList;
     Increment _size;
     */
	virtual bool unshift(T);
	/*
     Set the object at index, with T;
     Increment _size;
     */
	virtual bool set(int index, T);
	/*
     Remove object at index;
     If index is not reachable, returns false;
     else, decrement _size
     */
	virtual T remove(int index);
	/*
     Remove last object;
     */
	virtual T pop();
	/*
     Remove first object;
     */
	virtual T shift();
	/*
     Get the index'th element on the list;
     Return Element if accessible,
     else, return false;
     */
	virtual T get(int index, bool useCached);
    
	/*
     Clear the entire array
     */
	virtual void clear();
    
};

// Initialize LinkedList with false values
template<typename T>
LinkedList<T>::LinkedList()
{
	root=false;
	last=false;
	_size=0;
    
	lastNodeGot = root;
	lastIndexGot = 0;
	isCached = false;
}

// Clear Nodes and free Memory
template<typename T>
LinkedList<T>::~LinkedList()
{
	ListNode<T>* tmp;
	while(root!=false)
	{
		tmp=root;
		root=root->next;
		delete tmp;
	}
	last = false;
	_size=0;
	isCached = false;
}

/*
 Actualy "logic" coding
 */

template<typename T>
ListNode<T>* LinkedList<T>::getNode(int index){
    
	int _pos = 0;
	ListNode<T>* current = root;
    
	// Check if the node trying to get is
	// immediatly AFTER the previous got one
	if(isCached && lastIndexGot <= index){
		_pos = lastIndexGot;
		current = lastNodeGot;
	}
    
	while(_pos < index && current){
		current = current->next;
        
		_pos++;
	}
    
	// Check if the object index got is the same as the required
	if(_pos == index){
		isCached = true;
		lastIndexGot = index;
		lastNodeGot = current;
        
		return current;
	}
    
	return false;
}

template<typename T>
int LinkedList<T>::size(){
	return _size;
}

template<typename T>
bool LinkedList<T>::add(int index, T _t){
    
	// if(index == 0)
	// 	return shift(_t);
    
	if(index >= _size)
		return add(_t);
    
	ListNode<T> *tmp = new ListNode<T>(),
    *_prev = getNode(index-1);
	tmp->data = _t;
	tmp->next = _prev->next;
	_prev->next = tmp;
    
	_size++;
	isCached = false;
    
	return true;
}

template<typename T>
bool LinkedList<T>::add(T _t){
    
	ListNode<T> *tmp = new ListNode<T>();
	tmp->data = _t;
	tmp->next = false;
	
	if(root){
		// Already have elements inserted
		last->next = tmp;
		last = tmp;
	}else{
		// First element being inserted
		root = tmp;
		last = tmp;
	}
    
	_size++;
	isCached = false;
    
	return true;
}

template<typename T>
bool LinkedList<T>::unshift(T _t){
    
	if(_size == 0)
		return add(_t);
    
	ListNode<T> *tmp = new ListNode<T>();
	tmp->next = root;
	tmp->data = _t;
	
	_size++;
	isCached = false;
	
	return true;
}

template<typename T>
bool LinkedList<T>::set(int index, T _t){
	// Check if index position is in bounds
	if(index < 0 || index >= _size)
		return false;
    
	getNode(index)->data = _t;
	return true;
}

template<typename T>
T LinkedList<T>::pop(){
	if(_size <= 0)
		return T();
	
	isCached = false;
    
	if(_size >= 2){
		ListNode<T> *tmp = getNode(_size - 2);
		T ret = tmp->next->data;
		delete(tmp->next);
		tmp->next = false;
		last = tmp;
		_size--;
		return ret;
	}else{
		// Only one element left on the list
		T ret = root->data;
		delete(root);
		root = false;
		last = false;
		_size = 0;
		return ret;
	}
}

template<typename T>
T LinkedList<T>::shift(){
	if(_size <= 0)
		return T();
    
	if(_size > 1){
		ListNode<T> *_next = root->next;
		T ret = root->data;
		delete(root);
		root = _next;
		_size --;
		isCached = false;
        
		return ret;
	}else{
		// Only one left, then pop()
		return pop();
	}
    
}

template<typename T>
T LinkedList<T>::remove(int index){
	if(index < 0 || index >= _size)
		return T();
    
	if(index == 0)
		return shift();
    
	if(index - 1 == _size)
		return pop();
    
	ListNode<T> *tmp = getNode(index - 1);
	ListNode<T> *toDelete = tmp->next;
	tmp->next = tmp->next->next;
	delete(toDelete);
	_size--;
	isCached = false;
    
	return T();
}

template<typename T>
T LinkedList<T>::get(int index, bool useCached = false){
	ListNode<T> *tmp = getNode(index);
    
	return (tmp ? tmp->data : T());
}

template<typename T>
void LinkedList<T>::clear(){
	while(size() > 0)
		shift();
}


#endif
