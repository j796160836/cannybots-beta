//
//  CannybotsController.h
//  BrainDash
//
//  Created by Wayne Keenan on 09/07/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import <Foundation/Foundation.h>
#import "NSData+hex.h"

#import "CannybotsTypes.h"


// for ObjC / C++ blocks
// TODO: typedef the other method signatures...
typedef void (^cb_bridged_callback_int16_3)(int16_t, int16_t, int16_t);
typedef void (^cb_bridged_callback_int16_2)(int16_t, int16_t);
typedef void (^cb_bridged_callback_int16_1)(int16_t);
typedef void (^cb_bridged_callback_string)(const char*);

@protocol CannybotsReceiver
- (void) didReceiveData:(NSData*)data;
@end



@interface CannybotsController : NSObject <CannybotsReceiver>

+ (id)sharedInstance;



- (void) callMethod:(cb_id*)cid p1:(int16_t)p1 p2:(int16_t)p2 p3:(int16_t)p3;
- (void) callMethod:(cb_id*)cid p1:(int16_t)p1 p2:(int16_t)p2;
- (void) callMethod:(cb_id*)cid p1:(int16_t)p1;

// TODO: methods for the other types...
- (void) registerHandler:(cb_id*)cid withBlockFor_INT16_3:(cb_bridged_callback_int16_3)block;
- (void) registerHandler:(cb_id*)cid withBlockFor_INT16_2:(cb_bridged_callback_int16_2)block;
- (void) registerHandler:(cb_id*)cid withBlockFor_INT16_1:(cb_bridged_callback_int16_1)block;

- (void) registerHandler:(cb_id*)cid withBlockFor_STRING:(cb_bridged_callback_string)block;
- (void) deregisterHandler:(cb_id*)cid;

- (NSArray*) getConfigParameterList;


// TODO: methods for the other types...4
- (void) setConfigParameter_BOOL:(cb_nv_id*)cid p1:(uint8_t)p1;
- (void) setConfigParameter_BYTE:(cb_nv_id*)cid p1:(uint8_t)p1;
- (void) setConfigParameter_UINT8:(cb_nv_id*)cid p1:(uint8_t)p1;
- (void) setConfigParameter_INT8:(cb_nv_id*)cid p1:(int8_t)p1;
- (void) setConfigParameter_UINT16:(cb_nv_id*)cid p1:(uint16_t)p1;
- (void) setConfigParameter_INT16:(cb_nv_id*)cid p1:(int16_t)p1;
//- (void) setConfigParameter_UINT32:(cb_nv_id*)cid p1:(uint32_t)p1;
//- (void) setConfigParameter_INT32:(cb_nv_id*)cid p1:(int32_t)p1;


@end
