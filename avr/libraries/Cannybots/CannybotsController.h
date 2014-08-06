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

- (void) registerHandler:(cb_id*)cid withBlockFor_INT16_3:(cb_bridged_callback_int16_3)block;
- (void) registerHandler:(cb_id*)cid withBlockFor_INT16_2:(cb_bridged_callback_int16_2)block;
- (void) registerHandler:(cb_id*)cid withBlockFor_INT16_1:(cb_bridged_callback_int16_1)block;

- (void) registerHandler:(cb_id*)cid withBlockFor_STRING:(cb_bridged_callback_string)block;
- (void) deregisterHandler:(cb_id*)cid;

@end
