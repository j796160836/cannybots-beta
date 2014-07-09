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


@protocol CannybotsReceiver
- (void) didReceiveData:(NSData*)data;
@end



@interface CannybotsController : NSObject <CannybotsReceiver>

+ (id)sharedInstance;


- (void) callMethod:(cb_id)cid p1:(int16_t)p1 p2:(int16_t)p2 p3:(int16_t)p3;

@end
