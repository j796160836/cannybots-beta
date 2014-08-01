//
//  Cannybots.m
//  BrainDash
//
//  Created by Wayne Keenan on 09/07/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import "CannybotsController.h"

#import "Cannybots.h"
#import "BrainSpeakBLE.h"


@interface CannybotsController ()
{
    Cannybots* cb;
    BrainSpeakBLE*  bsle;
}
@end

@implementation CannybotsController


#pragma mark Singleton Methods

+ (id)sharedInstance {
    static CannybotsController *shared = nil;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
        shared = [[self alloc] init];
    });
    return shared;
}

- (id)init {
    if (self = [super init]) {
        //someProperty = [[NSString alloc] initWithString:@"Default Property Value"];
        cb = &Cannybots::getInstance();
        
        bsle     = [BrainSpeakBLE sharedInstance];
        bsle.cbdelegate = self;

        [self registerHandler:_CB_SYS_LOG withBlockFor_STRING:^(const char* p1){
            NSLog(@"CB_REMOTE_DBG:%s", p1);
            }
         ];
    }
    return self;
}

- (void)dealloc {
    // Should never be called, but just here for clarity really.
}


// Utils

- (void) sendMessage:(Message*)msg {
   NSData *data = [NSData dataWithBytesNoCopy:(void*)msg->payload length:msg->size freeWhenDone:NO];
  [bsle sendData:data];
}


// Callers
// TODO: add to an outbound queue (?)
// TODO: create an C++ BLE API adapter plugin api for iOS, Android, Pi/Linux etc.

- (void) callMethod:(cb_id)cid p1:(int16_t)p1 p2:(int16_t)p2 p3:(int16_t)p3 {
    Message msg;
    cb->createMessage(&msg, cid, p1, p2,p3);
    [self sendMessage:&msg];
}

- (void) callMethod:(cb_id)cid p1:(int16_t)p1 p2:(int16_t)p2{
    Message msg;
    cb->createMessage(&msg, cid, p1, p2);
    [self sendMessage:&msg];
}
- (void) callMethod:(cb_id)cid p1:(int16_t)p1 {
    Message msg;
    cb->createMessage(&msg, cid, p1);
    [self sendMessage:&msg];
}

- (void) registerHandler:(cb_id)cid withBlockFor_INT16_3:(cb_bridged_callback_int16_3)block {
    NSLog(@"registerHandler: %d", cid.cid);
    cb->registerHandler(cid, Cannybots::CB_INT16_3, (void*)CFBridgingRetain(block));
    
}

- (void) registerHandler:(cb_id)cid withBlockFor_INT16_2:(cb_bridged_callback_int16_2)block {
    NSLog(@"registerHandler: %d", cid.cid);
    cb->registerHandler(cid, Cannybots::CB_INT16_2, (void*)CFBridgingRetain(block));
    
}

- (void) registerHandler:(cb_id)cid withBlockFor_INT16_1:(cb_bridged_callback_int16_1)block {
    NSLog(@"registerHandler: %d", cid.cid);
    cb->registerHandler(cid, Cannybots::CB_INT16_1, (void*)CFBridgingRetain(block));
    
}

- (void) registerHandler:(cb_id)cid withBlockFor_STRING:(cb_bridged_callback_string)block {
    NSLog(@"registerHandler: %d", cid.cid);
    cb->registerHandler(cid, Cannybots::CB_STRING, (void*)CFBridgingRetain(block));
    
}

- (void) deregisterHandler:(cb_id)cid  {
    NSLog(@"degisterHandler: %d", cid.cid);

    CFBridgingRelease(cb->descriptors[cid.cid].data);
    cb->descriptors[cid.cid].data=0;
    
}


- (void) didReceiveData:(NSData *)data {
    //long      len = [data length];
    uint8_t * buf = (uint8_t*)[data bytes];
    
    NSString* hexString = [data hexRepresentationWithSpaces:YES];
    //NSLog(@"Received: %@", hexString);
    
    uint8_t cid = buf[CB_MSG_OFFSET_CMD];
    //NSLog(@"cmd= %d", cid);
    cb_descriptor desc = cb->descriptors[cid];
    
    if (desc.isMethod) {
        //NSLog(@"isMethod");
        
        if (desc.type == Cannybots::CB_INT16_3) {
            //NSLog(@"is CB_INT16_3");
            //NSLog(@"block @ %x", desc.data);
            if (desc.data) {
                ((__bridge cb_bridged_callback_int16_3)desc.data)( mk16bit( buf[CB_MSG_OFFSET_DATA+1],buf[CB_MSG_OFFSET_DATA+0]),
                                                          mk16bit( buf[CB_MSG_OFFSET_DATA+3],buf[CB_MSG_OFFSET_DATA+2]),
                                                          mk16bit( buf[CB_MSG_OFFSET_DATA+5],buf[CB_MSG_OFFSET_DATA+4]));
            }
        } else if (desc.type == Cannybots::CB_INT16_2) {
            //NSLog(@"is CB_INT16_2");
            //NSLog(@"block @ %x", desc.data);
            if (desc.data) {
                ((__bridge cb_bridged_callback_int16_2)desc.data)( mk16bit( buf[CB_MSG_OFFSET_DATA+1],buf[CB_MSG_OFFSET_DATA+0]),
                                                          mk16bit( buf[CB_MSG_OFFSET_DATA+3],buf[CB_MSG_OFFSET_DATA+2]));
            }
        } else if (desc.type == Cannybots::CB_INT16_1) {
            //NSLog(@"is CB_INT16_2");
            //NSLog(@"block @ %x", desc.data);
            if (desc.data) {
                ((__bridge cb_bridged_callback_int16_1)desc.data)( mk16bit( buf[CB_MSG_OFFSET_DATA+1],buf[CB_MSG_OFFSET_DATA+0]));
            }
        } else if (desc.type == Cannybots::CB_STRING) {
            //NSLog(@"is CB_INT16_2");
            //NSLog(@"block @ %x", desc.data);
            if (desc.data) {
                ((__bridge cb_bridged_callback_string)desc.data)( (const char*) &buf[CB_MSG_OFFSET_DATA+0]);
            }
        } else {
            NSLog(@"ERROR: unrecognised desc.type in didReceiveData (%d)", desc.isMethod);
        }

    }

}


//////////////////////////////////////////////////////////////////////////////////////
// Bluetooth





@end
