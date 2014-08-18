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

        
        cb->begin();

        // TODO: re-instate this when we can push/pop handlers using the API
        /*[self registerHandler:&_CB_SYS_LOG withBlockFor_STRING:^(const char* p1){
            NSLog(@"CB_REMOTE_DBG:%s", p1);
        }
        
         ];*/
        
        
        
        [self registerHandler:&_CB_SYS_CALL withBlockFor_INT16_3:^(int16_t p1, int16_t p2, int16_t p3) {
            switch (p1) {
                case _CB_SYSCALL_NOP: break;
                case _CB_SYSCALL_CFG_PARAM_META: {
                    
                    cb_descriptor* desc = cb->getDescriptorForConfigParameter(p2);
                    if (desc) {
                        NSLog(@"param[%d][%s] type=%d", p2, desc->cid_t.cidNV->name, p3);
                    } else {
                        NSLog(@"unknown param %d", p2);
                    }
                    
                    break;
                }
                    
                default:
                    NSLog(@"Unrecognised Sys call: %d, %d,%d", p1, p2, p3);
            }
        }
        ];

        //            NSLog(@"CB_REMOTE_DBG:%s", p1);
        
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

- (void) callMethod:(cb_id*)cid p1:(int16_t)p1 p2:(int16_t)p2 p3:(int16_t)p3 {
    Message msg;
    cb->createMessage(&msg, cid, p1, p2,p3);
    [self sendMessage:&msg];
}

- (void) callMethod:(cb_id*)cid p1:(int16_t)p1 p2:(int16_t)p2{
    Message msg;
    cb->createMessage(&msg, cid, p1, p2);
    [self sendMessage:&msg];
}
- (void) callMethod:(cb_id*)cid p1:(int16_t)p1 {
    Message msg;
    cb->createMessage(&msg, cid, p1);
    [self sendMessage:&msg];
}

- (void) registerHandler:(cb_id*)cid withBlockFor_INT16_3:(cb_bridged_callback_int16_3)block {
    @synchronized(self) {
        NSLog(@"registerHandler: %d", cid->cid);
        cb->registerHandler(cid, Cannybots::CB_INT16_3, (void*)CFBridgingRetain(block));
    }
}

- (void) registerHandler:(cb_id*)cid withBlockFor_INT16_2:(cb_bridged_callback_int16_2)block {
    @synchronized(self) {
        
        NSLog(@"registerHandler: %d", cid->cid);
        cb->registerHandler(cid, Cannybots::CB_INT16_2, (void*)CFBridgingRetain(block));
    }
}

- (void) registerHandler:(cb_id*)cid withBlockFor_INT16_1:(cb_bridged_callback_int16_1)block {
    @synchronized(self) {
        
        NSLog(@"registerHandler: %d", cid->cid);
        cb->registerHandler(cid, Cannybots::CB_INT16_1, (void*)CFBridgingRetain(block));
    }
}

- (void) registerHandler:(cb_id*)cid withBlockFor_STRING:(cb_bridged_callback_string)block {
    @synchronized(self) {
        
        NSLog(@"registerHandler: %d", cid->cid);
        cb->registerHandler(cid, Cannybots::CB_STRING, (void*)CFBridgingRetain(block));
    }
}

- (void) deregisterHandler:(cb_id*)cid  {
    return;
    // TODO: fix the data sync issue when this trys to modifiy the underlying link list whilst data may be received.
    @synchronized(self) {
        NSLog(@"degisterHandler: %d", cid->cid);
        
        cb_descriptor* desc = cb->getDescriptorForCommand(cid->cid);
        if (desc) {
            CFBridgingRelease(desc->data);
            //desc->data=0;
            cb->deregisterHandler(cid);
        } else {
            NSLog(@"deregistar called when 'desc' is NULL on %d", cid->cid);
        }
    }
}

- (NSArray*) getConfigParameterList {
    
    //cb->getConfigParameterListFromRemote(); // call returns data asyncronoulsy
    [self callMethod:&_CB_SYS_CALL p1:_CB_SYSCALL_GET_CFG_LIST p2:0 p3:0];
    
    int len = cb->getConfigParameterListSize();
    NSMutableArray* desc = [[NSMutableArray alloc] initWithCapacity:len];
    
    for (int index = 0; index < len; index ++ ){
        //cb_descriptor* desc = cb->getConfigParameterListItem(index);
    }
    
    return desc;
}



- (void) setConfigParameter_BOOL:(cb_nv_id*)cid p1:(uint8_t)p1 {
    
    [self callMethod:&_CB_SYS_CALL p1:_CB_SYSCALL_SET_CFG_PARAM p2:cid->cid  p3:p1];

    
}

- (void) setConfigParameter_BYTE:(cb_nv_id*)cid p1:(uint8_t)p1 {
    [self callMethod:&_CB_SYS_CALL p1:_CB_SYSCALL_SET_CFG_PARAM p2:cid->cid  p3:p1];
}


- (void) setConfigParameter_INT8:(cb_nv_id*)cid p1:(int8_t)p1 {
    [self callMethod:&_CB_SYS_CALL p1:_CB_SYSCALL_SET_CFG_PARAM p2:cid->cid  p3:p1];
}
- (void) setConfigParameter_UINT8:(cb_nv_id*)cid p1:(uint8_t)p1 {
    [self callMethod:&_CB_SYS_CALL p1:_CB_SYSCALL_SET_CFG_PARAM p2:cid->cid  p3:p1];
}


- (void) setConfigParameter_INT16:(cb_nv_id*)cid p1:(int16_t)p1 {
    [self callMethod:&_CB_SYS_CALL p1:_CB_SYSCALL_SET_CFG_PARAM p2:cid->cid  p3:p1];
    
}

- (void) setConfigParameter_UINT16:(cb_nv_id*)cid p1:(uint16_t)p1 {
    [self callMethod:&_CB_SYS_CALL p1:_CB_SYSCALL_SET_CFG_PARAM p2:cid->cid  p3:p1];
    
}

//- (void) setConfigParameter_UINT32:(cb_nv_id*)cid p1:(uint32_t)p1
//- (void) setConfigParameter_INT32:(cb_nv_id*)cid p1:(int32_t)p1



- (void) didReceiveDataCBLIB:(NSData *)data {
        @synchronized(self) {
    //long      len = [data length];
    uint8_t * buf = (uint8_t*)[data bytes];
    
    NSString* hexString = [data hexRepresentationWithSpaces:YES];
    NSLog(@"Received: %@", hexString);
    
    uint8_t commandId = buf[CB_MSG_OFFSET_CMD];
    cb_descriptor* desc = cb->getDescriptorForCommand(commandId);
    //NSLog(@"cmd= %d", commandId);
    
    if (desc && CB_CMD_IS_METHOD(commandId)) {
        //NSLog(@"Method=%s", desc->cid_t.cidMT->name);
        
        if (desc->type == Cannybots::CB_INT16_3) {
            //NSLog(@"is CB_INT16_3");
            //NSLog(@"block @ %x", desc->data);
            if (desc->data) {
                ((__bridge cb_bridged_callback_int16_3)desc->data)( mk16bit( buf[CB_MSG_OFFSET_DATA+1],buf[CB_MSG_OFFSET_DATA+0]),
                                                                   mk16bit( buf[CB_MSG_OFFSET_DATA+3],buf[CB_MSG_OFFSET_DATA+2]),
                                                                   mk16bit( buf[CB_MSG_OFFSET_DATA+5],buf[CB_MSG_OFFSET_DATA+4]));
            }
        } else if (desc->type == Cannybots::CB_INT16_2) {
            //NSLog(@"is CB_INT16_2");
            //NSLog(@"block @ %x", desc.data);
            if (desc->data) {
                ((__bridge cb_bridged_callback_int16_2)desc->data)( mk16bit( buf[CB_MSG_OFFSET_DATA+1],buf[CB_MSG_OFFSET_DATA+0]),
                                                                   mk16bit( buf[CB_MSG_OFFSET_DATA+3],buf[CB_MSG_OFFSET_DATA+2]));
            }
        } else if (desc->type == Cannybots::CB_INT16_1) {
            //NSLog(@"is CB_INT16_2");
            //NSLog(@"block @ %x", desc.data);
            if (desc->data) {
                ((__bridge cb_bridged_callback_int16_1)desc->data)( mk16bit( buf[CB_MSG_OFFSET_DATA+1],buf[CB_MSG_OFFSET_DATA+0]));
            }
        } else if (desc->type == Cannybots::CB_STRING) {
            //NSLog(@"is CB_STRING");
            //NSLog(@"block @ %x", desc.data);
            if (desc->data) {
                ((__bridge cb_bridged_callback_string)desc->data)( (const char*) &buf[CB_MSG_OFFSET_DATA+0]);
            }
        } else {
            NSLog(@"ERROR: unrecognised desc.type in didReceiveData (%d)", commandId);
        }
        
    }
        }
}


//////////////////////////////////////////////////////////////////////////////////////
// Simple Key/Value API

typedef __attribute__((__packed__)) struct {
    uint8_t botId;
    char    varName[5];
} msgHeader_t;

- (void) didReceiveData:(NSData *)data {
    @synchronized(self) {
        long      len = [data length];
        uint8_t * buf = (uint8_t*)[data bytes];
        
        //NSString* hexString = [data hexRepresentationWithSpaces:YES];
        //NSLog(@"Received: %@", hexString);
        
        if (len >=20) {
            uint8_t botId   = buf[0];
            char    varName[5+1] = {0};
            memcpy(varName, &buf[1] , 5);
            varName[5]=0;
            
            NSString* varNameObj = [[NSString stringWithUTF8String:varName]  stringByTrimmingCharactersInSet:[NSCharacterSet whitespaceCharacterSet]];
            
            if (!varNameObj) {
                NSLog(@"Couldn't parse name   : %s", varName);
                return;
            }
            NSData* varData =[ NSData dataWithBytes:&buf[6] length:14];
            // TODO: pass bot id
            [[NSNotificationCenter defaultCenter] postNotificationName:varNameObj object:varData];
        }
    }
}

- (void) writeInt:(int16_t)p1 forVariable:(NSString*)varName {
    char msg[21] = {0};
    snprintf(msg, sizeof(msg), "%c%5.5s%c%c", 0, [varName UTF8String], highByte(p1), lowByte(p1));
    
    NSData *data = [NSData dataWithBytesNoCopy:msg length:sizeof(msg)-1 freeWhenDone:NO];
    [bsle sendData:data];
}

@end
