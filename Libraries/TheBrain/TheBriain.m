#import "TheBrain.h"

#import "BrainSpeakBLE.h"
#import "NTProtocol.h"


@interface TheBrain () {
    CBCentralManager    *cm;
    UARTPeripheral      *currentPeripheral;
}


@end


@implementation TheBrain

#pragma mark Singleton Methods

+ (id)sharedInstance {
    static TheBrain *sharedBrain = nil;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
        sharedBrain = [[self alloc] init];
    });
    return sharedBrain;
}

- (id)init {
    if (self = [super init]) {
        //someProperty = [[NSString alloc] initWithString:@"Default Property Value"];
    }
    return self;
}

- (void)dealloc {
    // Should never be called, but just here for clarity really.
}



////////////////////////////////////////////////////


- (void) sendData:(NSData*)data {
    [self.writer sendData:data];
}
void dumpCmd(const NT_cmd cmd) {
    NSLog(@"Cat=%d, Cmd=%d, ID=%d, argc=%d, arg1=%d", cmd.category, cmd.command, cmd.id, cmd.argc, cmd.arg1);
}


- (void) didReceiveData:(NSData *)data {
    long      len = [data length];
    uint8_t * buf = (uint8_t*)[data bytes];
    
#ifdef NT_DEBUG
    NSString* hexString = [newData hexRepresentationWithSpaces:YES];
    NSLog(@"Received: %@", hexString);
#endif
    
    
    int status = NT_validateMsg(buf, len);
    
    // Not a valid message? Assume it's debug messages and just log it.
    if (NT_STATUS_OK != status) {
        //NSLog(@"Msg failed msg validation, status code: %c", status);
        NSString* str =   [[NSString alloc] initWithData:data encoding:NSASCIIStringEncoding];
        //NSLog(@"BLE_DBG: %@", str);
        if (self.debugDelegate) {
            [self.debugDelegate didReceiveDebug:str];
        }
        return;
    }
    
    
    NT_cmd cmd0,cmd1,cmd2,cmd3;
    //TODO: get number of commands!
    status = NT_extractCommand(buf, 0, &cmd0);
    status = NT_extractCommand(buf, 1, &cmd1);
    status = NT_extractCommand(buf, 2, &cmd2);
    status = NT_extractCommand(buf, 3, &cmd3);
    dumpCmd(cmd0);
    dumpCmd(cmd1);

}




///////////////////////////////////////////////////////////////////////




- (void)getInfo {
    
    uint8_t msg[NT_MSG_SIZE] = {
        NT_DEFAULT_MSG_HEADER(),
        NT_CREATE_CMD1(NT_CAT_AX12, NT_CMD_AX12_DISCOVER, 0, 0),
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP
    };
    NT_MSG_CALC_CRC(msg);
    
    NSData *data = [NSData dataWithBytesNoCopy:(void*)msg length:NT_MSG_SIZE freeWhenDone:NO];
    
    [self sendData:data];
}





- (void)ping:(uint8_t)_id{
    uint8_t msg[NT_MSG_SIZE] = {
        NT_DEFAULT_MSG_HEADER(),
        NT_CREATE_CMD1(NT_CAT_AX12, NT_CMD_AX12_TESTS, 2, 0),
        NT_CREATE_CMD_NOP,
        //NT_CREATE_CMD1(NT_CAT_SONAR, NT_CMD_SONAR_PING, 1, 0)
        //NT_CREATE_CMD1(NT_CAT_SONAR, NT_CMD_SONAR_PING, 2, 0)
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP
    };
    NT_MSG_CALC_CRC(msg);
    
    
    NSData *data = [NSData dataWithBytesNoCopy:(void*)msg length:NT_MSG_SIZE freeWhenDone:NO];
    [self sendData:data];
}




- (void) setServoSpeed:(uint16_t)speed forId:(uint8_t)sid {
    
    uint8_t msg[NT_MSG_SIZE] = {
        NT_DEFAULT_MSG_HEADER(),
        NT_CREATE_CMD1(NT_CAT_AX12, NT_CMD_AX12_SET_ENDLESS_TURN_SPEED, sid, speed),
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP
    };
    NT_MSG_CALC_CRC(msg);
    
    NSData *data = [NSData dataWithBytesNoCopy:(void*)msg length:NT_MSG_SIZE freeWhenDone:NO];
    [self sendData:data];
}




@end
