#import "TheBrain.h"

#import "BrainSpeakBLE.h"
#import "NTProtocol.h"
#import "NSData+hex.h"

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

#define NT_DEBUG 1
- (void) didReceiveData:(NSData *)data {
    long      len = [data length];
    uint8_t * buf = (uint8_t*)[data bytes];
    
#ifdef NT_DEBUG
    NSString* hexString = [data hexRepresentationWithSpaces:YES];
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
    //TODO: get number of commands / TODO: multiple paramter extraction
    status = NT_extractCommand(buf, 0, &cmd0);
    status = NT_extractCommand(buf, 1, &cmd1);
    status = NT_extractCommand(buf, 2, &cmd2);
    status = NT_extractCommand(buf, 3, &cmd3);
    
    // TODO: iterate the cmd[0...2] to reduce logic
    // TODO: cat/cmd macros
    // TODO: i
    if (cmd0.category==NT_CAT_SONAR && cmd0.command==NT_CMD_SONAR_PING_RESULT) {
        if (_sonarDelegate)
            [self.sonarDelegate didReceiveSonarPing:cmd0.arg1  forId:cmd0.id];
    }
    if (cmd1.category==NT_CAT_SONAR && cmd1.command==NT_CMD_SONAR_PING_RESULT) {
        if (_sonarDelegate)
            [self.sonarDelegate didReceiveSonarPing:cmd1.arg1  forId:cmd1.id];
    }
    if (cmd0.category==NT_CAT_IRRECV && cmd0.command==NT_CMD_IRRECV_RESULT) {
        if (_infraRedDelegate)
            [self.infraRedDelegate didReceiveIR:cmd0.arg1 forId:cmd0.id withType:cmd0.arg2];
    }
    if (cmd0.category==NT_CAT_MIC && cmd0.command==NT_CMD_MIC_RESULT) {
        if (_microphoneDelegate)
            [self.microphoneDelegate didReceiveMicrophoneLevel:cmd0.arg1 forId:cmd0.id];
    }
    
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



- (void) playDitty:(uint8_t)_id {
    uint8_t msg[NT_MSG_SIZE] = {
        NT_DEFAULT_MSG_HEADER(),
        NT_CREATE_CMD1(NT_CAT_TONE, NT_CMD_TONE_PLAY_DITTY, _id, 0),
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP
    };
    NT_MSG_CALC_CRC(msg);
    
    NSData *data = [NSData dataWithBytesNoCopy:(void*)msg length:NT_MSG_SIZE freeWhenDone:NO];
    [self sendData:data];
    
    
}
- (void) playTone:(uint8_t)duration tone:(uint16_t)tone {
    uint8_t msg[NT_MSG_SIZE] = {
        NT_DEFAULT_MSG_HEADER(),
        NT_CREATE_CMD1(NT_CAT_TONE, NT_CMD_TONE_PLAY_TONE, duration, tone),
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP
    };
    NT_MSG_CALC_CRC(msg);

    NSData *data = [NSData dataWithBytesNoCopy:(void*)msg length:NT_MSG_SIZE freeWhenDone:NO];
    [self sendData:data];
}


// AX12




- (void) setServoSpeed:(int16_t)speed forId:(uint8_t)sid {
    
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

- (void) setServoVelocity:(int16_t)velocity forId:(uint8_t)_id {
    uint8_t msg[NT_MSG_SIZE] = {
        NT_DEFAULT_MSG_HEADER(),
        NT_CREATE_CMD1(NT_CAT_AX12, NT_CMD_AX12_SET_VEL, _id, velocity),
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP
    };
    NT_MSG_CALC_CRC(msg);
    
    NSData *data = [NSData dataWithBytesNoCopy:(void*)msg length:NT_MSG_SIZE freeWhenDone:NO];
    [self sendData:data];
    
}

- (void) setServoPosition:(int16_t)position forId:(uint8_t)_id {
    uint8_t msg[NT_MSG_SIZE] = {
        NT_DEFAULT_MSG_HEADER(),
        NT_CREATE_CMD1(NT_CAT_AX12, NT_CMD_AX12_SET_POS, _id, position),
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP
    };
    NT_MSG_CALC_CRC(msg);
    
    NSData *data = [NSData dataWithBytesNoCopy:(void*)msg length:NT_MSG_SIZE freeWhenDone:NO];
    [self sendData:data];
    
}

- (void) setEndlessTurnMode:(uint8_t)endlessMode forId:(uint8_t)_id {
    uint8_t msg[NT_MSG_SIZE] = {
        NT_DEFAULT_MSG_HEADER(),
        NT_CREATE_CMD1(NT_CAT_AX12, NT_CMD_AX12_SET_ENDLESS_TURN_MODE, _id, endlessMode),
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP,
        NT_CREATE_CMD_NOP
    };
    NT_MSG_CALC_CRC(msg);
    
    NSData *data = [NSData dataWithBytesNoCopy:(void*)msg length:NT_MSG_SIZE freeWhenDone:NO];
    [self sendData:data];

}



@end
