
#import "BrainDashFlipsideViewController.h"
#import "UARTPeripheral.h"

@protocol NTDebugReceiver
- (void) didReceiveDebug:(NSString*)msg;
@end


@protocol NTSender
- (void) sendData:(NSData*)data;
@end

@protocol NTReceiver
- (void) didReceiveData:(NSData*)data;
@end

@protocol NTSonarReceiver
- (void) didReceiveSonarPing:(int16_t) distance forId:(uint8_t)_id;
@end

@protocol NTIRReceiver
- (void) didReceiveIR:(int16_t)code forId:(uint8_t)_id withType:(uint16_t)type;
@end


@protocol NTMicrophoneReceiver
- (void) didReceiveMicrophoneLevel:(int16_t)level forId:(uint8_t)_id;
@end




@interface TheBrain : NSObject <NTReceiver>

@property(nonatomic, retain)    id<NTSender>        writer;

@property(nonatomic, retain)    id<NTDebugReceiver> debugDelegate;
@property(nonatomic, retain)    id<NTSonarReceiver> sonarDelegate;
@property(nonatomic, retain)    id<NTIRReceiver>    infraRedDelegate;
@property(nonatomic, retain)    id<NTMicrophoneReceiver>    microphoneDelegate;

+ (id)sharedInstance;


- (void) getInfo;
- (void) ping:(uint8_t)_id;
- (void) playDitty:(uint8_t)_id;
- (void) playTone:(uint8_t)duration tone:(int16_t)tone;

// AX 12 interface

- (void) setServoSpeed:(int16_t)speed forId:(uint8_t)_id;
- (void) setServoVelocity:(int16_t)velocity forId:(uint8_t)_id;
- (void) setServoPosition:(int16_t)position forId:(uint8_t)_id;
- (void) setEndlessTurnMode:(uint8_t)endlessMode forId:(uint8_t)_id;


@end
    