
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

@protocol NTCommandReceiver
- (void) didReceiveCommand:(uint8_t)cmd forId:(uint8_t)_id withArg:(int16_t)arg1;
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
@property(nonatomic, retain)    id<NTCommandReceiver> appDelegate;
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


// APP:  Line following
//TODO: move this to seprate class
// Config

- (void) lf_cfg_get_pid_p;
- (void) lf_cfg_get_pid_i;
- (void) lf_cfg_get_pid_d;

typedef struct lfconfig_t {
    int16_t p,i,d;
    uint8_t rgbCol, rgbBrightness;
} lfconfig;

- (void) lf_cfg_write_config:(lfconfig)config;
- (void) lf_cfg_get_config;


// Actions
- (void) lf_go;
- (void) lf_stop;
- (void) lf_left;
- (void) lf_right;
- (void) lf_switchNextJunction;
- (void) lf_speed:(int16_t)speed;
- (void) lf_setMotorSpeed:(int16_t)speed forId:(uint8_t)sid;

@end
    