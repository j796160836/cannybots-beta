
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
- (void) didReceiveSonarPing:(uint16_t) distance forId:(uint8_t)_id;
@end

@protocol NTIRReceiver
- (void) didReceiveIR:(uint16_t)code forId:(uint8_t)_id withType:(uint16_t)type;
@end






@interface TheBrain : NSObject <NTReceiver>

@property(nonatomic, retain)    id<NTSender> writer;
@property(nonatomic, retain)    id<NTDebugReceiver> debugDelegate;

+ (id)sharedInstance;


- (void) getInfo;
- (void) ping:(uint8_t)_id;
- (void) setServoSpeed:(uint16_t)speed forId:(uint8_t)_id;

@end
    