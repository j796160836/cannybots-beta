//
//  BrainDashLineFollowingViewController.m
//  BrainDash
//
//  Created by Wayne Keenan on 04/06/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//



#import "BrainDashLineFollowingViewController.h"
#import <CannybotsController.h>
#import "BrainSpeakBLE.h"

#import "CannybotsRacerGlu.h"

@interface BrainDashLineFollowingViewController ()
{
    bool manualModePressed ;
}
@end

@implementation BrainDashLineFollowingViewController

@synthesize mManager = _mManager;
@synthesize referenceAttitude = _referenceAttitude;


- (id)initWithNibName:(NSString *)nibNameOrNil bundle:(NSBundle *)nibBundleOrNil
{
    self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
    if (self) {
        _mManager = [[CMMotionManager alloc] init];
        _referenceAttitude = nil;
        manualModePressed = false;
    }
    return self;
}



cb_app_config cbr_settings;

- (void)viewDidLoad
{
    
    //cannybotsRacerGlu_setupConfig(&cbr_settings);

    [super viewDidLoad];
}

- (void) viewDidAppear:(BOOL)animated {
    [self startUpdateAccelerometer ];
}

- (void) viewWillDisappear:(BOOL)animated {
    [self stopUpdate];
    
}   

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

/*
#pragma mark - Navigation

// In a storyboard-based application, you will often want to do a little preparation before navigation
- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender
{
    // Get the new view controller using [segue destinationViewController].
    // Pass the selected object to the new view controller.
}3
*/
- (IBAction)swapLanes:(id)sender {
    CannybotsController* cb = [CannybotsController sharedInstance];
    
    //NSArray* configDescriptors =
    [cb getConfigParameterList]; // result returned async

    //NSLog(@"Config Params: %@",configDescriptors);
}

#define PAUSE_HACK   [NSThread sleepForTimeInterval: 0.2];

- (IBAction)goLeft:(id)sender {

    return;
    CannybotsController* cb = [CannybotsController sharedInstance];

    //[cb setConfigParameter_UINT32:&cfg_type p1:0xFF00FF00]; PAUSE_HACK;
    [cb setConfigParameter_UINT16:&cfg_id p1:0x0707];    PAUSE_HACK;
    [cb setConfigParameter_UINT16:&cfg_version p1:0x0101];    PAUSE_HACK;
    //[cb setConfigParameter_UINT32:&cfg_authentication_pin p1:0x01020304]; PAUSE_HACK;

    
    [cb setConfigParameter_UINT8:&cfg_battery_hasSense p1:0];    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_battery_pin_sense p1:0];    PAUSE_HACK;

    
    [cb setConfigParameter_INT16:&cfg_ir_max   p1:1000];    PAUSE_HACK;
    [cb setConfigParameter_INT16:&cfg_ir_whiteThreshold   p1:700];    PAUSE_HACK;

    
    // TODO: crate convineice for Arduiono pins
    [cb setConfigParameter_UINT8:&cfg_ir_pin_1 p1:24]; PAUSE_HACK; // A6
    [cb setConfigParameter_UINT8:&cfg_ir_pin_2 p1:26]; PAUSE_HACK; // A8
    [cb setConfigParameter_UINT8:&cfg_ir_pin_3 p1:29]; PAUSE_HACK; // A10
    
    [cb setConfigParameter_UINT8:&cfg_ir_bias_1 p1:1]; PAUSE_HACK; // A6
    [cb setConfigParameter_UINT8:&cfg_ir_bias_2 p1:2]; PAUSE_HACK; // A8
    [cb setConfigParameter_UINT8:&cfg_ir_bias_3 p1:3]; PAUSE_HACK; // A10
    

    [cb setConfigParameter_UINT8:&cfg_motorDriver_type p1:0];    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorDriver_driveModePin p1:2];    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorDriver_maxSpeed p1:255];    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorDriver_hasDriveMode p1:1];    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorDriver_hasMotorSense p1:0];    PAUSE_HACK;


    [cb setConfigParameter_UINT8:&cfg_motorA_pin_1 p1:3];    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorA_pin_2 p1:5];    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorA_pin_sense p1:0];    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorA_postiveSpeedisFwd p1:1];    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorA_id p1:0];    PAUSE_HACK;

    [cb setConfigParameter_UINT8:&cfg_motorB_pin_1 p1:6];    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorB_pin_2 p1:9];    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorB_pin_sense p1:0];    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorB_postiveSpeedisFwd p1:1];    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorB_id p1:1];    PAUSE_HACK;

    [cb setConfigParameter_UINT8:&cfg_motor_speedSmoothingDivisions p1:1];    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motor_speedSmoothingMaxDelta p1:255];    PAUSE_HACK;

    [cb setConfigParameter_INT16:&cfg_pid_p   p1:3];    PAUSE_HACK;
    [cb setConfigParameter_INT16:&cfg_pid_i   p1:0];    PAUSE_HACK;
    [cb setConfigParameter_INT16:&cfg_pid_d   p1:1];    PAUSE_HACK;
    [cb setConfigParameter_INT16:&cfg_pid_divisor   p1:10];    PAUSE_HACK;
    [cb setConfigParameter_INT16:&cfg_pid_sampleTime   p1:5];    PAUSE_HACK;


    [cb setConfigParameter_UINT8:&cfg_joystick_xAxisDeadzone p1:50];    PAUSE_HACK;

    [cb setConfigParameter_UINT8:&cfg_cruiseSpeed_defaultSpeed p1:120];    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_cruiseSpeed_manualMaxSpeed p1:255];    PAUSE_HACK;
    
    [cb setConfigParameter_INT16:&cfg_offLineMaxTime   p1:200];    PAUSE_HACK;
    [cb setConfigParameter_UINT16:&cfg_info_printValsInterval p1:1000];    PAUSE_HACK;
    
    [cb setConfigParameter_BOOL:&cfg_debugFlag p1:false];    PAUSE_HACK;

}

- (IBAction)goRight:(id)sender {
    return;
    [self goLeft:sender];
    
    CannybotsController* cb = [CannybotsController sharedInstance];

    [cb setConfigParameter_UINT16:&cfg_id p1:0x0123];    PAUSE_HACK;

    [cb setConfigParameter_UINT8:&cfg_motorA_postiveSpeedisFwd p1:0];    PAUSE_HACK;
    [cb setConfigParameter_UINT8:&cfg_motorB_postiveSpeedisFwd p1:0];    PAUSE_HACK;
}

- (IBAction)stop:(id)sender {
    //NSLog(@"manualMode touch up inside3");
    manualModePressed = false;
}

- (IBAction)go:(id)sender {
    CMDeviceMotion *deviceMotion = self.mManager.deviceMotion;
    CMAttitude *attitude         = deviceMotion.attitude;
    self.referenceAttitude       = attitude;
    
}
- (IBAction)manualModePressed:(UIButton *)sender forEvent:(UIEvent *)event {
    //NSLog(@"manualMode");
    manualModePressed = true;
}

- (IBAction)speedChanged:(UISlider*)sender {
    int speed= (int)(sender.value * 255);
    NSLog(@"Speed = %d", speed);
}

- (BOOL)shouldAutorotateToInterfaceOrientation:(UIInterfaceOrientation)interfaceOrientation {
    return YES;
}

// Core Motion

- (CMMotionManager *)mManager
{
    if (!_mManager) {
        _mManager = [[CMMotionManager alloc] init];
    }
    return _mManager;
}

#define radiansToDegrees(x) (180/M_PI)*x


long map(long x, long in_min, long in_max, long out_min, long out_max);

// @see:  http://wwwbruegge.in.tum.de/lehrstuhl_1/home/98-teaching/tutorials/505-sgd-ws13-tutorial-core-motion
// @see: http://blog.denivip.ru/index.php/2013/07/the-art-of-core-motion-in-ios/?lang=en
// @see: https://github.com/trentbrooks/ofxCoreMotion/blob/master/ofxCoreMotion/src/ofxCoreMotion.mm

static bool sendUpdate =false;
- (void)startUpdateAccelerometer
{
    CMDeviceMotion *deviceMotion = self.mManager.deviceMotion;
    CMAttitude *attitude         = deviceMotion.attitude;
    self.referenceAttitude       = attitude;
    self.mManager.deviceMotionUpdateInterval = 0.1;
    sendUpdate = true;
    
    // Pitch: A pitch is a rotation around a lateral (X) axis that passes through the device from side to side
    // pitch is rotation about the gfx x axis when in portrait mode
    
    // Roll: A roll is a rotation around a longitudinal (Y) axis that passes through the device from its top to bottom
    // roll  is rotation about the gfx y axis when in portrait mode

    // Yaw: A yaw is a rotation around an axis (Z) that runs vertically through the device. It is perpendicular to the body of the device, with its origin at the center of gravity and directed toward the bottom of the device
    
    CMAttitudeReferenceFrame refFrame =CMAttitudeReferenceFrameXArbitraryCorrectedZVertical;
    
    if ( ! ([CMMotionManager availableAttitudeReferenceFrames] & refFrame)) {
        UIAlertView *alert = [[UIAlertView alloc] initWithTitle:@"CoreMotion Error"
                                                        message:@"Request reference frame not available."
                                                       delegate:nil
                                              cancelButtonTitle:@"OK"
                                              otherButtonTitles:nil];
        [alert show];
    }
    
    [self.mManager startDeviceMotionUpdatesUsingReferenceFrame:refFrame toQueue:[NSOperationQueue mainQueue] withHandler:^(CMDeviceMotion *devMotion, NSError *error) {
        
        CMDeviceMotion *deviceMotion = self.mManager.deviceMotion;
        CMAttitude *attitude = deviceMotion.attitude;
        if (self.referenceAttitude != nil ) {
            [attitude multiplyByInverseOfAttitude:self.referenceAttitude];
        }
        
        CMQuaternion quat = attitude.quaternion;
        float roll = radiansToDegrees(atan2(2*(quat.y*quat.w - quat.x*quat.z), 1 - 2*quat.y*quat.y - 2*quat.z*quat.z)) ;
        float pitch = radiansToDegrees(atan2(2*(quat.x*quat.w + quat.y*quat.z), 1 - 2*quat.x*quat.x - 2*quat.z*quat.z));
        float yaw = radiansToDegrees(asin(2*quat.x*quat.y + 2*quat.w*quat.z));

        if (roll < -45)  roll = -45;
        if (roll > 45)   roll = 45;
        //if (pitch < -90) pitch = -90;
        //if (pitch > 90)  pitch = 90;
        if (yaw < -90)   yaw = -90;
        if (yaw > 90)    yaw = 90;
        
        uint8_t xAxis = map(-yaw,  -90, 90, 0, 255);
        uint8_t yAxis = map( roll, -45, 45, 0, 255);
        
        //NSLog(@"%f,%f,%f = %d, %d", roll, pitch, yaw, xAxis, yAxis);
        if (sendUpdate)
            [self sendJoypadUpdate:xAxis y:yAxis b:manualModePressed];

    }];

}

- (void)stopUpdate
{
    if ([self.mManager isDeviceMotionActive] == YES)
    {
        sendUpdate = false;
        [self.mManager stopDeviceMotionUpdates];
        [self sendJoypadUpdate:0x7f y:0x7f b:0];
    }
    
}

- (void) sendJoypadUpdate:(uint8_t)x y:(uint8_t)y b:(uint8_t)b {
    char msg[15] = {0};
    snprintf(msg, sizeof(msg), "%c%c%c", x, y, b);
    NSData *data = [NSData dataWithBytesNoCopy:msg length:3 freeWhenDone:NO];
    BrainSpeakBLE*  bsle = [BrainSpeakBLE sharedInstance];
    [bsle sendData:data];
}



@end
