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


@interface BrainDashLineFollowingViewController ()
{
    bool manualModePressed ;
    bool sendUpdate;
    int  currentControlMode;
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

- (void)viewDidLoad {
    [super viewDidLoad];
    //self.wheelSlider.value = 45;
    //self.handlebarSlider.value = 45;
    //self.accelSlider.value = 45;
    //rself.controlModeSegment.selectedSegmentIndex=0;
    
    [self accelChanged:self.accelSlider];
    [self wheelChanged:self.wheelSlider];
    [self handlebarChanged:self.handlebarSlider];
    [self controlModeChanged:self.controlModeSegment];
}

- (void) viewDidAppear:(BOOL)animated {
}

- (void) viewWillDisappear:(BOOL)animated {
    [self stopUpdate];
}   

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
}

// Button presses
- (IBAction)swapLanes:(id)sender {
}

- (IBAction)goLeft:(id)sender {
}

- (IBAction)goRight:(id)sender {
}


- (IBAction)stop:(id)sender {
    manualModePressed = false;
}
- (IBAction)manualModePressed:(UIButton *)sender forEvent:(UIEvent *)event {
    manualModePressed = true;
    [self startUpdateAccelerometer ];
    [self resetReferenceFrameToCurrent];
}

- (IBAction)go:(id)sender {
    [self startUpdateAccelerometer ];
    [self resetReferenceFrameToCurrent];
}


- (IBAction)speedChanged:(UISlider*)sender {
}

- (BOOL)shouldAutorotateToInterfaceOrientation:(UIInterfaceOrientation)interfaceOrientation {
    return YES;
}


// Core Motion

- (void) resetReferenceFrameToCurrent {
    CMDeviceMotion *deviceMotion = self.mManager.deviceMotion;
    CMAttitude *attitude         = deviceMotion.attitude;
    self.referenceAttitude       = attitude;
}

- (CMMotionManager *)mManager
{
    if (!_mManager) {
        _mManager = [[CMMotionManager alloc] init];
    }
    return _mManager;
}

// @see:  http://wwwbruegge.in.tum.de/lehrstuhl_1/home/98-teaching/tutorials/505-sgd-ws13-tutorial-core-motion
// @see: http://blog.denivip.ru/index.php/2013/07/the-art-of-core-motion-in-ios/?lang=en
// @see: https://github.com/trentbrooks/ofxCoreMotion/blob/master/ofxCoreMotion/src/ofxCoreMotion.mm

- (void)startUpdateAccelerometer
{
    CMDeviceMotion *deviceMotion = self.mManager.deviceMotion;
    CMAttitude *attitude         = deviceMotion.attitude;
    self.referenceAttitude       = attitude;
    self.mManager.deviceMotionUpdateInterval = 0.1;
    sendUpdate = true;

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
        [self updateOrientationPitch:pitch roll:roll yaw:yaw];
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

// snesitivity of 'acceleration' (pitch)
- (IBAction)accelChanged:(UISlider *)sender {
    
    self.maxRoll = sender.maximumValue-sender.value;
    self.minRoll = -self.maxRoll;
    NSLog(@"accelChanged=%f", sender.value);
}

// snesitivity of 'wheel' mode (yaw)
- (IBAction)wheelChanged:(UISlider *)sender {
    self.maxYaw = sender.maximumValue-sender.value;
    self.minYaw = -self.maxYaw;
    NSLog(@"wheelChanged=%f", sender.value);
}

// snesitivity of 'handlebars' (roll)
- (IBAction)handlebarChanged:(UISlider *)sender {
    self.maxPitch = sender.maximumValue-sender.value;
    self.minPitch = -self.maxPitch;
    NSLog(@"handlebarChanged=%f", sender.value);

}
- (IBAction)controlModeChanged:(UISegmentedControl *)sender {
    currentControlMode = sender.selectedSegmentIndex;
    NSLog(@"controlModeChanged=%d", currentControlMode);
}

- (void) updateOrientationPitch:(float)pitch roll:(float)roll yaw:(float)yaw {
    
    // Pitch: A pitch is a rotation around a lateral (X) axis that passes through the device from side to side
    // pitch is rotation about the gfx x axis when in portrait mode
    
    // Roll: A roll is a rotation around a longitudinal (Y) axis that passes through the device from its top to bottom
    // roll  is rotation about the gfx y axis when in portrait mode
    
    // Yaw: A yaw is a rotation around an axis (Z) that runs vertically through the device. It is perpendicular to the body of the device, with its origin at the center of gravity and directed toward the bottom of the device
    NSLog(@"INPUT: roll/throttle=(%.0f), pitch/handlebars=(%.0f), yaw/steer=(%.0f)",  roll, pitch, yaw);
    
    
    if (roll < self.minRoll)  roll = self.minRoll;
    if (roll > self.maxRoll)  roll = self.maxRoll;
    if (pitch < self.minPitch) pitch = self.minPitch;
    if (pitch > self.maxPitch) pitch = self.maxPitch;
    if (yaw < self.minYaw)   yaw = self.minYaw;
    if (yaw > self.maxYaw)   yaw = self.maxYaw;
    NSLog(@"CLAMP: roll/throttle=(%.0f), pitch/handlebars=(%.0f), yaw/steer=(%.0f)",  roll, pitch, yaw);

    if (pitch !=0.0)
        pitch = map(pitch, self.minPitch, self.maxPitch, -180, 180);
    if (yaw !=0.0)
        yaw   = map(  yaw, self.minYaw, self.maxYaw,     -180, 180);
    if (roll !=0.0)
        roll  = map( roll, self.minRoll, self.maxRoll,   -180, 180);
    NSLog(@"MAP  : roll/throttle=(%.0f), pitch/handlebars=(%.0f), yaw/steer=(%.0f)",  roll, pitch, yaw);

    
    
    // Forward/Backward

    roll = roll * self.invertThrottleSwitch.on?-1:1;
    
    // Turning
    if (self.invertDirectionSwitch.on) {
        pitch = pitch * -1;
        yaw   = yaw   * -1;
    }
    
    //pitch = map(pitch, _minPitch, _maxPitch, 0, 255);
    //yaw   = map(  yaw, _minYaw, _maxYaw,     0, 255);
    //roll  = map( roll, _minRoll, _maxRoll,   0, 255);

    NSLog(@"INV? : roll/throttle=(%.0f), pitch/handlebars=(%.0f), yaw/steer=(%.0f)",  roll, pitch, yaw);

    float xAxis = 0;
    float yAxis = 0;
    
    switch (currentControlMode) {
        case 0: // Wheel mode,  yaw  is steering wheel
            xAxis = yaw;
            yAxis = roll;
            break;
    
        case 1: // handle bar, roll/twisty handlebar
            xAxis = pitch;
            yAxis = roll;
            break;
        case 2:
            xAxis = (pitch + yaw)/2;
            yAxis = roll;
            break;
        default:
            xAxis = 0;
            yAxis = 0;
            break;

    }
    //pitch = map(pitch, _minPitch, _maxPitch, 0, 255);
    //yaw   = map(  yaw, _minYaw, _maxYaw,     0, 255);
    //roll  = map( roll, _minRoll, _maxRoll,   0, 255);
    uint8_t xAxisByte  = map(xAxis, -180, 180, 0,255);
    uint8_t yAxisByte  = map(yAxis, -180, 180, 0,255);
    NSLog(@"maxR=%d,maxP=%d,maxY=%d\t\t%d,%d",
          self.maxRoll, self.maxPitch, self.maxYaw,
          xAxisByte, yAxisByte);
    
    if (sendUpdate)
        [self sendJoypadUpdate:xAxisByte y:yAxisByte b:manualModePressed];
}
- (void) sendJoypadUpdate:(uint8_t)x y:(uint8_t)y b:(uint8_t)b {
    char msg[15] = {0};
    snprintf(msg, sizeof(msg), "%c%c%c", x, y, b);
    NSData *data = [NSData dataWithBytesNoCopy:msg length:3 freeWhenDone:NO];
    BrainSpeakBLE*  bsle = [BrainSpeakBLE sharedInstance];
    [bsle sendData:data];
}



@end
