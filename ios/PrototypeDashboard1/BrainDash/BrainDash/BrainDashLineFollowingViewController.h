//
//  BrainDashLineFollowingViewController.h
//  BrainDash
//
//  Created by Wayne Keenan on 04/06/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <CoreMotion/CoreMotion.h>

@protocol BrainDashLineFollowingViewControllerDelegate
- (void)updateLabelWithX:(double)accelerometerX Y:(double)accelerometerY;
@end


@interface BrainDashLineFollowingViewController : UIViewController 


@property (nonatomic, strong) CMMotionManager *mManager;
@property (nonatomic, strong) CMAttitude* referenceAttitude;


@property (nonatomic, assign) int  minPitch;
@property (nonatomic, assign) int  maxPitch;
@property (nonatomic, assign) int  minRoll;
@property (nonatomic, assign) int  maxRoll;
@property (nonatomic, assign) int  minYaw;
@property (nonatomic, assign) int  maxYaw;
@property (nonatomic, assign) bool usePitchForXAxis;
@property (nonatomic, assign) bool useYawForXAxis;
@property (weak, nonatomic) IBOutlet UISlider *accelSlider;
@property (weak, nonatomic) IBOutlet UISlider *wheelSlider;
@property (weak, nonatomic) IBOutlet UISlider *handlebarSlider;
@property (weak, nonatomic) IBOutlet UISegmentedControl *controlModeSegment;
@property (weak, nonatomic) IBOutlet UISwitch *invertThrottleSwitch;
@property (weak, nonatomic) IBOutlet UISwitch *invertDirectionSwitch;

- (void)startUpdateAccelerometer;
- (void)stopUpdate;

- (IBAction)accelChanged:(UISlider *)sender;
- (IBAction)wheelChanged:(UISlider *)sender;
- (IBAction)handlebarChanged:(UISlider *)sender;

@end
