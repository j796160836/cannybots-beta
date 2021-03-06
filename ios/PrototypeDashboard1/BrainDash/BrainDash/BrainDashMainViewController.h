//
//  BrainDashMainViewController.h
//  BrainDash
//
//  Created by Wayne Keenan on 16/04/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//


#import "HelloWorldScene.h"
#import <cocos2d.h>
#import "NSData+hex.h"
#import "BrainSpeakBLE.h"


@interface BrainDashMainViewController : UIViewController <CCDirectorDelegate>

@property (weak, nonatomic) IBOutlet UIView *joypadView;
@property (weak, nonatomic) IBOutlet UIView *gestureView;

@property (weak, nonatomic) IBOutlet UILabel *statusLabel;


@property (weak, nonatomic) IBOutlet UISlider *leftSpeedSlider;
@property (weak, nonatomic) IBOutlet UISlider *rightSpeedSlider;


- (IBAction)tapDetected:(UITapGestureRecognizer *)sender;
- (IBAction)rotationDetected:(UIRotationGestureRecognizer *)sender;
- (IBAction)pinchDetected:(UIPinchGestureRecognizer *)sender;
- (IBAction)swipeDetected:(UISwipeGestureRecognizer *)sender;
- (IBAction)longPressDetected:(UILongPressGestureRecognizer *)sender;
- (IBAction)panDetected:(UIPanGestureRecognizer *)sender;

@property (strong, nonatomic) IBOutlet UIPinchGestureRecognizer *pinchRecognizer;
@property (weak, nonatomic) IBOutlet UISegmentedControl *modeSegment;

@property (weak, nonatomic) IBOutlet UITextField *lapCounterTextField;
@property (weak, nonatomic) IBOutlet UITextField *lapTimeTextField;
@property (weak, nonatomic) IBOutlet UITextField *bestLapTimeTextField;
@end
