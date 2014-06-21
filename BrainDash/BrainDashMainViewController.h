//
//  BrainDashMainViewController.h
//  BrainDash
//
//  Created by Wayne Keenan on 16/04/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//


#import "TheBrain.h"
#import "HelloWorldScene.h"
#import <cocos2d.h>

@interface BrainDashMainViewController : UIViewController <BrainDashFlipsideViewControllerDelegate, NTCommandReceiver, CCDirectorDelegate>

@property (weak, nonatomic) IBOutlet UIView *joypadView;
@property (weak, nonatomic) IBOutlet UIView *gestureView;

@property (weak, nonatomic) IBOutlet UILabel *statusLabel;

@property (weak, nonatomic) IBOutlet UISlider *speedSlider;
- (IBAction)tapDetected:(UITapGestureRecognizer *)sender;
- (IBAction)rotationDetected:(UIRotationGestureRecognizer *)sender;
- (IBAction)pinchDetected:(UIPinchGestureRecognizer *)sender;
- (IBAction)swipeDetected:(UISwipeGestureRecognizer *)sender;
- (IBAction)longPressDetected:(UILongPressGestureRecognizer *)sender;
- (IBAction)panDetected:(UIPanGestureRecognizer *)sender;

@property (strong, nonatomic) IBOutlet UIPinchGestureRecognizer *pinchRecognizer;
- (IBAction)speedChanged:(id)sender;
@property (weak, nonatomic) IBOutlet UISegmentedControl *modeSegment;

@end
