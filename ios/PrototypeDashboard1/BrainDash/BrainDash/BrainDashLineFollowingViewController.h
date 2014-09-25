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

- (void)startUpdateAccelerometer;
- (void)stopUpdate;

@end
