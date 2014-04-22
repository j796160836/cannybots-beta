//
//  BrainDashFlipsideViewController.h
//  BrainDash
//
//  Created by Wayne Keenan on 16/04/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "TheBrain.h"



@class BrainDashFlipsideViewController;

@protocol BrainDashFlipsideViewControllerDelegate
- (void)flipsideViewControllerDidFinish:(BrainDashFlipsideViewController *)controller;
@end

@interface BrainDashFlipsideViewController : UIViewController //<NTDebugReceiver, NTSonarReceiver, NTIRReceiver>

@property (weak, nonatomic) id <BrainDashFlipsideViewControllerDelegate> delegate;
@property (weak, nonatomic) IBOutlet UITextView *debugTextView;

- (IBAction)done:(id)sender;
@property (weak, nonatomic) IBOutlet UILabel *sonar1Label;
@property (weak, nonatomic) IBOutlet UILabel *sonar2Label;
@property (weak, nonatomic) IBOutlet UILabel *irLabel;
@property (weak, nonatomic) IBOutlet UILabel *micLabel;

@end
