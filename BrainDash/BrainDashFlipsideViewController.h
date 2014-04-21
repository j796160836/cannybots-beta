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

@interface BrainDashFlipsideViewController : UIViewController //<NTDebugReceiver>

@property (weak, nonatomic) id <BrainDashFlipsideViewControllerDelegate> delegate;
@property (weak, nonatomic) IBOutlet UITextView *debugTextView;

- (IBAction)done:(id)sender;

@end
