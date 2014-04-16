//
//  BrainDashFlipsideViewController.h
//  BrainDash
//
//  Created by Wayne Keenan on 16/04/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import <UIKit/UIKit.h>

@class BrainDashFlipsideViewController;

@protocol BrainDashFlipsideViewControllerDelegate
- (void)flipsideViewControllerDidFinish:(BrainDashFlipsideViewController *)controller;
@end

@interface BrainDashFlipsideViewController : UIViewController

@property (weak, nonatomic) id <BrainDashFlipsideViewControllerDelegate> delegate;

- (IBAction)done:(id)sender;

@end
