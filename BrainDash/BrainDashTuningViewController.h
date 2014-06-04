//
//  BrainDashTuningViewController.h
//  BrainDash
//
//  Created by Wayne Keenan on 04/06/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import <UIKit/UIKit.h>

@interface BrainDashTuningViewController : UIViewController
@property (weak, nonatomic) IBOutlet UITextField *pTextField;
@property (weak, nonatomic) IBOutlet UITextField *iTextField;
@property (weak, nonatomic) IBOutlet UITextField *dTExtField;
@property (weak, nonatomic) IBOutlet UITextField *ledBrightnessTextField;
@property (weak, nonatomic) IBOutlet UISegmentedControl *ledColourSegment;
- (IBAction)ledColourSelected:(id)sender;

-(IBAction)textFieldReturn:(id)sender;

@end
