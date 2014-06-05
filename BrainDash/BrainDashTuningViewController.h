//
//  BrainDashTuningViewController.h
//  BrainDash
//
//  Created by Wayne Keenan on 04/06/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "TheBrain.h"

@interface BrainDashTuningViewController : UIViewController <NTCommandReceiver, UITextFieldDelegate>
@property (weak, nonatomic) IBOutlet UITextField *pTextField;
@property (weak, nonatomic) IBOutlet UITextField *iTextField;
@property (weak, nonatomic) IBOutlet UITextField *dTExtField;
@property (weak, nonatomic) IBOutlet UIStepper *pStepper;
@property (weak, nonatomic) IBOutlet UIStepper *iStepper;
@property (weak, nonatomic) IBOutlet UIStepper *dStepper;
@property (weak, nonatomic) IBOutlet UITextField *ledBrightnessTextField;
@property (weak, nonatomic) IBOutlet UISegmentedControl *ledColourSegment;
@property (weak, nonatomic) IBOutlet UITextField *deviceIdTextField;
- (IBAction)ledColourSelected:(id)sender;

-(IBAction)textFieldReturn:(id)sender;
- (IBAction)pStepperValueChanged:(UIStepper*)sender;
- (IBAction)iStepperValueChanged:(UIStepper*)sender;
- (IBAction)dStepperValueChanged:(UIStepper *)sender;

@end
