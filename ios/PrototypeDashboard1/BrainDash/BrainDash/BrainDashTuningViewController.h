//
//  BrainDashTuningViewController.h
//  BrainDash
//
//  Created by Wayne Keenan on 04/06/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "JRTranscriptView.h"

@interface BrainDashTuningViewController : UIViewController <UITextFieldDelegate>
@property (weak, nonatomic) IBOutlet UITextField *pTextField;
@property (weak, nonatomic) IBOutlet UITextField *iTextField;
@property (weak, nonatomic) IBOutlet UITextField *dTExtField;
@property (weak, nonatomic) IBOutlet UIStepper *pStepper;
@property (weak, nonatomic) IBOutlet UIStepper *iStepper;
@property (weak, nonatomic) IBOutlet UIStepper *dStepper;
@property (weak, nonatomic) IBOutlet UITextField *ledBrightnessTextField;
@property (weak, nonatomic) IBOutlet UIStepper *ledBrightnessStepper;
@property (weak, nonatomic) IBOutlet UISegmentedControl *ledColourSegment;
@property (weak, nonatomic) IBOutlet UITextField *deviceIdTextField;
@property (weak, nonatomic) IBOutlet UITextField *IRBias1TextField;
@property (weak, nonatomic) IBOutlet UITextField *IRBias2TextField;
@property (weak, nonatomic) IBOutlet UITextField *IRBias3TextField;
@property (weak, nonatomic) IBOutlet UITextField *IRReading1;
@property (weak, nonatomic) IBOutlet UITextField *IRReading2;
@property (weak, nonatomic) IBOutlet UITextField *IRReading3;
- (IBAction)ledColourSelected:(id)sender;
@property (weak, nonatomic) IBOutlet JRTranscriptView *debugView;
@property (weak, nonatomic) IBOutlet UIProgressView *ir1Level;
@property (weak, nonatomic) IBOutlet UIProgressView *ir2Level;
@property (weak, nonatomic) IBOutlet UIProgressView *ir3Level;

-(IBAction)textFieldReturn:(id)sender;
- (IBAction)pStepperValueChanged:(UIStepper*)sender;
- (IBAction)iStepperValueChanged:(UIStepper*)sender;
- (IBAction)dStepperValueChanged:(UIStepper *)sender;

@end
