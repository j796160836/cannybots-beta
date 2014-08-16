//
//  BrainDashTuningViewController.m
//  BrainDash
//
//  Created by Wayne Keenan on 04/06/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import "BrainDashTuningViewController.h"

#import "CannybotsController.h"
#import "CannybotsRacerGlu.h"

@interface BrainDashTuningViewController () {
}

@property (nonatomic, assign) id currentResponder;

@end

@implementation BrainDashTuningViewController

- (id)initWithNibName:(NSString *)nibNameOrNil bundle:(NSBundle *)nibBundleOrNil
{
    self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
    if (self) {

    }
    return self;
}

- (void)viewDidLoad
{
    [super viewDidLoad];
    UITapGestureRecognizer *singleTap = [[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(resignOnTap:)];
    [singleTap setNumberOfTapsRequired:1];
    [singleTap setNumberOfTouchesRequired:1];
    [self.view addGestureRecognizer:singleTap];
    [self revertPressed:nil];

}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

/*
#pragma mark - Navigation

// In a storyboard-based application, you will often want to do a little preparation before navigation
- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender
{
    // Get the new view controller using [segue destinationViewController].
    // Pass the selected object to the new view controller.
}
*/


- (void) viewDidAppear:(BOOL)animated {
    [[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(updateVariable:) name:@"IR_1"  object:nil];
    [[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(updateVariable:) name:@"IR_2"  object:nil];
    [[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(updateVariable:) name:@"IR_3"  object:nil];
    [[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(updateVariable:) name:@"PID_P" object:nil];
    [[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(updateVariable:) name:@"PID_D" object:nil];
    CannybotsController* cb = [CannybotsController sharedInstance];
    [cb writeInt:0 forVariable:@"GETCF"];
    [cb writeInt:1 forVariable:@"SNDIR"];

}

-(void)updateVariable:(NSNotification *)notification {
    NSString* name = [notification name];
    NSNumber* val  = [notification object];
    if ([name isEqualToString:@"IR_1"]) {
        _IRReading1.text = [val stringValue];
    } else if ([name isEqualToString:@"IR_2"]) {
        _IRReading2.text = [val stringValue];
    } else if ([name isEqualToString:@"IR_3"]) {
        _IRReading3.text = [val stringValue];
    } else if ([name isEqualToString:@"PID_P"]) {
        _pTextField.text = [val stringValue];
    } else if ([name isEqualToString:@"PID_D"]) {
        _dTExtField.text = [val stringValue];
    }
}

- (void) viewWillDisappear:(BOOL)animated {
    CannybotsController* cb = [CannybotsController sharedInstance];
    [cb writeInt:0 forVariable:@"SNDIR"];

    [[NSNotificationCenter defaultCenter] removeObserver:self name:@"IR_1" object:nil];
    [[NSNotificationCenter defaultCenter] removeObserver:self name:@"IR_2" object:nil];
    [[NSNotificationCenter defaultCenter] removeObserver:self name:@"IR_3" object:nil];
    [[NSNotificationCenter defaultCenter] removeObserver:self name:@"PID_P" object:nil];
    [[NSNotificationCenter defaultCenter] removeObserver:self name:@"PID_D" object:nil];
}

-(IBAction)textFieldReturn:(id)sender
{
    [sender resignFirstResponder];
}

- (BOOL)textField:(UITextField *)textField shouldChangeCharactersInRange:(NSRange)range replacementString:(NSString *)string {
    if (textField==_pTextField) {
        _pStepper.value = [[textField text] intValue];
    } else if (textField==_iTextField) {
        _iStepper.value = [[textField text] intValue];
    } else if (textField==_dTExtField) {
        _dStepper.value = [[textField text] intValue];
    }
    return true;
}

- (void)textFieldDidBeginEditing:(UITextField *)textField {
    self.currentResponder = textField;
}

- (void)resignOnTap:(id)iSender {
    [self.currentResponder resignFirstResponder];
}

- (IBAction)pStepperValueChanged:(UIStepper *)sender {
    _pTextField.text = [NSString stringWithFormat:@"%d",(int16_t)sender.value];
}
- (IBAction)iStepperValueChanged:(UIStepper *)sender {
}
- (IBAction)dStepperValueChanged:(UIStepper *)sender {
    _dTExtField.text = [NSString stringWithFormat:@"%d",(int16_t)sender.value];

}
// LED brightness
- (IBAction)ledBrightnessValueChanged:(UIStepper *)sender {
}
- (IBAction)ledBrightnessTextFieldValueChanged:(id)sender {
}
- (IBAction)deviceIdValueChanged:(id)sender {
}
- (IBAction)ledColourSelected:(id)sender {
}


- (IBAction)savePressed:(id)sender {
    CannybotsController* cb = [CannybotsController sharedInstance];
    
    [cb writeInt:[_pTextField.text intValue] forVariable:@"PID_P"];
    [cb writeInt:[_dTExtField.text intValue] forVariable:@"PID_D"];
}
- (IBAction)revertPressed:(id)sender {
    CannybotsController* cb = [CannybotsController sharedInstance];
    [cb writeInt:0 forVariable:@"GETCF"];
}


@end
