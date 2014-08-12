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

    CannybotsController* cb = [CannybotsController sharedInstance];
    [cb registerHandler:&RACER_PID withBlockFor_INT16_3: ^(int16_t p1, int16_t p2, int16_t p3)
     {
         _pTextField.text = [NSString stringWithFormat:@"%d", p1];
         _iTextField.text = [NSString stringWithFormat:@"%d", p2];
         _dTExtField.text = [NSString stringWithFormat:@"%d", p3];
         _pStepper.value = p1;
         _iStepper.value = p2;
         _dStepper.value = p3;
     }];
    
    [cb registerHandler:&RACER_IRBIAS withBlockFor_INT16_3: ^(int16_t p1, int16_t p2, int16_t p3)
     {
         _IRBias1TextField.text =[NSString stringWithFormat:@"%d", p1];
         _IRBias2TextField.text =[NSString stringWithFormat:@"%d", p2];
         _IRBias3TextField.text =[NSString stringWithFormat:@"%d", p3];
     }];
    

    [cb registerHandler:&RACER_IRVALS withBlockFor_INT16_3: ^(int16_t p1, int16_t p2, int16_t p3)
     {
         _IRReading1.text =[NSString stringWithFormat:@"%d", p1];
         _IRReading2.text =[NSString stringWithFormat:@"%d", p2];
         _IRReading3.text =[NSString stringWithFormat:@"%d", p3];
     }];

    
    [cb registerHandler:&_CB_SYS_LOG withBlockFor_STRING:^(const char* p1)
     {
         //NSLog(@"%s", p1);
         //NSLog(@"%s", p1);
         if (p1) {
            NSString* str = [NSString stringWithUTF8String:p1];
             [self.debugView appendString:str?str:@"nil"];
         }
     }];
    [cb callMethod:&RACER_CONFIG p1:0 p2:0 p3:0];

}

- (void) viewWillDisappear:(BOOL)animated {
    CannybotsController* cb = [CannybotsController sharedInstance];
    [cb deregisterHandler:&RACER_PID];
    [cb deregisterHandler:&RACER_IRBIAS];
    [cb deregisterHandler:&RACER_IRVALS];
    [cb deregisterHandler:&_CB_SYS_LOG]; // TODO: we should have a'restore previous' or implement a 'stack' of pushable/poppable handlers
}

-(IBAction)textFieldReturn:(id)sender
{
    [sender resignFirstResponder];
}

- (BOOL)textField:(UITextField *)textField shouldChangeCharactersInRange:(NSRange)range replacementString:(NSString *)string {
    NSLog(@"shouldChangeCharactersInRange");
    if (textField==_pTextField) {
        _pStepper.value = [[textField text] intValue];
    } else if (textField==_iTextField) {
        _iStepper.value = [[textField text] intValue];
    } else if (textField==_dTExtField) {
        _dStepper.value = [[textField text] intValue];
    } else if (textField==_ledBrightnessTextField) {
        _ledBrightnessStepper.value = [textField.text intValue];
    }
    return true;
}

- (void)textFieldDidBeginEditing:(UITextField *)textField {
    self.currentResponder = textField;
}

- (void)resignOnTap:(id)iSender {
    [self.currentResponder resignFirstResponder];
}



///

- (IBAction)pStepperValueChanged:(UIStepper *)sender {
    _pTextField.text = [NSString stringWithFormat:@"%d",(int16_t)sender.value];
}



- (IBAction)ledBrightnessTextFieldValueChanged:(id)sender {
}

- (IBAction)iStepperValueChanged:(UIStepper *)sender {
    _iTextField.text = [NSString stringWithFormat:@"%d",(int16_t)sender.value];

}

// Device ID

- (IBAction)dStepperValueChanged:(UIStepper *)sender {
    _dTExtField.text = [NSString stringWithFormat:@"%d",(int16_t)sender.value];

}


// LED brightness
- (IBAction)ledBrightnessValueChanged:(UIStepper *)sender {
    _ledBrightnessTextField.text =  [NSString stringWithFormat:@"%d",(int16_t)sender.value];
}




- (IBAction)deviceIdValueChanged:(id)sender {
}


- (IBAction)ledColourSelected:(id)sender {
}



- (IBAction)savePressed:(id)sender {
    CannybotsController* cb = [CannybotsController sharedInstance];

    [cb callMethod:&RACER_PID
                p1:[[_pTextField text] intValue]
                p2:[[_iTextField text] intValue]
                p3:[[_dTExtField text] intValue]];
    
    [cb callMethod:&RACER_IRBIAS
                p1:[[_IRBias1TextField text] intValue]
                p2:[[_IRBias2TextField text] intValue]
                p3:[[_IRBias3TextField text] intValue]];

}
- (IBAction)revertPressed:(id)sender {
    CannybotsController* cb = [CannybotsController sharedInstance];
    [cb callMethod:&RACER_CONFIG p1:0 p2:0 p3:0];
}


@end
