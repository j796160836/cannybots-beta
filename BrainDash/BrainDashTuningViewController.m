//
//  BrainDashTuningViewController.m
//  BrainDash
//
//  Created by Wayne Keenan on 04/06/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import "BrainDashTuningViewController.h"

#import "CannybotsLineFollowing.h"

@interface BrainDashTuningViewController () {
    TheBrain            *theBrain;

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
    theBrain = [TheBrain sharedInstance];
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
    theBrain.appDelegate = self;
}

- (void) viewWillDisappear:(BOOL)animated {
    theBrain.appDelegate = nil;

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
    
    lfconfig cfg;
    
    cfg.deviceId = [[_deviceIdTextField text] intValue];
    cfg.p = [[_pTextField text] intValue];
    cfg.i = [[_iTextField text] intValue];
    cfg.d = [[_dTExtField text] intValue];
    cfg.rgbCol = _ledColourSegment.selectedSegmentIndex;
    cfg.rgbBrightness = [_ledBrightnessTextField.text intValue];
    cfg.IRBias[0] = [_IRBias1TextField.text intValue];
    cfg.IRBias[1] = [_IRBias2TextField.text intValue];
    cfg.IRBias[2] = [_IRBias3TextField.text intValue];
    
    [theBrain lf_cfg_write_config:cfg];
    
   
}
- (IBAction)revertPressed:(id)sender {
    [theBrain lf_cfg_get_config];

}

- (void) didReceiveCommand:(uint8_t)cmd forId:(uint8_t)_id withArg:(int16_t)arg1 {
    
    NSLog(@"CMD=%d, id=%d, arg=%d", cmd, _id, arg1);
    if (NT_CMD_LINEFOLLOW_CONFIG_GET == cmd) {
        if (LINEFOLLOW_CFG_PID_P == _id){
            _pTextField.text = [NSString stringWithFormat:@"%d", arg1];
            _pStepper.value = arg1;
        } else if (LINEFOLLOW_CFG_PID_I == _id){
            _iTextField.text = [NSString stringWithFormat:@"%d", arg1];
            _iStepper.value = arg1;
        } else if (LINEFOLLOW_CFG_PID_D == _id){
            _dTExtField.text = [NSString stringWithFormat:@"%d", arg1];
            _dStepper.value = arg1;
        } else if ( LINEFOLLOW_CFG_DEVICE_ID == _id){
            NSLog(@"device id = %d", arg1);
            _deviceIdTextField.text = [NSString stringWithFormat:@"%d", arg1];
        } else if ( LINEFOLLOW_CFG_RGB_COLOUR == _id){
            NSLog(@"RGB col= %d", arg1);
            _ledColourSegment.selectedSegmentIndex = arg1;
        } else if ( LINEFOLLOW_CFG_RGB_BRIGHTNESS == _id){
            NSLog(@"RGB bri= %d", arg1);
            _ledBrightnessTextField.text =[NSString stringWithFormat:@"%d", arg1];
            _ledBrightnessStepper.value = arg1;
        } else if ( LINEFOLLOW_CFG_IR_BIAS_1 == _id){
            NSLog(@"IR1= %d", arg1);
            _IRBias1TextField.text =[NSString stringWithFormat:@"%d", arg1];
        } else if ( LINEFOLLOW_CFG_IR_BIAS_2 == _id){
            NSLog(@"IR2= %d", arg1);
            _IRBias2TextField.text =[NSString stringWithFormat:@"%d", arg1];
        } else if ( LINEFOLLOW_CFG_IR_BIAS_3 == _id){
            NSLog(@"IR3= %d", arg1);
            _IRBias3TextField.text =[NSString stringWithFormat:@"%d", arg1];
        } else {
            NSLog(@"WARNING: unrecognised CFG value: %d",_id);
        }
    }
}


@end
