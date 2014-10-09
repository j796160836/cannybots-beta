//
//  BrainDashTuningViewController.m
//  BrainDash
//
//  Created by Wayne Keenan on 04/06/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

#import "BrainDashTuningViewController.h"
#import "CannybotsController.h"

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
    [[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(updateVariable:) name:@"IRVAL"  object:nil];
    [[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(updateVariable:) name:@"PID" object:nil];
    CannybotsController* cb = [CannybotsController sharedInstance];
    [cb writeInt:0 forVariable:@"GETCF"];
    [cb writeInt:1 forVariable:@"SNDIR"];

}


-(void)updateVariable:(NSNotification *)notification {
    NSString* name    = [notification name];
    NSData* data      = [notification object];
    const char *bytes = [data bytes];
    
    //NSString* hexString = [data hexRepresentationWithSpaces:YES];
    //NSLog(@"UpVarRcv: %@", hexString);
    
    if ([name isEqualToString:@"IRVAL"]) {
        int ir1= CFSwapInt16BigToHost(( (int16_t*) bytes)[0]);
        int ir2= CFSwapInt16BigToHost(( (int16_t*) bytes)[1]);
        int ir3= CFSwapInt16BigToHost(( (int16_t*) bytes)[2]);
        _IRReading1.text = [NSString stringWithFormat:@"%d", ir1 ];
        _IRReading2.text = [NSString stringWithFormat:@"%d", ir2 ];
        _IRReading3.text = [NSString stringWithFormat:@"%d", ir3 ];
        
        _ir1Level.progress = (1.0/1024.0) * ir1;
        _ir2Level.progress = (1.0/1024.0) * ir1;
        _ir3Level.progress = (1.0/1024.0) * ir1;
        
    } else if ([name isEqualToString:@"PID"]) {
        _pTextField.text = [NSString stringWithFormat:@"%d", CFSwapInt16BigToHost(( (int16_t*) bytes)[0]) ];;
        _dTExtField.text = [NSString stringWithFormat:@"%d", CFSwapInt16BigToHost(( (int16_t*) bytes)[1]) ];;
        _pStepper.value = [[_pTextField text] intValue];
        _dStepper.value = [[_dTExtField text] intValue];

        
    } else {
        NSLog(@"Un recognised variable: %@", name);
    }
}

- (void) viewWillDisappear:(BOOL)animated {
    CannybotsController* cb = [CannybotsController sharedInstance];
    [cb writeInt:0 forVariable:@"SNDIR"];

    [[NSNotificationCenter defaultCenter] removeObserver:self name:@"IRVAL" object:nil];
    [[NSNotificationCenter defaultCenter] removeObserver:self name:@"PID" object:nil];
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

@implementation UIProgressView (customView)
- (CGSize)sizeThatFits:(CGSize)size {
    CGSize newSize = CGSizeMake(self.frame.size.width,9);
    return newSize;
}
@end

