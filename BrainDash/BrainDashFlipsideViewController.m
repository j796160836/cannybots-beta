//
//  BrainDashFlipsideViewController.m
//  BrainDash
//
//  Created by Wayne Keenan on 16/04/2014.
//  Copyright (c) 2014 CannyBots. All rights reserved.
//

// cocos 2d embedding:  http://www.raywenderlich.com/4817/how-to-integrate-cocos2d-and-uikit

#import "BrainDashFlipsideViewController.h"

@interface BrainDashFlipsideViewController () <NTDebugReceiver>{
TheBrain            *theBrain;
}
@end

@implementation BrainDashFlipsideViewController

- (void) viewWillAppear:(BOOL)animated
{
    [self.navigationController setNavigationBarHidden:YES animated:NO];
    [super viewWillAppear:animated];
}


// Add to end of viewDidUnload

- (void)viewDidLoad
{
    [super viewDidLoad];
    theBrain = [TheBrain sharedInstance];
    
    theBrain.debugDelegate = self;
    theBrain.sonarDelegate = self;
    theBrain.infraRedDelegate = self;
    theBrain.microphoneDelegate = self;

}

- (void) viewWillDisappear:(BOOL)animated {
    theBrain.debugDelegate = nil;
}



- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    theBrain=nil;
}

#pragma mark - Actions

- (IBAction)done:(id)sender
{
    [self.delegate flipsideViewControllerDidFinish:self];
}


- (void) didReceiveDebug:(NSString*)msg {
    dispatch_async(dispatch_get_main_queue(), ^(void) {
        NSString *newText = [NSString stringWithFormat: @"%@%@\n", _debugTextView.text, msg];
        
        _debugTextView.text = newText;
        
        [_debugTextView scrollRangeToVisible:NSMakeRange(newText.length, 0)];
    });
    //_debugTextView.text = [_debugTextView.text stringByAppendingString:msg];
    //NSLog(@"didReceiveDebug: %@", msg);
}


- (void) didReceiveSonarPing:(uint16_t) distance forId:(uint8_t)_id {
    
    if (_id ==1 ) {
        _sonar1Label.text= [NSString stringWithFormat:@"%d", distance];
    } else {
        _sonar2Label.text= [NSString stringWithFormat:@"%d", distance];
    }
}

- (void) didReceiveIR:(int16_t)code forId:(uint8_t)_id withType:(uint16_t)type {
    static uint16_t lasstCode = 0;
    if (code!=-1)
        lasstCode = code;
    _irLabel.text = [NSString stringWithFormat:@"%X %@", lasstCode, code==-1? @"R":@""];

}

- (void) didReceiveMicrophoneLevel:(int16_t)level forId:(uint8_t)_id {
    _micLabel.text = [NSString stringWithFormat:@"%d", level];
}

- (IBAction)scanPressed:(id)sender {
}


- (IBAction)getInfo:(id)sender{
    [theBrain getInfo];
}

- (IBAction)forward:(id)sender{
    [theBrain setServoSpeed:600 forId:1];
    [theBrain setServoSpeed:600 forId:2];
}

- (IBAction)backward:(id)sender{
    [theBrain setServoSpeed:-600 forId:1];
    [theBrain setServoSpeed:-600 forId:2];
}

- (IBAction)left:(id)sender{
    [theBrain setServoSpeed:600 forId:1];
    [theBrain setServoSpeed:-600 forId:2];
}

- (IBAction)right:(id)sender{
    [theBrain setServoSpeed:-600 forId:1];
    [theBrain setServoSpeed:600 forId:2];
}


- (IBAction)stop:(id)sender{
    [theBrain setServoSpeed:0 forId:1];
    [theBrain setServoSpeed:0 forId:2];
}


- (IBAction)ping:(id)sender{
    [theBrain ping:1];
}

- (IBAction)playDitty:(id)sender{
    [theBrain playDitty:0];
}

- (IBAction)soundHorn:(id)sender{
    [theBrain playTone:8 tone:523]; // C5
}



@end
